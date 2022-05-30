#include "CvArmorDetector.hpp"

#include "cv_util.hpp"
#include "cv_armor_util.hpp"
#include "HammingMatch.hpp"

#include "threading/threading.hpp"

#include <algorithm>

namespace rmcv::detect
{
    int32_t lightbar_color(const cv::Mat &img, const RRect &lb)
    {
        auto rect = to_bounding_rect(make_boundingbox(lb));
        rect.x = std::max(0.0f, std::min(rect.x, (float)img.cols));
        rect.y = std::max(0.0f, std::min(rect.y, (float)img.rows));
        if (rect.x + rect.width > img.cols) rect.width = img.cols - rect.x;
        if (rect.y + rect.height > img.rows) rect.height = img.rows - rect.y;
        cv::Mat roi = img(rect);

        std::vector<cv::Mat> chs;
        cv::split(roi, chs);
        int32_t diff = 0;
        for (int i = 0; i < roi.rows; i++)
        {
            uint8_t *r = chs[2].data + i * roi.cols;
            uint8_t *b = chs[0].data + i * roi.cols;
            for (int j = 0; j < roi.cols; j++)
            {
                diff += r[j] - b[j];
            }
        }
        return diff > 0;
    }

    CvArmorDetector::CvArmorDetector()
    {
        armor_classifier_.add_match(1, make_icon_hammimg_match("icon/1.png"));
        armor_classifier_.add_match(2, make_icon_hammimg_match("icon/2.png"));
        armor_classifier_.add_match(3, make_icon_hammimg_match("icon/3.png"));
        armor_classifier_.add_match(4, make_icon_hammimg_match("icon/4.png"));
        armor_classifier_.add_match(5, make_icon_hammimg_match("icon/5.png"));
    }

    int32_t CvArmorDetector::match_armor_icon(const cv::Mat &img, const BoundingBox &armor_bbox, const double &thresh)
    {
        cv::Point2f icon_vertexs_dst[4];
        icon_vertexs_dst[0] = cv::Point2f(0 - 15, 0);
        icon_vertexs_dst[1] = cv::Point2f(0 - 15, 50);
        icon_vertexs_dst[2] = cv::Point2f(100 + 15, 50);
        icon_vertexs_dst[3] = cv::Point2f(100 + 15, 0);

        cv::Mat icon;
        cv::Mat m = cv::getPerspectiveTransform(armor_bbox.pts, icon_vertexs_dst);
        cv::warpPerspective(img, icon, m, cv::Size(100, 50), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        cv::cvtColor(icon, icon, cv::COLOR_BGR2GRAY);
        cv::threshold(icon, icon, 0, 255, cv::THRESH_OTSU);

        // rmcv::threading::RoslikeTopic<cv::Mat>::set("debug_img_2", icon);

        std::vector<std::vector<cv::Point>> contours = find_external_contours(icon);

        cv::Point2f center(icon.cols / 2.0, icon.rows / 2.0);
        for (const auto &con : contours)
        {
            auto rect = cv::boundingRect(con);
            if (rect.contains(center))
            {
                return armor_classifier_(icon(rect), thresh);
            }
        }

        return -1;
    }

    std::vector<BoundingBox> CvArmorDetector::operator()(const cv::Mat &src)
    {
        cv::Mat img;
        double scale = 640.0 / src.cols;
        cv::resize(src, img, cv::Size(640.0, src.rows * scale));
        // double scale = 1;
        // src.copyTo(img);

        cv::Mat img_thr;// = awakenlion_threshold(img, 0.3, 0.7);
        cv::cvtColor(img, img_thr, cv::COLOR_BGR2GRAY);
        
        double gray_count[256], prefix_sum[256];
        for (int i=0;i<256;i++) gray_count[i] = 0;
        for (int i=0;i<img_thr.rows;i++)
            for (int j=0;j<img_thr.cols;j++)
                gray_count[*(img_thr.data+i*img_thr.cols+j)]+=1;
        prefix_sum[0] = gray_count[0];
        for (int i=1;i<256;i++) prefix_sum[i] = prefix_sum[i-1]+gray_count[i];
        int32_t thresh=255;
        for (int i=10;i<255;i++) if (gray_count[i]/(img_thr.rows*img_thr.cols)<0.001)
        {
            thresh = i;
            break;
        }
        thresh = std::min(255, thresh+25);
        std::cout << thresh << std::endl;
        int32_t maxp = 0;
        for (int i=0;i<256;i++) if (gray_count[i]>gray_count[maxp]) maxp = i;
        maxp = gray_count[maxp];
        for (int i=0;i<256;i++) gray_count[i] = gray_count[i]/maxp;
        // for (int i=0;i<256;i++) std::cout << stat[i] << std::endl;
        cv::Mat stat_img{cv::Size(256, 600), CV_8UC1, cv::Scalar(0)};
        cv::line(stat_img, cv::Point(thresh, 0), cv::Point(thresh, 600), cv::Scalar(125));
        for (int i=0;i<256;i++)
            cv::line(stat_img, cv::Point(i, 600), cv::Point(i, 600-gray_count[i]*600), cv::Scalar(255));
        
        rmcv::threading::RoslikeTopic<cv::Mat>::set("debug_img_2", stat_img);

        cv::threshold(img_thr, img_thr, thresh, 255, cv::THRESH_BINARY);
        cv::morphologyEx(img_thr, img_thr, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 4)));
        // cv::morphologyEx(img_thr, img_thr, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)));

        rmcv::threading::RoslikeTopic<cv::Mat>::set("debug_img_1", img_thr);

        std::vector<std::vector<cv::Point>> contours = find_external_contours(img_thr);

        std::vector<RRect> rrects;
        std::vector<LightbarMatchResult> pairs;
        for (const auto &con : contours)
        {
            // 筛选轮廓
            if (cv::contourArea(con) < 1)
                continue;

            rrects.push_back(RRect(cv::minAreaRect(con)));
        }
        match_lightbars(rrects, pairs);

        std::vector<uint8_t> vis(rrects.size(), 0);
        std::vector<BoundingBox> armors;
        for (const auto &pair : pairs)
        {
            if (pair.confidence < 0.75)
                break;
            if (vis[pair.left_idx] || vis[pair.right_idx])
                continue;

            const RRect &left = rrects[pair.left_idx];
            const RRect &right = rrects[pair.right_idx];

            auto armor_left = left; armor_left.width *= 3;
            auto armor_right = right; armor_right.width *= 3;
            int32_t robot_id = match_armor_icon(img, make_boundingbox(armor_left, armor_right), 0.75);
            if (robot_id == -1) continue;

            BoundingBox bbox = make_boundingbox(left, right);
            bbox.tag_id = robot_id;
            bbox.color_id = lightbar_color(img, left);
            for (int i=0;i<4;i++)
            {
                bbox.pts[i].x /= scale;
                bbox.pts[i].y /= scale;
            }

            // std::cout << bbox.tag_id << " <-> " << bbox.color_id << std::endl;

            vis[pair.left_idx] = vis[pair.right_idx] = 1;
            armors.push_back(std::move(bbox));
        }

        tracker_.track(src, armors);

        return armors;
    }
}