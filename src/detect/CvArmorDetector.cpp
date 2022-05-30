#include "CvArmorDetector.hpp"

#include "cv_util.hpp"
#include "HammingMatch.hpp"

#include "threading/threading.hpp"

namespace rmcv::detect
{
    CvArmorDetector::CvArmorDetector()
    {
        armor_classifier_.add_match(1, make_icon_hammimg_match("icon/1.png"));
        armor_classifier_.add_match(2, make_icon_hammimg_match("icon/2.png"));
        armor_classifier_.add_match(3, make_icon_hammimg_match("icon/3.png"));
        armor_classifier_.add_match(4, make_icon_hammimg_match("icon/4.png"));
        armor_classifier_.add_match(5, make_icon_hammimg_match("icon/5.png"));
    }

    int32_t lightbar_color(const cv::Mat &img, const Lightbar &lb)
    {
        cv::Range range_rows, range_cols;
        range_rows = cv::Range(lb.vertex_up.y, lb.vertex_down.y+1);
        if (lb.vertex_up.x < lb.vertex_down.x)
            range_cols = cv::Range(lb.vertex_up.x, lb.vertex_down.x+1);
        else
            range_cols = cv::Range(lb.vertex_down.x, lb.vertex_up.x+1);
        cv::Mat roi = safe_range_roi(img, range_rows, range_cols);

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

    int32_t CvArmorDetector::match_armor_icon(const cv::Mat &img, const Lightbar &left, const Lightbar &right, const double &thresh)
    {
        cv::Point2f icon_vertexs_src[4];
        auto armor_left = left.extend(3);
        auto armor_right = right.extend(3);
        icon_vertexs_src[0] = cv::Point2f(armor_left.vertex_up.x, armor_left.vertex_up.y);
        icon_vertexs_src[1] = cv::Point2f(armor_left.vertex_down.x, armor_left.vertex_down.y);
        icon_vertexs_src[2] = cv::Point2f(armor_right.vertex_down.x, armor_right.vertex_down.y);
        icon_vertexs_src[3] = cv::Point2f(armor_right.vertex_up.x, armor_right.vertex_up.y);
        cv::Point2f icon_vertexs_dst[4];
        icon_vertexs_dst[0] = cv::Point2f(0 - 10, 0 - 10);
        icon_vertexs_dst[1] = cv::Point2f(0 - 10, 100 + 10);
        icon_vertexs_dst[2] = cv::Point2f(200 + 10, 100 + 10);
        icon_vertexs_dst[3] = cv::Point2f(200 + 10, 0 - 10);

        cv::Mat icon;
        cv::Mat m = cv::getPerspectiveTransform(icon_vertexs_src, icon_vertexs_dst);
        cv::warpPerspective(img, icon, m, cv::Size(200, 100), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        cv::cvtColor(icon, icon, cv::COLOR_BGR2GRAY);
        cv::threshold(icon, icon, 0, 255, cv::THRESH_OTSU);

        rmcv::threading::RoslikeTopic<cv::Mat>::set("debug_img_2", icon);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy; // unused
        cv::findContours(icon, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

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
        // double scale = 640.0 / src.cols;
        // cv::resize(src, img, cv::Size(640.0, src.rows * scale));
        double scale = 1;
        src.copyTo(img);

        cv::Mat img_thr = awakenlion_threshold(img);
        cv::morphologyEx(img_thr, img_thr, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(12, 6)));

        rmcv::threading::RoslikeTopic<cv::Mat>::set("debug_img_1", img_thr);

        std::vector<std::vector<cv::Point>> contours = find_external_contours(img_thr);

        std::vector<RRect> rrects;
        std::vector<Lightbar> lightbars;
        std::vector<LightbarMatchResult> pairs;
        for (const auto &con : contours)
        {
            // 筛选轮廓
            // if (cv::contourArea(con) < 16)
            //     continue;

            rrects.push_back(RRect(cv::minAreaRect(con)));
        }
        match_lightbars(rrects, lightbars, pairs);

        std::vector<uint8_t> vis(lightbars.size(), 0);
        std::vector<BoundingBox> armors;
        for (const auto &pair : pairs)
        {
            if (pair.confidence < 0.70)
                break;
            if (vis[pair.left_idx] || vis[pair.right_idx])
                continue;

            const Lightbar &left = lightbars[pair.left_idx];
            const Lightbar &right = lightbars[pair.right_idx];

            int32_t robot_id = match_armor_icon(img, left, right, 0.70);
            if (robot_id == -1)
                continue;

            vis[pair.left_idx] = vis[pair.right_idx] = 1;

            BoundingBox bbox;
            bbox.tag_id = robot_id;
            bbox.color_id = lightbar_color(img, left);
            bbox.pts[0] = cv::Point2f(left.vertex_up.x / scale, left.vertex_up.y / scale);
            bbox.pts[1] = cv::Point2f(left.vertex_down.x / scale, left.vertex_down.y / scale);
            bbox.pts[2] = cv::Point2f(right.vertex_down.x / scale, right.vertex_down.y / scale);
            bbox.pts[3] = cv::Point2f(right.vertex_up.x / scale, right.vertex_up.y / scale);

            std::cout << bbox.tag_id << " <-> " << bbox.color_id << std::endl;

            armors.push_back(std::move(bbox));
        }

        return armors;
    }
}