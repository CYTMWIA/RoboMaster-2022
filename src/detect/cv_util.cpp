#include "cv_util.hpp"

#include "threading/threading.hpp"

#include <algorithm>

namespace rmcv::detect
{
    Lightbar::Lightbar(const RRect &rrect, const double &scale)
    {
        float dx = rrect.width / 2 * cos(rrect.rad), dy = rrect.width / 2 * sin(rrect.rad);
        vertex_up = cv::Point2f(rrect.center.x - dx*scale, rrect.center.y - dy*scale);
        vertex_down = cv::Point2f(rrect.center.x + dx*scale, rrect.center.y + dy*scale);
    }

    Lightbar Lightbar::extend(double scale) const
    {
        Lightbar res;
        double cx = (vertex_up.x+vertex_down.x)/2.0;
        double cy = (vertex_up.y+vertex_down.y)/2.0;
        res.vertex_up = cv::Point2f(cx+(vertex_up.x-cx)*scale, cy+(vertex_up.y-cy)*scale);
        res.vertex_down = cv::Point2f(cx+(vertex_down.x-cx)*scale, cy+(vertex_down.y-cy)*scale);
        return res;
    }

    RRect::RRect(const cv::RotatedRect &cvrrect)
    {
        // 确定中心
        center = cvrrect.center;

        cv::Point2f pts[4];
        cvrrect.points(pts);

        // 确定长宽
        float len01 = sqrt((pts[0].x - pts[1].x) * (pts[0].x - pts[1].x) + (pts[0].y - pts[1].y) * (pts[0].y - pts[1].y));
        float len03 = sqrt((pts[0].x - pts[3].x) * (pts[0].x - pts[3].x) + (pts[0].y - pts[3].y) * (pts[0].y - pts[3].y));
        if (len01 < len03)
        {
            std::swap(pts[1], pts[3]);
            std::swap(len01, len03);
        }
        width = len01;
        height = len03;

        // 确定倾斜弧度
        if (pts[0].y > pts[1].y) std::swap(pts[0], pts[1]);
        rad = std::atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x);
    }

    inline double distance(cv::Point2f p1, cv::Point2f p2)
    {
        return abs(cv::norm(p1-p2));
    }

    cv::Mat awakenlion_threshold(const cv::Mat &src, double gray_max_, double gray_avg_, double ch0, double ch1, double ch2)
    {
        // AWAKENLION 二值化
        // https://github.com/Ash1104/RoboMaster2021-FOSU-AWAKENLION-OpenSource
        std::vector<cv::Mat> chs;
        cv::split(src, chs);
        cv::Mat img = chs[0] * ch0 + chs[1] * ch1 + chs[2] * ch2;

        //最大灰度
        uchar gray_max = img.data[0];
        for (int i = 0; i < img.rows; i++)
        {
            uchar *p = img.data + i * img.cols;
            for (int j = 0; j < img.cols; j++)
            {
                if (*(p + j) > gray_max)
                    gray_max = *(p + j);
            }
        }
        // 平均灰度
        long sum_gray = 0;
        for (int j = 0; j < img.rows; j++)
        {
            uchar *data = img.ptr<uchar>(j);
            for (int i = 0; i < img.cols; i++)
            {
                sum_gray += data[i];
            }
        }
        uint8_t gray_avg = sum_gray * 1.0 / (img.cols * img.rows);

        int thresh = gray_max * gray_max_ + gray_avg * gray_avg_; // 醒师代码中两者的比例为0.6、0.4

        // 二值化
        cv::Mat res;
        cv::threshold(img, res, thresh, 255, cv::THRESH_BINARY);
        return res;
    }

    void match_lightbars(std::vector<RRect> &rrects, std::vector<Lightbar> &lightbars, std::vector<LightbarMatchResult> &results)
    {
        if (rrects.empty()) return;

        std::sort(rrects.begin(), rrects.end(), [](auto& r1, auto& r2) { return r1.center.x < r2.center.x; }); // 按照中心x升序（从左到右）
        lightbars.clear();
        lightbars.reserve(rrects.size());
        for (const auto& rr: rrects) lightbars.push_back(Lightbar(rr));

        for (int i = 0; i < rrects.size() - 1; i++)
            for (int j = i + 1; j < rrects.size(); j++)
            {
                using namespace std;

                LightbarMatchResult res = {i, j, 0};
                const auto &lrr = rrects[i];
                const auto &rrr = rrects[j];
                const auto &llb = lightbars[i];
                const auto &rlb = lightbars[j];
                auto &c = res.confidence;
                double total_score=0;

#define ADD_CONFIDENCE(weight, x) total_score+=weight; c += (double)(weight) * (x);

                // 平行
                float angle = abs(lrr.rad - rrr.rad);
                ADD_CONFIDENCE(40.0, 1 - abs(min(angle, 180 - angle) / (M_PI / 4.0)));

                // 长宽相同
                ADD_CONFIDENCE(30.0, 1 - abs(2 * (lrr.width - rrr.width) / (lrr.width + rrr.width)));
                ADD_CONFIDENCE(20.0, 1 - abs(2 * (lrr.height - rrr.height) / (lrr.height + rrr.height)));

                // 组成四边形的长宽比
                float width_up = distance(llb.vertex_up , rlb.vertex_up);
                float width_down = distance(llb.vertex_down, rlb.vertex_down);
                float ratio_tl = width_up / lrr.width;
                float ratio_br = width_down / rrr.width;
                ADD_CONFIDENCE(25.0, 1 - min(abs(270.0 / 55.0 - ratio_tl), abs(135.0 / 55.0 - ratio_tl))/2.0);
                ADD_CONFIDENCE(25.0, 1 - min(abs(270.0 / 55.0 - ratio_br), abs(135.0 / 55.0 - ratio_br))/2.0);

                // 灯条占装甲板宽度
                ADD_CONFIDENCE(15.0, 1-lrr.height/width_up);
                ADD_CONFIDENCE(15.0, 1-lrr.height/width_down);
                ADD_CONFIDENCE(15.0, 1-rrr.height/width_up);
                ADD_CONFIDENCE(15.0, 1-rrr.height/width_down);

                c = max(0.0, c)/total_score;
                std::cout << c << std::endl;

#undef ADD_CONFIDENCE
                results.push_back(std::move(res));
            }
        if (results.empty()) return;

        std::sort(results.begin(), results.end(), [](auto &p1, auto &p2){ return p2.confidence < p1.confidence; }); // 按置信度倒序
    }

    void fix_boundingbox(BoundingBox &bbox, const cv::Mat &img)
    {
        using namespace rmcv::threading;
        /*
         * 使用OpenCV重新定位检测结果中的四个点
         */

        cv::Range range_row, range_col; // 外接矩形
        range_row.start = std::min(bbox.pts[0].y, std::min(bbox.pts[1].y, std::min(bbox.pts[2].y, bbox.pts[3].y)));
        range_row.end = std::max(bbox.pts[0].y, std::max(bbox.pts[1].y, std::max(bbox.pts[2].y, bbox.pts[3].y)));
        range_col.start = std::min(bbox.pts[0].x, std::min(bbox.pts[1].x, std::min(bbox.pts[2].x, bbox.pts[3].x)));
        range_col.end = std::max(bbox.pts[0].x, std::max(bbox.pts[1].x, std::max(bbox.pts[2].x, bbox.pts[3].x)));

        // 范围扩大
        const float ws = 0.25, hs=0.25;
        range_row.start = std::max(0, range_row.start - int((range_row.end - range_row.start) * hs / 2));
        range_row.end = std::min(img.rows, range_row.end + int((range_row.end - range_row.start) * hs / 2));
        range_col.start = std::max(0, range_col.start - int((range_col.end - range_col.start) * ws / 2));
        range_col.end = std::min(img.cols, range_col.end + int((range_col.end - range_col.start) * ws / 2));

        cv::Mat roi = img(range_row, range_col);
        
        // 统一宽度
        double scale = 100.0/roi.cols;
        cv::resize(roi, roi, cv::Size(100, roi.rows*scale));

        roi = awakenlion_threshold(roi);

        // 形态学
        cv::morphologyEx(roi, roi, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 3)));

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy; // unused
        cv::findContours(roi, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 灯条匹配
        std::vector<RRect> rrects;
        std::vector<Lightbar> lightbars;
        std::vector<LightbarMatchResult> pairs;
        for (const auto &con : contours)
        {
            // 筛选轮廓
            if (cv::contourArea(con)<16) continue;

            rrects.push_back(RRect(cv::minAreaRect(con)));
            // cv::Point2f pts4[4];
            // cv::minAreaRect(con).points(pts4);
            // for (int i = 0; i < 4; i++)
            //     cv::line(roi, pts4[i], pts4[(i+1)%4], cv::Scalar(127), 2);
        }
        if (!rrects.size()) return;
        match_lightbars(rrects, lightbars, pairs);

        cv::Mat debug; roi.copyTo(debug); RoslikeTopic<cv::Mat>::set("fix_boundingbox", debug);

        if (pairs.size() && pairs[0].confidence > 0.7)
        {
            // std::cout << "修正实行" << std::endl;
            const Lightbar &left = lightbars[pairs[0].left_idx];
            const Lightbar &right = lightbars[pairs[0].right_idx];
            bbox.pts[0] = cv::Point2f(range_col.start + left.vertex_up.x/scale, range_row.start + left.vertex_up.y/scale);
            bbox.pts[1] = cv::Point2f(range_col.start + left.vertex_down.x/scale, range_row.start + left.vertex_down.y/scale);
            bbox.pts[2] = cv::Point2f(range_col.start + right.vertex_down.x/scale, range_row.start + right.vertex_down.y/scale);
            bbox.pts[3] = cv::Point2f(range_col.start + right.vertex_up.x/scale, range_row.start + right.vertex_up.y/scale);
        }
    }
}