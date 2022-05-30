#include "cv_util.hpp"

#include "threading/threading.hpp"

#include "logging/logging.hpp"

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
                ADD_CONFIDENCE(40.0, 1 - abs(min(angle, 180 - angle) / (M_PI / 2.0)));
                // std::cout << c << " ";

                // 长宽相同
                ADD_CONFIDENCE(40.0, 1 - abs(2 * (lrr.width - rrr.width+1) / (lrr.width + rrr.width+1))); // +1 避免 nan
                ADD_CONFIDENCE(10.0, 1 - abs(2 * (lrr.height - rrr.height+1) / (lrr.height + rrr.height+1)));
                // std::cout << c << " ";

                // 组成四边形的长宽比
                std::vector<cv::Point2f> pts = { llb.vertex_up , rlb.vertex_up, llb.vertex_down, rlb.vertex_down };
                auto arect = RRect(cv::minAreaRect(pts));
                float ratio = arect.width/arect.height;
                ADD_CONFIDENCE(50.0, 1 - min(abs(270.0 / 55.0 - ratio), abs(135.0 / 55.0 - ratio))/1.5);
                // std::cout << c << " ";

                // 灯条占装甲板宽度
                float width_up = distance(llb.vertex_up , rlb.vertex_up);
                float width_down = distance(llb.vertex_down, rlb.vertex_down);
                ADD_CONFIDENCE(10.0, 1-lrr.height/width_up);
                ADD_CONFIDENCE(10.0, 1-lrr.height/width_down);
                ADD_CONFIDENCE(10.0, 1-rrr.height/width_up);
                ADD_CONFIDENCE(10.0, 1-rrr.height/width_down);
                // std::cout << c << " ";

                c = max(0.0, c)/total_score;
                // std::cout << ">> " << c << std::endl;

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

        // 自动二值化，用尽可能小的阈值达成区域内只有两个轮廓（使灯条区域最大）
        cv::Mat gray, thr;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        std::vector<std::vector<cv::Point>> contours, cons;
        int32_t lo=100, hi=255, mid;
        while (lo + 16 < hi)
        {
            contours.clear();

            mid = (lo+hi)/2;
            cv::threshold(gray, thr, mid, 255, cv::THRESH_BINARY);
            cons = find_external_contours(thr);
            std::copy_if(cons.begin(), cons.end(), std::back_inserter(contours), [](auto &c){ return cv::contourArea(c) > 4;});

            if (2 < contours.size()) 
            {
                lo = mid + 1;
            }
            else
            {
                hi = mid;
            }
        }

        // 灯条匹配
        std::vector<RRect> rrects;
        std::vector<Lightbar> lightbars;
        std::vector<LightbarMatchResult> pairs;
        if (contours.size() == 2)
        {
            for (const auto &con : contours)
            {
                rrects.push_back(RRect(cv::minAreaRect(con)));
            }
            // RoslikeTopic<cv::Mat>::set("debug_img_1", thr);
            // RoslikeTopic<cv::Mat>::set("debug_img_2", awakenlion_threshold(roi));
        }
        else
        {
            roi = awakenlion_threshold(roi);
            cv::morphologyEx(roi, roi, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 4)));
            contours = find_external_contours(roi);

            for (const auto &con : contours)
            {
                // 筛选轮廓
                if (cv::contourArea(con)<16) continue;

                rrects.push_back(RRect(cv::minAreaRect(con)));
            }
            if (rrects.size() < 2) return;
        }
        match_lightbars(rrects, lightbars, pairs);

        

        if (pairs.size() && (contours.size() == 2 || pairs[0].confidence > 0.6))
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

    std::vector<std::vector<cv::Point>> find_external_contours(const cv::Mat &src)
    {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy; // unused
        cv::findContours(src, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        return contours;
    }

    cv::Mat safe_range_roi(const cv::Mat &src, cv::Range range_rows, cv::Range range_cols)
    {
        range_rows.start = std::max(0, range_rows.start);
        range_rows.end = std::min(src.rows, range_rows.end);
        range_cols.start = std::max(0, range_cols.start);
        range_cols.end = std::min(src.cols, range_cols.end);
        return src(range_rows, range_cols);
    }
}