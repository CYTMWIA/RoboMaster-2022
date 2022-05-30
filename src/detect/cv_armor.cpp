#include "cv_armor.hpp"

namespace rmcv::detect
{
    LightBar minAreaRect(cv::InputArray points)
    {
        /*
         * 返回值：width始终为长边，angle始终为长边从 向正右方射线 沿逆时针转动的角度即[0, 180)
         */
        cv::RotatedRect rrect = cv::minAreaRect(points);

        cv::Point2f pts[4];
        rrect.points(pts);

        LightBar res;
        res.center = rrect.center;

        float len01 = sqrt((pts[0].x - pts[1].x) * (pts[0].x - pts[1].x) + (pts[0].y - pts[1].y) * (pts[0].y - pts[1].y));
        float len03 = sqrt((pts[0].x - pts[3].x) * (pts[0].x - pts[3].x) + (pts[0].y - pts[3].y) * (pts[0].y - pts[3].y));
        if (len01 < len03)
        {
            std::swap(pts[1], pts[3]);
            std::swap(len01, len03);
        }
        res.width = len01;
        res.height = len03;
        if (pts[0].y > pts[1].y) std::swap(pts[0], pts[1]);
        res.rad = std::atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x);

        return res;
    }

    BoundingBox fix_boundingbox(const BoundingBox &ori_bbox, const cv::Mat &ori_img)
    {
        /*
         * 使用OpenCV重新定位检测结果中的四个点
         */

        cv::Range range_row, range_col; // 外接矩形
        range_row.start = std::min(ori_bbox.pts[0].y, std::min(ori_bbox.pts[1].y, std::min(ori_bbox.pts[2].y, ori_bbox.pts[3].y)));
        range_row.end = std::max(ori_bbox.pts[0].y, std::max(ori_bbox.pts[1].y, std::max(ori_bbox.pts[2].y, ori_bbox.pts[3].y)));
        range_col.start = std::min(ori_bbox.pts[0].x, std::min(ori_bbox.pts[1].x, std::min(ori_bbox.pts[2].x, ori_bbox.pts[3].x)));
        range_col.end = std::max(ori_bbox.pts[0].x, std::max(ori_bbox.pts[1].x, std::max(ori_bbox.pts[2].x, ori_bbox.pts[3].x)));

        // 边长扩大1.1倍
        range_row.start = std::max(0, range_row.start - int((range_row.end - range_row.start) * 0.1 / 2));
        range_row.end = std::min(ori_img.rows, range_row.end + int((range_row.end - range_row.start) * 0.1 / 2));
        range_col.start = std::max(0, range_col.start - int((range_col.end - range_col.start) * 0.1 / 2));
        range_col.end = std::min(ori_img.cols, range_col.end + int((range_col.end - range_col.start) * 0.1 / 2));

        cv::Mat roi = ori_img(range_row, range_col);

        cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
        
        cv::threshold(roi, roi, 0, 255, cv::THRESH_OTSU); // 阈值

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy; // unused
        cv::findContours(roi, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 灯条匹配
        std::vector<LightBar> rrects;
        for (const auto &con : contours) rrects.push_back(detect::minAreaRect(con));
        std::sort(rrects.begin(), rrects.end(), // 按照中心x升序（从左到右）
                  [](auto& r1, auto& r2) { return r1.center.x < r2.center.x; });
        // std::cout << rrects.size() << std::endl;
        std::vector<std::vector<int>> pairs; // 左灯条编号，右灯条编号，置信度
        for (int i=0;i<rrects.size()-1;i++)
            for (int j=i+1;j<rrects.size();j++)
            {
                std::vector<int> pair {i, j, 0};

                const auto& l = rrects[i];
                const auto& r = rrects[j];
                auto& c = pair[2];

                // 平行
                if (std::abs(l.rad-r.rad)<15*(360.0/(2*M_PI))) c += 30;
                // 大小、长宽相同
                auto avg_width=(l.width+r.width)/2.0, avg_height=(l.height+r.height)/2.0;
                if (   std::abs(l.width  - r.width)  < 0.3*avg_width
                    && std::abs(l.height - r.height) < 0.3*avg_height) c += 50;
                // 长宽比符合
                if (3 < l.width/l.height && l.width/l.height < 10) c += 25;
                if (3 < r.width/r.height && r.width/r.height < 10) c += 25;
                // 两灯条的距离够远
                if (r.center.x-l.center.x > 0.7*(ori_bbox.pts[3].x-ori_bbox.pts[0].x)) c += 30;
                // __LOG_DEBUG("{} {} | {} {}", i, (float)(l.width/l.height), j, (float)(r.width/r.height));
                pairs.push_back(std::move(pair));
            }
        std::sort(pairs.begin(), pairs.end(),
                  [](auto& p1, auto& p2) { return p2[2]<p1[2]; }); // 按置信度倒序

        BoundingBox res = ori_bbox;
        if (pairs.size() && pairs[0][2]>80)
            for (int i=0;i<2;i++)
            {
                const auto& rr = rrects[pairs[0][i]];
                float dx = rr.width / 2 * cos(rr.rad), dy = rr.width / 2 * sin(rr.rad);
                res.pts[2*i   + i] = cv::Point2f(range_col.start + rr.center.x - dx, range_row.start + rr.center.y - dy);
                res.pts[2*i+1 - i] = cv::Point2f(range_col.start + rr.center.x + dx, range_row.start + rr.center.y + dy);
            }

        // cv::line(ori_img, res.pts[0], res.pts[1], cv::Scalar{0, 0, 255}, 2);
        // cv::line(ori_img, res.pts[1], res.pts[2], cv::Scalar{0, 0, 255}, 2);
        // cv::line(ori_img, res.pts[2], res.pts[3], cv::Scalar{0, 0, 255}, 2);
        // cv::line(ori_img, res.pts[3], res.pts[0], cv::Scalar{0, 0, 255}, 2);
        // VariableCenter<cv::Mat>::set("debug_fb", roi);
        // // STRATEGY_DEBUG_IMAGE.set(ori_img);

        return res;
    }
}