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

    void fix_boundingbox(BoundingBox &bbox, const cv::Mat &img)
    {
        /*
         * 使用OpenCV重新定位检测结果中的四个点
         */

        cv::Range range_row, range_col; // 外接矩形
        range_row.start = std::min(bbox.pts[0].y, std::min(bbox.pts[1].y, std::min(bbox.pts[2].y, bbox.pts[3].y)));
        range_row.end = std::max(bbox.pts[0].y, std::max(bbox.pts[1].y, std::max(bbox.pts[2].y, bbox.pts[3].y)));
        range_col.start = std::min(bbox.pts[0].x, std::min(bbox.pts[1].x, std::min(bbox.pts[2].x, bbox.pts[3].x)));
        range_col.end = std::max(bbox.pts[0].x, std::max(bbox.pts[1].x, std::max(bbox.pts[2].x, bbox.pts[3].x)));

        // 范围扩大
        const float ws = 0.1, hs=0.15;
        range_row.start = std::max(0, range_row.start - int((range_row.end - range_row.start) * hs / 2));
        range_row.end = std::min(img.rows, range_row.end + int((range_row.end - range_row.start) * hs / 2));
        range_col.start = std::max(0, range_col.start - int((range_col.end - range_col.start) * ws / 2));
        range_col.end = std::min(img.cols, range_col.end + int((range_col.end - range_col.start) * ws / 2));

        cv::Mat roi = img(range_row, range_col);
        std::vector<cv::Mat> chs;
        cv::split(roi, chs);
        // cv::imshow("CH-1", chs[0]); cv::imshow("CH-2", chs[1]); cv::imshow("CH-3", chs[2]);

        // cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
        if (bbox.color_id==1)
            roi = chs[2]; // 红色通道
        else if (bbox.color_id==0)
            roi = chs[0]; // 蓝色通道

        cv::threshold(roi, roi, 0, 255, cv::THRESH_OTSU); // 阈值
        // cv::threshold(roi, roi, 250, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy; // unused
        cv::findContours(roi, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 灯条匹配
        std::vector<LightBar> rrects;
        for (const auto &con : contours) rrects.push_back(detect::minAreaRect(con));
        std::sort(rrects.begin(), rrects.end(), [](auto& r1, auto& r2) { return r1.center.x < r2.center.x; }); // 按照中心x升序（从左到右）

        std::vector<std::vector<int>> pairs; // 左灯条编号，右灯条编号，置信度
        for (int i=0;i<rrects.size()-1;i++)
            for (int j=i+1;j<rrects.size();j++)
            {
                std::vector<int> pair = {i, j, 0};
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
                if (r.center.x-l.center.x > 0.7*(bbox.pts[3].x-bbox.pts[0].x)) c += 30;
                // 两灯条长度
                if (l.width<roi.rows*0.5 && r.width<roi.rows*0.5) c -= 30;
                if (l.width<roi.rows*0.3 && r.width<roi.rows*0.3) break;
                // __LOG_DEBUG("{} {} | {} {}", i, (float)(l.width/l.height), j, (float)(r.width/r.height));
                pairs.push_back(std::move(pair));
            }
        std::sort(pairs.begin(), pairs.end(), [](auto& p1, auto& p2) { return p2[2]<p1[2]; }); // 按置信度倒序

        if (pairs.size() && pairs[0][2] > 80)
        {
            // std::cout << "修正实行" << std::endl;
            for (int i = 0; i < 2; i++)
            {
                const auto &rr = rrects[pairs[0][i]];
                float dx = rr.width / 2 * cos(rr.rad), dy = rr.width / 2 * sin(rr.rad);
                bbox.pts[2 * i + i] = cv::Point2f(range_col.start + rr.center.x - dx, range_row.start + rr.center.y - dy);
                bbox.pts[2 * i + 1 - i] = cv::Point2f(range_col.start + rr.center.x + dx, range_row.start + rr.center.y + dy);
                cv::line(roi, cv::Point2f(rr.center.x - dx, rr.center.y - dy), cv::Point2f(rr.center.x + dx, rr.center.y + dy), {0, 0, 0}, 2);
            }
        }
    }
}