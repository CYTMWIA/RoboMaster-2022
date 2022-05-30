#include "cv_armor.hpp"

#include "threading/threading.hpp"

namespace rmcv::detect
{
    RRect min_area_rect(cv::InputArray points)
    {
        /*
         * 返回值：width始终为长边，angle始终为长边从 向正右方射线 沿逆时针转动的角度即[0, 180)
         */
        cv::RotatedRect rrect = cv::minAreaRect(points);

        cv::Point2f pts[4];
        rrect.points(pts);

        RRect res;
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

    void threshold(cv::Mat &img)
    {
        // 二值化
        // https://github.com/Ash1104/RoboMaster2021-FOSU-AWAKENLION-OpenSource
        std::vector<cv::Mat> chs;
        cv::split(img, chs);
        img = chs[0]*0.1+chs[1]*0.2+chs[2]*0.7;

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
        for(int j = 0; j < img.rows; j++)
        {
            uchar *data = img.ptr<uchar>(j);
            for(int i = 0; i < img.cols; i++)
            {
                sum_gray += data[i];
            }
        }
        uint8_t gray_avg = sum_gray * 1.0 / (img.cols * img.rows);

        int thresh = gray_max*0.5 + gray_avg*0.5;

        // 二值化
        cv::threshold(img, img, thresh, 255, cv::THRESH_BINARY);
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

        threshold(roi);

        // 形态学
        cv::morphologyEx(roi, roi, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 3)));

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy; // unused
        cv::findContours(roi, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 灯条匹配
        std::vector<RRect> rrects;
        for (const auto &con : contours)
        {
            // 筛选轮廓
            if (cv::contourArea(con)<16) continue;

            rrects.push_back(detect::min_area_rect(con));
            // cv::Point2f pts4[4];
            // cv::minAreaRect(con).points(pts4);
            // for (int i = 0; i < 4; i++)
            //     cv::line(roi, pts4[i], pts4[(i+1)%4], cv::Scalar(127), 2);
        }
        if (!rrects.size()) return;
        std::sort(rrects.begin(), rrects.end(), [](auto& r1, auto& r2) { return r1.center.x < r2.center.x; }); // 按照中心x升序（从左到右）

        cv::Mat debug; roi.copyTo(debug); RoslikeTopic<cv::Mat>::set("fix_boundingbox", debug);

        std::vector<std::vector<int>> pairs; // 左灯条编号，右灯条编号，置信度
        for (int i=0;i<rrects.size()-1;i++)
            for (int j=i+1;j<rrects.size();j++)
            {
                std::vector<int> pair = {i, j, 0};
                const auto& l = rrects[i];
                const auto& r = rrects[j];
                auto& c = pair[2];

                // 平行
                c += (1-abs((l.rad-r.rad)/(M_PI/2.0)))*40;
                
                // 长宽相同
                c += (1-abs((l.width-r.width)/(l.width+r.width)))*25;
                c += (1-abs((l.height-r.height)/(l.height+r.height)))*25;
                std::cout << c << std::endl;

                // __LOG_DEBUG("{} {} | {} {}", i, (float)(l.width/l.height), j, (float)(r.width/r.height));
                pairs.push_back(std::move(pair));
            }
        std::sort(pairs.begin(), pairs.end(), [](auto& p1, auto& p2) { return p2[2]<p1[2]; }); // 按置信度倒序

        if (pairs.size() && pairs[0][2] > 70)
        {
            // std::cout << "修正实行" << std::endl;
            for (int i = 0; i < 2; i++)
            {
                const auto &rr = rrects[pairs[0][i]];
                float dx = rr.width / 2 * cos(rr.rad), dy = rr.width / 2 * sin(rr.rad);
                bbox.pts[2 * i + i] = cv::Point2f(range_col.start + (rr.center.x - dx)/scale, range_row.start + (rr.center.y - dy)/scale);
                bbox.pts[2 * i + 1 - i] = cv::Point2f(range_col.start + (rr.center.x + dx)/scale, range_row.start + (rr.center.y + dy)/scale);
                // cv::line(roi, cv::Point2f(rr.center.x - dx, rr.center.y - dy), cv::Point2f(rr.center.x + dx, rr.center.y + dy), {0, 0, 0}, 2); // Debug
            }
        }
    }
}