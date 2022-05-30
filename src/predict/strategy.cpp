#include <cstring>
#include <vector>
#include <cmath>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include "logging.hpp"
#include "detect.hpp"

#include "cv_armor.hpp"

#include "strategy.hpp"

namespace rmcv
{
    void start_strategy(const Config::Strategy &cfg)
    {
        float small_armor_width = 135, small_armor_height = 55; // mm
        std::vector<cv::Point3f> small_armor_points = {
            cv::Point3f(-small_armor_width/2, small_armor_height/2, 0),
            cv::Point3f(-small_armor_width/2, -small_armor_height/2, 0),
            cv::Point3f(small_armor_width/2, -small_armor_height/2, 0),
            cv::Point3f(small_armor_width/2, small_armor_height/2, 0)
        };

        // 读取相机矩阵
        cv::FileStorage fs{cfg.camera_calibration_file, cv::FileStorage::READ};
        cv::Mat camera_matrix, dist_coeffs;
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> dist_coeffs;

        while (true)
        {
            auto detect_result = VariableCenter<std::vector<BoundingBox>>::get("detect_result");
            cv::Mat detect_image = VariableCenter<cv::Mat>::get("detect_image");

            if (detect_result.empty()) continue;
            std::vector<cv::Point2f> points{detect_result[0].pts, &(detect_result[0].pts[4])};

            cv::Mat rvecs = cv::Mat::zeros(3,1,CV_64FC1);
            cv::Mat tvecs = cv::Mat::zeros(3,1,CV_64FC1);
            cv::solvePnP(small_armor_points, points, camera_matrix, dist_coeffs, rvecs, tvecs, false, cv::SOLVEPNP_P3P);
            cv::Mat rotM = cv::Mat::eye(3,3,CV_64F);
            cv::Mat rotT = cv::Mat::eye(3,3,CV_64F);
            cv::Rodrigues(rvecs, rotM);  //将旋转向量变换成旋转矩阵
            cv::Rodrigues(tvecs, rotT);

            //计算相机旋转角
            double theta_x, theta_y,theta_z;
            double PI = 3.14;
            theta_x = atan2(rotM.at<double>(2, 1), rotM.at<double>(2, 2));
            theta_y = atan2(-rotM.at<double>(2, 0),
            sqrt(rotM.at<double>(2, 1)*rotM.at<double>(2, 1) + rotM.at<double>(2, 2)*rotM.at<double>(2, 2)));
            theta_z = atan2(rotM.at<double>(1, 0), rotM.at<double>(0, 0));
            theta_x = theta_x * (180 / PI);
            theta_y = theta_y * (180 / PI);
            theta_z = theta_z * (180 / PI);

            cv::Mat P;
            P = (rotM.t()) * tvecs;

            __LOG_DEBUG("Angle: {:.2f}, {:.2f}, {:.2f}", theta_x, theta_y, theta_z);
            std::cout << "...:" << P << std::endl;
            std::cout << "R矩阵:" << rvecs << std::endl;
            std::cout << "T矩阵:" << tvecs << std::endl;
            __LOG_DEBUG("{}, {}, {}", tvecs.at<double>(0), tvecs.at<double>(2), tvecs.at<double>(2));
            std::cout << "距离:" << sqrt(tvecs.at<double>(0)*tvecs.at<double>(0) + tvecs.at<double>(1)*tvecs.at<double>(1) + tvecs.at<double>(2)*tvecs.at<double>(2)) << std::endl;
            // BoundingBox target_bbox = detect_result[0];
            // target_bbox = fix_boundingbox(target_bbox, detect_image);

        }

        if (cfg.robot == "")
        {
            __LOG_ERROR_AND_EXIT("未指定机器人");
        }

        

        double lw = 135, lh = 55;
        // 小装甲板
        std::vector<cv::Point3f> s_armor = {
            // 横轴，竖轴，0
            cv::Point3f(-lw / 2, +lh / 2, 0), // 左灯条上
            cv::Point3f(-lw / 2, -lh / 2, 0), // 左灯条下
            cv::Point3f(+lw / 2, -lh / 2, 0), // 右灯条下
            cv::Point3f(+lw / 2, +lh / 2, 0), // 右灯条上
        };

        std::vector<uint8_t> msg2ec;
        msg2ec.resize(12);
        msg2ec[0] = msg2ec[1] = 0xAA;
        msg2ec[2] = 0;
        msg2ec[11] = 0xBB;
        int empty_count = 0;
        while (true)
        {
            auto detected = VariableCenter<std::vector<BoundingBox>>::get("detect_result");
            if (detected.size() <= 0)
            {
                if (empty_count >= 3)
                {
                    for (int i = 3; i < 11; i++)
                        msg2ec[i] = 0;
                    // send2ec.set(msg2ec);
                }
                empty_count++;
                continue;
            }
            empty_count = 0;

            // std::vector<cv::Point2f> points = {
            //     detected[0].pts[0],
            //     detected[0].pts[1],
            //     detected[0].pts[2],
            //     detected[0].pts[3],
            // };

            // rvec = cv::Mat::zeros(3,1,CV_64FC1);
            // tvec = cv::Mat::zeros(3,1,CV_64FC1);

            // cv::solvePnP(s_armor, points, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_P3P);
            // cv::Mat rotM = cv::Mat::eye(3,3,CV_64F);
            // cv::Mat rotT = cv::Mat::eye(3,3,CV_64F);
            // cv::Rodrigues(rvec, rotM); //将旋转向量变换成旋转矩阵
            // cv::Rodrigues(tvec, rotT);

            // //计算相机旋转角
            // double theta_x, theta_y,theta_z;
            // double PI = 3.14;
            // theta_x = atan2(rotM.at<double>(2, 1), rotM.at<double>(2, 2));
            // theta_y = atan2(-rotM.at<double>(2, 0),
            // sqrt(rotM.at<double>(2, 1)*rotM.at<double>(2, 1) + rotM.at<double>(2, 2)*rotM.at<double>(2, 2)));
            // theta_z = atan2(rotM.at<double>(1, 0), rotM.at<double>(0, 0));
            // theta_x = theta_x * (180 / PI);
            // theta_y = theta_y * (180 / PI);
            // theta_z = theta_z * (180 / PI);

            // //计算深度
            // cv::Mat P;
            // P = (rotM.t()) * tvec;

            // std::cout<< "角度"   <<std::endl;
            // std::cout<< theta_x <<std::endl;
            // std::cout<< theta_y <<std::endl;
            // std::cout<< theta_z <<std::endl;
            // std::cout<< P       <<std::endl;

            // int icx=1280/2, icy=1024/2; // Daheng
            int icx = 640 / 2, icy = 512 / 2; // 步兵

            int ci = 0;
            float dis = 9999, acx, acy;
            for (int i = 0; i < detected.size(); i++)
            {
                float cx = (detected[i].pts[0].x + detected[i].pts[1].x + detected[i].pts[2].x + detected[i].pts[3].x) / 4.0;
                float cy = (detected[i].pts[0].y + detected[i].pts[1].y + detected[i].pts[2].y + detected[i].pts[3].y) / 4.0;
                float d = sqrt((cx - icx) * (cx - icx) + (cy - icy) * (cy - icy));
                if (d < dis)
                {
                    ci = i;
                    dis = d;
                    acx = cx;
                    acy = cy;
                }
            }

            auto armor = detected[ci];
            float armor_width_px = sqrt(
                (armor.pts[2].x - armor.pts[1].x) * (armor.pts[2].x - armor.pts[1].x) + (armor.pts[2].y - armor.pts[1].y) * (armor.pts[2].y - armor.pts[1].y));

            float f = 410.0 * (400.0 / 135.0); // (像素长度/(测量距离/小装甲板宽度))
            // float depth = 135.0/(atan(armor_width_px/f));
            float depth = 135.0 * (f / armor_width_px);

            float mm_per_px = 135.0 / armor_width_px;
            float yaw = atan(((icx - acx) * mm_per_px) / depth) * 57.29577951308;
            float pitch = -atan(((icy - acy) * mm_per_px) / depth) * 57.29577951308;

            // __LOG_DEBUG("深度 {}mm | YAW {} | PITCH {}", depth, yaw, pitch);

            // uint32_t t = 0x01020304;
            // memcpy(&msg2ec[3], &t, 4);
            // memcpy(&msg2ec[7], &t, 4);

            // 左下为正方向
            memcpy(&msg2ec[3], &pitch, 4);
            // std::swap(msg2ec[3], msg2ec[6]); std::swap(msg2ec[4], msg2ec[5]);
            memcpy(&msg2ec[7], &yaw, 4);
            // std::swap(msg2ec[7], msg2ec[10]); std::swap(msg2ec[8], msg2ec[9]);

            // send2ec.set(msg2ec);
        }
    }
}