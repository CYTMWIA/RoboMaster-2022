#include "PnpSolver.hpp"

namespace rmcv::predict
{
    double CameraPose::distance()
    {
        return sqrt(x*x+y*y+z*z);
    }

    PnpSolver::PnpSolver(cv::Mat camera_matrix, cv::Mat distortion_coefficients):
        camera_matrix_(camera_matrix),
        distortion_coefficients_(distortion_coefficients)
    { }

    PnpSolver::PnpSolver(std::string camera_calibration_file)
    {
        cv::FileStorage fs{camera_calibration_file, cv::FileStorage::READ};
        fs["camera_matrix"] >> camera_matrix_;
        fs["distortion_coefficients"] >> distortion_coefficients_;
    }

    CameraPose PnpSolver::solve(ArmorType armor_type, const cv::Point2f pts[])
    {
        cv::Mat rvec = cv::Mat::zeros(3,1,CV_64FC1);
        cv::Mat tvec = cv::Mat::zeros(3,1,CV_64FC1);
        std::vector<cv::Point2f> points{pts, pts+4};
        
        cv::solvePnP(kArmorPoints[armor_type], points, camera_matrix_, distortion_coefficients_, rvec, tvec, false, cv::SOLVEPNP_P3P);

        CameraPose cp;
        // 将相机作为零点
        cp.x = -tvec.at<double>(0, 0);
        cp.y = -tvec.at<double>(1, 0);
        cp.z = -tvec.at<double>(2, 0);
        
        /*******************提取旋转矩阵*********************/
	    /*
	        r11 r12 r13
	        r21 r22 r23
	        r31 r32 r33
	    */
        double rm[9];
	    cv::Mat rmat = cv::Mat(3, 3, CV_64FC1, rm);
	    Rodrigues(rvec, rmat);
	    double r11 = rmat.ptr<double>(0)[0];
	    double r12 = rmat.ptr<double>(0)[1];
	    double r13 = rmat.ptr<double>(0)[2];
	    double r21 = rmat.ptr<double>(1)[0];
	    double r22 = rmat.ptr<double>(1)[1];
	    double r23 = rmat.ptr<double>(1)[2];
	    double r31 = rmat.ptr<double>(2)[0];
	    double r32 = rmat.ptr<double>(2)[1];
	    double r33 = rmat.ptr<double>(2)[2];

        //将旋转矩阵按zxy顺规解出欧拉角
	    //万向节死锁将出现在pitch轴正负90°(一般不会发生)
	    cp.theta_z = atan2(-r12, r22) / CV_PI * 180;
	    cp.theta_x = atan2(r32, sqrt(r31 * r31 + r33 * r33)) / CV_PI * 180;
	    cp.theta_y = atan2(-r31, r33) / CV_PI * 180;

        return cp;
    }
}