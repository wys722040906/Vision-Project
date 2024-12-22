#include "detect/pnpsolver.h"
#include <detect/paramLoad.h>

namespace detectAndTrack{

void PnPsolver::showParams(cv::Mat& img){
    std::stringstream ss;
    
    cv::Mat cameraMatrix_show = cameraMatrix_.clone();
    cameraMatrix_.convertTo(cameraMatrix_show, CV_16F);
    ss << "camera_matrix: " << cameraMatrix_show;
    cv::putText(img, ss.str(), cv::Point(10, 400), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    ss.str("");
    ss.clear();

    cv::Mat distCoeffs_show = distCoeffs_.clone();
    distCoeffs_.convertTo(distCoeffs_show, CV_16F);
    ss << "distortion_coefficients: " << distCoeffs_show ;
    cv::putText(img, ss.str(), cv::Point(10, 420), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    ss.str("");
    ss.clear();

    // ss << "width: " << GreenRubbishWidth_;
    // cv::putText(img, ss.str(), cv::Point(10, 440), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    // ss << "hight: " << GreenRubbishHeight_ ;
    // cv::putText(img, ss.str(), cv::Point(10, 460), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    ss << "ball_radius: " << ball_radius_;
    cv::putText(img, ss.str(), cv::Point(10, 440), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    ss.str("");
    ss.clear();
}


PnPsolver::PnPsolver(const std::string& path){
    ParameterReader paramLoad(path);
    paramLoad.read("camera_matrix", cameraMatrix_);
    paramLoad.read("distortion_coefficients", distCoeffs_);
    paramLoad.read("width", GreenRubbishWidth_);
    paramLoad.read("hight", GreenRubbishHeight_);
//圆检测
    paramLoad.read("ball_radius", ball_radius_);

    objectPoints_.resize(8);
    // objectPoints_[0] = cv::Point3f(-GreenRubbishWidth_/2, GreenRubbishHeight_/2, 0);     //左上
    // objectPoints_[1] = cv::Point3f(GreenRubbishWidth_/2, GreenRubbishHeight_/2, 0);      //右上
    // objectPoints_[2] = cv::Point3f(GreenRubbishWidth_/2, -GreenRubbishHeight_/2, 0);     //右下
    // objectPoints_[3] = cv::Point3f(-GreenRubbishWidth_/2, -GreenRubbishHeight_/2, 0);    //左下
/*
增加更多点（比如16点）使轮廓更接近圆形
或者添加z方向的点来构建3D球体模型
*/
    objectPoints_.resize(8);
    for(int i = 0; i < 8; i++){
        double angle = 2 * CV_PI * i / 8;  // 顺时针
        objectPoints_[i] = cv::Point3f(ball_radius_ * cos(angle),  // 添加负号
                                    ball_radius_ * sin(angle), 
                                    0);
    }
}
/*
原点：相机光心
X轴：向右为正
Y轴：向下为正
Z轴：向前为正（沿光轴方向）
*/
bool PnPsolver::solvePnP(const std::vector<cv::Point2f>& imagePoints){
    try{
        // std::cout << "\n3D Model Points:" << std::endl;
        // for(size_t i = 0; i < objectPoints_.size(); i++) {
        //     std::cout << "Point " << i << ": (" 
        //              << objectPoints_[i].x << ", " 
        //              << objectPoints_[i].y << ", " 
        //              << objectPoints_[i].z << ")" << std::endl;
        // }
        
        // std::cout << "\nImage Points:" << std::endl;
        // for(size_t i = 0; i < imagePoints.size(); i++) {
        //     std::cout << "Point " << i << ": (" 
        //              << imagePoints[i].x << ", " 
        //              << imagePoints[i].y << ")" << std::endl;
        // }

        cv::solvePnP(objectPoints_, imagePoints, cameraMatrix_, distCoeffs_, rvec_, tvec_, false, cv::SOLVEPNP_ITERATIVE);
        // std::cout << "rvec: " << rvec_.t() << std::endl;
        // std::cout << "tvec: " << tvec_.t() << std::endl;        
        return true;
    }catch(cv::Exception& e){
        std::cerr << "Pnp: Error: " << e.what() << std::endl;
        return false;
    }

}


bool PnPsolver::calculatePose() {
    if (objectPoints_.empty()) {
        return false;
    }

    Eigen::Matrix3d R;
    cv::Mat R_cv;
    cv::Rodrigues(rvec_, R_cv);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_map(R_cv.ptr<double>(), 3, 3);
    R = R_map;

    Eigen::Vector3d tvec_eigen(tvec_.at<double>(0), tvec_.at<double>(1), tvec_.at<double>(2));
    
    //--取均值，增加z的可信性
    std::vector<Eigen::Vector3d> cameraPoints;
    // Eigen::Vector3d cameraPoint;
    for (const auto& objectPoint : objectPoints_) {
        Eigen::Vector3d worldPoint(objectPoint.x, objectPoint.y, objectPoint.z);
        Eigen::Vector3d cameraPoint = R * worldPoint + tvec_eigen;
        cameraPoints.emplace_back(cameraPoint);
    }

    Eigen::Vector3d centroid(0, 0, 0);
    for (const auto& pt : cameraPoints) {
        centroid += pt;
    }
    centroid /= cameraPoints.size();
    // cameraPoint = R * centroid + tvec_eigen;

    
    pose["x"] = centroid.x();
    pose["y"] = -centroid.y();
    pose["z"] = centroid.z();
    pose["distance"] = centroid.norm();

    return true;
}



}
