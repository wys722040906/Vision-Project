#ifndef PNP_SOLVER_H
#define PNP_SOLVER_H

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <sstream>


#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

namespace detectAndTrack{

class PnPsolver{
public:
    PnPsolver(const std::string& path);
    ~PnPsolver() = default;

    bool solvePnP(const std::vector<cv::Point2f>& imagePoints); 

    bool calculatePose(void);

    void showParams(cv::Mat& img);

public:
    std::map<std::string, double> pose = {
        {"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"distance", 0.0}
    }; 

private:
    std::vector<cv::Point3f> objectPoints_;

    cv::Mat cameraMatrix_ = cv::Mat(3,3,CV_64F, cv::Scalar::all(0));
    cv::Mat distCoeffs_ = cv::Mat(1,5,CV_64F, cv::Scalar::all(0));
    cv::Mat rvec_;  
    cv::Mat tvec_;
    double GreenRubbishWidth_;
    double GreenRubbishHeight_;
    double ball_radius_;
    uint8_t pnpMethod_ = cv::SOLVEPNP_ITERATIVE;

};
};




#endif // PNP_SOLVER_H