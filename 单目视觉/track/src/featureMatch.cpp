#include "track/featureMatch.h"


namespace detectAndTrack{

void ObjectTracker::updateParams(const std::string& paramFile){
    ParameterReader paramLoad(paramFile);
    uint8_t winSize_,maxLevel_,maxCount_,minEps_;
    paramLoad.read("winSize", winSize_);
    paramLoad.read("maxLevel", maxLevel_);
    paramLoad.read("maxCount", maxCount_);
    paramLoad.read("minEps", minEps_);
    winSize = cv::Size(winSize_, winSize_);
    maxLevel = maxLevel_;
    criteria = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, maxCount_, minEps_);
}


ObjectTracker::ObjectTracker()
    : winSize(15, 15), maxLevel(2), criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 10, 0.03) {
    tracker = cv::TrackerCSRT::create();
}


// 初始化目标矩形框和跟踪器
bool ObjectTracker::init(const cv::Mat& frame, const cv::Rect2d& initial_bbox) {
    bbox = initial_bbox;
    prev_points.clear();
    prev_gray.release();
    tracker->init(frame, bbox);
    cv::cvtColor(frame, prev_gray, cv::COLOR_BGR2GRAY);

    // 计算初始特征点
    cv::Mat mask = cv::Mat::zeros(prev_gray.size(), CV_8UC1);
    cv::rectangle(mask, bbox, cv::Scalar(255), -1);
    cv::goodFeaturesToTrack(prev_gray, prev_points, 100, 0.3, 7, mask, 7);

    return true;
}

// 跟踪目标并返回更新后的矩形框
cv::Rect2d ObjectTracker::update(const cv::Mat& frame) {
    if (frame.empty()) return bbox;

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    if (!prev_points.empty() && !prev_gray.empty()) {
        // 计算光流
        std::vector<cv::Point2f> next_points;
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_gray, gray, prev_points, next_points, status, err, winSize, maxLevel, criteria);

        // 选取有效的特征点
        std::vector<cv::Point2f> good_points;
        for (size_t i = 0; i < status.size(); i++) {
            if (status[i]) {
                good_points.push_back(next_points[i]);
            }
        }

        if (!good_points.empty()) {
            // 更新边界框
            bbox = cv::boundingRect(good_points);
            prev_points = good_points; // 更新特征点
        }
    }

    // 更新前一帧图像
    prev_gray = gray.clone();
    return bbox;
}


} // namespace detectAndTrack