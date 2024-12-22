#ifndef FEATUREMATCH_H
#define FEATUREMATCH_H

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#include <detect/paramLoad.h>
#include <string>
#include <vector>

namespace detectAndTrack{

class ObjectTracker {
public:
    ObjectTracker();
    ~ObjectTracker() = default;

    void updateParams(const std::string& paramFile);
    // 初始化目标矩形框和跟踪器
    bool init(const cv::Mat& frame, const cv::Rect2d& initial_bbox);
    // 跟踪目标并返回更新后的矩形框
    cv::Rect2d update(const cv::Mat& frame);

    bool isInitialized() const { return  prev_bbox.width > 0 && prev_bbox.height > 0 ? true : false; }

public:
    cv::Mat prev_frame;
    cv::Rect2d prev_bbox;
    int8_t frame_count = 0;

private:
    cv::Size winSize;
    int maxLevel;
    cv::TermCriteria criteria;
    cv::Ptr<cv::Tracker> tracker;
    cv::Rect2d bbox;
    std::vector<cv::Point2f> prev_points;
    cv::Mat prev_gray;
};


}   // namespace detetcAndTrack


#endif // FEATUREMATCH_H


