#ifndef DETECT_H
#define DETECT_H

#include <opencv2/opencv.hpp>
#include <opencv2/xphoto.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <sstream>



namespace detectAndTrack {

    struct ThresholedObject {
        //颜色约束
        int lower_h = 61, lower_s = 94, lower_v = 87;
        int upper_h = 80, upper_s = 148, upper_v = 144;

        int ErodeIter = 0;
        int DialteIter = 6;
        int OpenIter = 0;
        int CloseIter = 0;
        int CannyLowThresh = 50;
        int CannyHighThresh = 150;

        int white_balance_flag = 1;
        int auto_wb_flag = 0;
        int clahe_flag = 0;
        int auto_clahe_flag = 0;

        //几何约束
        int min_area = 100;
        int max_area = 500000;
        float min_rabbish_length_width_ration = 0.1;
        float max_rabbish_length_width_ration = 1;

        //圆检测    
        int hough_dp = 2;
        int hough_min_dist = 200;
        int hough_param1 = 50;
        int hough_param2 = 29;
        int hough_min_radius = 20;
        int hough_max_radius = 100;
        int circle_area_thresh = 200;
        int circle_ratio_thresh = 40;
        int point_num = 8;
    };
    struct Rubbiish {
        cv::Rect rect;
        cv::Point2f center;
        cv::Point2d left_top;
        cv::Point2d right_top;
        cv::Point2d right_bottom;
        cv::Point2d left_bottom;
        bool status = false;

        // // 默认构造函数
        Rubbiish() = default;

        // // 拷贝构造函数
        // Rubbiish(const Rubbiish& other) = default;

        // // 移动构造函数
        // Rubbiish(Rubbiish&& other) noexcept = default;

        // // 拷贝赋值运算符
        // Rubbiish& operator=(const Rubbiish& other) = default;

        // // 移动赋值运算符
        // Rubbiish& operator=(Rubbiish&& other) noexcept = default;
    };
    struct Ball {
        Ball() = default;
        ~Ball() = default;
        int radius;
        cv::Point center;
        cv::Rect rect;
        std::vector<cv::Point2f> contour;
        bool status = false;
    };
    class Detect {
        public:
            Detect() = default;
            ~Detect() = default;
            void updateThresholed(const std::string& path);
            void showThresholed(cv::Mat& img);
            void imgPreProcess(const cv::Mat& img, cv::Mat& dst);
            Ball detectObject(const cv::Mat& img,cv::Mat& dst);
            void startDetect(const cv::Mat& img);

        private:
            void whiteBalance(cv::Mat image_input, cv::Mat& image_output);
            ThresholedObject obj_;
        
    };
}

#endif