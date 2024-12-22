#include "detect/detect.h"
#include "detect/paramLoad.h"

namespace detectAndTrack {
// Detect::Detect(const ThresholedObject& obj) : obj_(obj) {}

void Detect::whiteBalance(cv::Mat image_input, cv::Mat& image_output) {
    cv::Mat imageColor = image_input.clone();
    // 计算每个通道的平均值
    cv::Scalar mean = cv::mean(imageColor); 
    double avgGray = (mean[0] + mean[1] + mean[2]) / 3;

    // 计算每个通道的增益
    double gainR = avgGray / mean[2]; // 红色通道的增益
    double gainG = 0.9*avgGray / mean[1]; // 绿色通道的增益
    double gainB = avgGray / mean[0]; // 蓝色通道的增益

    // 应用增益
    for (int y = 0; y < imageColor.rows; ++y) {
        for (int x = 0; x < imageColor.cols; ++x) {
            cv::Vec3b& pixel = imageColor.at<cv::Vec3b>(y, x);
            pixel[0] = cv::saturate_cast<uchar>(pixel[0] * gainB); // B
            pixel[1] = cv::saturate_cast<uchar>(pixel[1] * gainG); // G
            pixel[2] = cv::saturate_cast<uchar>(pixel[2] * gainR); // R
        }
    }
}


void Detect::imgPreProcess(const cv::Mat& img, cv::Mat& dst){
    cv::Mat img_hsv, mask, src;

    src = img.clone();
    //图像增强
    if (obj_.white_balance_flag) {
    whiteBalance(src, src); 
    }
    if (obj_.auto_clahe_flag) {
    cv::Ptr<cv::xphoto::LearningBasedWB> wb = cv::xphoto::createLearningBasedWB();
    wb->setSaturationThreshold(0.8); 
    cv::Mat anvanced_wb_img(src.size(), src.type());
    wb->balanceWhite(src, src);  
    }
    if (obj_.clahe_flag) {
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    cv::Ptr<cv::CLAHE> clahe_green = cv::createCLAHE();
    clahe_green->setTilesGridSize(cv::Size(2, 2)); 
    clahe_green->setClipLimit(2.0);
    clahe_green->apply(channels[1], channels[1]);
    cv::merge(channels, src);
    }   
    if (obj_.auto_clahe_flag) {
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    cv::Ptr<cv::CLAHE> clahe_green = cv::createCLAHE();
    clahe_green->setTilesGridSize(cv::Size(2, 2)); 
    clahe_green->setClipLimit(2.0); 
    clahe_green->apply(channels[1], channels[1]);
    cv::merge(channels, src);
    }

    cv::cvtColor(src, img_hsv, cv::COLOR_BGR2HSV);
    cv::Scalar lower_hsv(obj_.lower_h, obj_.lower_s, obj_.lower_v);
    cv::Scalar upper_hsv(obj_.upper_h, obj_.upper_s, obj_.upper_v);
    cv::inRange(img_hsv, lower_hsv, upper_hsv, mask);

    //GaussianBlur
    // GaussianBlur(mask, mask, cv::Size(3, 3), 0);
    // cv::Canny(mask, mask, obj_.CannyLowThresh, obj_.CannyHighThresh);

    //形态学操作
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    for(uint i = 0; i < obj_.DialteIter; i++){
        cv::dilate(mask, mask, element);
    }
    for(uint i = 0; i < obj_.ErodeIter; i++){
        cv::erode(mask, mask, element);
    }
    for(uint i = 0; i < obj_.OpenIter; i++){
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element); //开运算--去噪点
    }
    for(uint i = 0; i < obj_.CloseIter; i++){
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element); //闭运算--连通分离
    }

    //Canny边缘检测
    dst = mask;
}

Ball Detect::detectObject(const cv::Mat& img, cv::Mat& dst){
    Ball ball = Ball();
    ball.contour.resize(obj_.point_num);
    // std::vector<std::vector<cv::Point>> contours;
    //垃圾桶检测
    // cv::Mat hierarchy;
    // int mode = cv::RETR_EXTERNAL;
    // int method = cv::CHAIN_APPROX_SIMPLE;

    // cv::Mat imgGray = img.clone();
    // cv::findContours(imgGray, contours, hierarchy, mode, method);

    // // rubbiish.status = false;
    // for(int i = 0; i < contours.size(); i++){ 
    //     cv::Rect rect = cv::boundingRect(contours[i]);
    //     cv::Point center = cv::Point(rect.x + rect.width/2, rect.y + rect.height/2);
    //     if(rect.area() > obj_.min_area && rect.area() < obj_.max_area && 
    //         static_cast<float>(rect.width) / static_cast<float>(rect.height) > obj_.min_rabbish_length_width_ration && 
    //         static_cast<float>(rect.width) / static_cast<float>(rect.height) < obj_.max_rabbish_length_width_ration && 
    //         cv::pointPolygonTest(contours[i], center, false) >= 0){
    //         cv::rectangle(dst, rect, cv::Scalar(0, 255, 0), 2);
    //         rubbiish.rect = rect;
    //         rubbiish.center = center;
    //         rubbiish.left_top = cv::Point(rect.x, rect.y);
    //         rubbiish.right_top = cv::Point(rect.x + rect.width, rect.y);
    //         rubbiish.right_bottom = cv::Point(rect.x + rect.width, rect.y + rect.height);
    //         rubbiish.left_bottom = cv::Point(rect.x, rect.y + rect.height);
    //         rubbiish.status = true;
    //     }
    // }
        //霍夫圆检测
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(img, circles, cv::HOUGH_GRADIENT, 
                    std::max(1,  obj_.hough_dp),  // dp不能为0
                    std::max(1, obj_.hough_min_dist),  // minDist不能为0
                    obj_.hough_param1, 
                    obj_.hough_param2,
                    obj_.hough_min_radius, 
                    obj_.hough_max_radius);
        // 查找轮廓（用于计算圆度）
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 绘制符合条件的圆
         // 绘制符合条件的圆
        for (std::size_t i = 0; i < circles.size(); i++) {
            cv::Point center(static_cast<int>(std::round(circles[i][0])), static_cast<int>(std::round(circles[i][1])));
            int radius = static_cast<int>(std::round(circles[i][2]));
            
            // 创建圆形mask来获取当前圆的轮廓
            cv::Mat circleMask = cv::Mat::zeros(img.size(), CV_8UC1);
            cv::circle(circleMask, center, radius, cv::Scalar(255), -1);
            
            // 找到与当前圆重叠最多的轮廓
            int bestContourIdx = -1;
            double maxOverlap = 0;
            
            for (std::size_t j = 0; j < contours.size(); j++) {
                cv::Rect boundRect = cv::boundingRect(contours[j]);
                if (center.x >= boundRect.x && center.x <= boundRect.x + boundRect.width &&
                    center.y >= boundRect.y && center.y <= boundRect.y + boundRect.height) {
                    
                    double area = cv::contourArea(contours[j]);
                    if (area > obj_.circle_area_thresh) {
                        double aspect_ratio = static_cast<double>(boundRect.width) / boundRect.height;
                        if (std::fabs(aspect_ratio - 1.0) < obj_.circle_ratio_thresh/100.0) {
                            bestContourIdx = j;
                            break;
                        }
                    }
                }
            }
            
            // 如果找到合适的轮廓，绘制圆
            if (bestContourIdx >= 0) {
                // 绘制圆心
                cv::circle(dst, center, 3, cv::Scalar(0, 255, 0), -1);
                // 绘制圆轮廓
                cv::circle(dst, center, radius, cv::Scalar(0, 255, 0), 2);
                ball.center = center;
                ball.radius = radius;
                ball.status = true;
                for(int i = 0; i < obj_.point_num; i++){
                    double angle =2 * CV_PI * i / obj_.point_num;  // 添加负号，改为顺时针
                    cv::Point2f point(center.x + radius * std::cos(angle), 
                                    center.y + radius * std::sin(angle));
                    ball.contour[i] = point;
                }
                ball.status = true;
            }
        }

    return ball;
}

void Detect::updateThresholed(const std::string& path) {
    ParameterReader paramLoad(path);
    paramLoad.read("auto_clahe_flag", obj_.auto_clahe_flag);
    paramLoad.read("clahe_flag", obj_.clahe_flag);
    paramLoad.read("white_balance_flag", obj_.white_balance_flag);
    paramLoad.read("auto_wb_flag", obj_.auto_wb_flag);

    paramLoad.read("lower_h", obj_.lower_h);
    paramLoad.read("lower_s", obj_.lower_s);
    paramLoad.read("lower_v", obj_.lower_v);
    paramLoad.read("upper_h", obj_.upper_h);
    paramLoad.read("upper_s", obj_.upper_s);
    paramLoad.read("upper_v", obj_.upper_v);

    paramLoad.read("DialteIter", obj_.DialteIter);
    paramLoad.read("ErodeIter", obj_.ErodeIter);
    paramLoad.read("OpenIter", obj_.OpenIter);
    paramLoad.read("CloseIter", obj_.CloseIter);
    paramLoad.read("CannyLowThresh", obj_.CannyLowThresh);
    paramLoad.read("CannyHighThresh", obj_.CannyHighThresh);

    paramLoad.read("min_area", obj_.min_area);
    paramLoad.read("max_area", obj_.max_area);
    paramLoad.read("min_rabbish_length_width_ration", obj_.min_rabbish_length_width_ration);
    paramLoad.read("max_rabbish_length_width_ration", obj_.max_rabbish_length_width_ration);

    paramLoad.read("hough_dp", obj_.hough_dp);
    paramLoad.read("hough_min_dist", obj_.hough_min_dist);
    paramLoad.read("hough_param1", obj_.hough_param1);
    paramLoad.read("hough_param2", obj_.hough_param2);
    paramLoad.read("hough_min_radius", obj_.hough_min_radius);
    paramLoad.read("hough_max_radius", obj_.hough_max_radius);
    paramLoad.read("circle_area_thresh", obj_.circle_area_thresh);
    paramLoad.read("circle_ratio_thresh", obj_.circle_ratio_thresh);
    paramLoad.read("point_num", obj_.point_num);
}


void Detect::showThresholed(cv::Mat& img){
    std::stringstream ss;
    // ss << "auto_clahe_flag: " << obj_.auto_clahe_flag;
    // cv::putText(img, ss.str(), cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    // ss << "clahe_flag: " << obj_.clahe_flag;
    // cv::putText(img, ss.str(), cv::Point(10, 40), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();
    
    // ss << "white_balance_flag: " << obj_.white_balance_flag;
    // cv::putText(img, ss.str(), cv::Point(10, 60), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    ss << "lower_h: " << obj_.lower_h;
    cv::putText(img, ss.str(), cv::Point(10, 80), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    ss.str("");
    ss.clear();

    ss << "lower_s: " << obj_.lower_s ;
    cv::putText(img, ss.str(), cv::Point(10, 100), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    ss.str("");
    ss.clear();

    ss << "lower_v: " << obj_.lower_v;
    cv::putText(img, ss.str(), cv::Point(10, 120), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    ss.str("");
    ss.clear();

    ss << "upper_h: " << obj_.upper_h;
    cv::putText(img, ss.str(), cv::Point(10, 140), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    ss.str("");
    ss.clear();

    ss << "upper_s: " << obj_.upper_s;
    cv::putText(img, ss.str(), cv::Point(10, 160), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    ss.str("");
    ss.clear();

    ss << "upper_v: " << obj_.upper_v;
    cv::putText(img, ss.str(), cv::Point(10, 180), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    ss.str("");
    ss.clear();

    // ss << "DialteIter: " << obj_.DialteIter;
    // cv::putText(img, ss.str(), cv::Point(10, 200), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    // ss << "ErodeIter: " << obj_.ErodeIter;
    // cv::putText(img, ss.str(), cv::Point(10, 220), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    // ss << "OpenIter: " << obj_.OpenIter;
    // cv::putText(img, ss.str(), cv::Point(10, 240), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    // ss << "CloseIter: " << obj_.CloseIter;
    // cv::putText(img, ss.str(), cv::Point(10, 260), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str(""); 
    // ss.clear();

    // ss << "CannyLowThresh: " << obj_.CannyLowThresh;
    // cv::putText(img, ss.str(), cv::Point(10, 280), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    // ss << "CannyHighThresh: " << obj_.CannyHighThresh;     
    // cv::putText(img, ss.str(), cv::Point(10, 300), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    // ss << "min_area: " << obj_.min_area;   
    // cv::putText(img, ss.str(), cv::Point(10, 320), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear(); 
    
    // ss << "max_area: " << obj_.max_area;
    // cv::putText(img, ss.str(), cv::Point(10, 340), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    // ss << "min_rabbish_length_width_ration: " << obj_.min_rabbish_length_width_ration;
    // cv::putText(img, ss.str(), cv::Point(10, 360), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    // ss << "max_rabbish_length_width_ration: " << obj_.max_rabbish_length_width_ration;
    // cv::putText(img, ss.str(), cv::Point(10, 380), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    // ss << "circle_area_thresh: " << obj_.circle_area_thresh;
    // cv::putText(img, ss.str(), cv::Point(10, 400), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear(); 

    // ss << "circle_ratio_thresh: " << obj_.circle_ratio_thresh;
    // cv::putText(img, ss.str(), cv::Point(10, 420), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear(); 

    // ss << "point_num: " << obj_.point_num;
    // cv::putText(img, ss.str(), cv::Point(10, 440), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    // ss << "hough_min_radius: " << obj_.hough_min_radius;
    // cv::putText(img, ss.str(), cv::Point(10, 460), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    // ss << "hough_max_radius: " << obj_.hough_max_radius;
    // cv::putText(img, ss.str(), cv::Point(10, 480), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    // ss << "circle_area_thresh: " << obj_.circle_area_thresh;
    // cv::putText(img, ss.str(), cv::Point(10, 500), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear();

    // ss << "circle_ratio_thresh: " << obj_.circle_ratio_thresh;
    // cv::putText(img, ss.str(), cv::Point(10, 520), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear(); 

    // ss << "point_num: " << obj_.point_num;
    // cv::putText(img, ss.str(), cv::Point(10, 540), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    // ss.str("");
    // ss.clear(); 
}

} // namespace detect