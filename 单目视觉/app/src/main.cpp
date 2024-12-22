#include <main.h>

using namespace detectAndTrack;

/*多线程加速 
-- 生产者消费者模型 
    -- 生产者线程负责从摄像头中读取帧，并将帧放入队列中，
       消费者线程负责从队列中取出帧，进行处理，并将结果放入另一个队列中。
-- 设备限制
*/ 

int main(int argc, char *argv[]) {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera" << std::endl;
        return -1;
    }
    cv::Mat frame = cv::Mat::zeros(720, 1280, CV_8UC3);
    cv::Mat paramPage = cv::Mat::zeros(900, 800, CV_8UC3); 
    cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("paramPage", cv::WINDOW_AUTOSIZE);
    std::string configFile = "/home/wys/Desktop/Project/VisionProject/单目视觉/app/config/config.yaml";

    Detect ball;
    ball.updateThresholed(configFile);
    ball.showThresholed(paramPage);

    PnPsolver pnpSolver(configFile);
    pnpSolver.showParams(paramPage);
    std::map<std::string, double> pose;

    ExtendedKalmanFilter ekf;
    ekf.updateParam(configFile);
    ekf.setTransitionFunction([ekf](const Eigen::VectorXf& state){
        Eigen::VectorXf newState(state.size());
        newState << 
            state(0) + state(3) * ekf.timeStep,
            state(1) + state(4) * ekf.timeStep,
            state(2) + state(5) * ekf.timeStep,
            state(3),
            state(4),
            state(5);
        return newState;
    });
    ekf.setMeasurementFunction([ekf](const Eigen::VectorXf& state){
        return state.head<3>();  //需返回[x,y,z]
        // Eigen::VectorXf measurement(state.size());
        // measurement << state(0), state(1), state(2);
        // return measurement;
    });
    Eigen::VectorXf measuremen(3);
    std::vector<double> predictPos;

    ObjectTracker tracker;
    tracker.updateParams(configFile);

//帧率计数
    FPSCalculator fpsCaculator(10);
    fpsCaculator.start();

//多线程处理
    // std::vector<cv::Mat> frames(5);  // 存储多个帧的容器
    // std::vector<std::future<void>> futures(5);  // 存储每个任务的future对象
    // std::vector<Rubbiish> results(5);  // 存储每帧处理结果的容器
    // ThreadPool threadPool(5);  // 线程池，容量为5

//可视化
    QApplication app(argc, argv);
    std::vector<std::vector<float>> predictPoints;
    QMainWindow window;
    ParabolaViewer* viewer = new ParabolaViewer(&window);
    window.setCentralWidget(viewer);
    window.resize(800, 600);
    window.show();
    viewer->setPlaneHeight(-50);
// 空数据计数
    int emptyDataCount = 0;
    int predictCount = 0;
    const int CLEAR_THRESHOLD = 30;
    const int WINDOW_SIZE = 200;

// 创建定时器来处理事件
    QTimer *timer = new QTimer(&window);
    QObject::connect(timer, &QTimer::timeout, [&]() {
    // while (true) {
        cap >> frame;
        cv::Mat frameGray;
        ball.imgPreProcess(frame,frameGray);
        Ball BalRect =  ball.detectObject(frameGray, frame);  
        // int num_threads = cv::getNumThreads();
        // std::cout << "OpenCV is using " << num_threads << " threads." << std::endl;

        //欠缺--只要检测到一次，就默认初始化，若是后续跟丢呢--前一帧检测到--后几帧跟丢--消除记录
        //没有吊用
        // if(GreenRubbishRect.status ){
        //     tracker.prev_frame = frame.clone();
        //     tracker.prev_bbox = GreenRubbishRect.rect;
        // }

        // if(!GreenRubbishRect.status && tracker.isInitialized()){
        //     tracker.init(tracker.prev_frame, tracker.prev_bbox);    //只有相邻帧才初始化   
        //     GreenRubbishRect.rect = tracker.update(frame);
        //     if(GreenRubbishRect.rect.area() > 0 && 
        //     GreenRubbishRect.rect.area() > 1000 && 
        //     GreenRubbishRect.rect.area() < 2000000 && 
        //     static_cast<float>(GreenRubbishRect.rect.width) / static_cast<float>(GreenRubbishRect.rect.height) > 0.4 && 
        //     static_cast<float>(GreenRubbishRect.rect.width) / static_cast<float>(GreenRubbishRect.rect.height) < 0.8 
        //     ){
        //     GreenRubbishRect.left_top = cv::Point2f(GreenRubbishRect.rect.x, GreenRubbishRect.rect.y);
        //     GreenRubbishRect.right_top = cv::Point2f(GreenRubbishRect.rect.x + GreenRubbishRect.rect.width, GreenRubbishRect.rect.y);
        //     GreenRubbishRect.right_bottom = cv::Point2f(GreenRubbishRect.rect.x + GreenRubbishRect.rect.width, GreenRubbishRect.rect.y + GreenRubbishRect.rect.height);
        //     GreenRubbishRect.left_bottom = cv::Point2f(GreenRubbishRect.rect.x, GreenRubbishRect.rect.y + GreenRubbishRect.rect.height);
        //     GreenRubbishRect.status = true;
        //     cv::rectangle(frame, GreenRubbishRect.rect, cv::Scalar(0, 0, 255), 2);
        //     if(++tracker.frame_count >= 10){
        //         tracker.frame_count = 0;
        //         tracker.prev_frame = cv::Mat();
        //         tracker.prev_bbox = cv::Rect2d();
        //     }
        //     static int count_ = 0;
        //     std::cout << "跟踪计数 count: " << count_++ << " 次"  <<  std::endl;
        //     }
        // }
        if(BalRect.status){
            //  std::cout << "检测到球" << BalRect.contour.size() << std::endl;         
            pnpSolver.solvePnP(BalRect.contour);

            pnpSolver.calculatePose();
            pose =  pnpSolver.pose;
            if(!measuremen.isZero()){measuremen.setZero();}
            measuremen << pose["x"], pose["y"], pose["z"];
            ekf.predict();
            ekf.update(measuremen);
            predictPos = ekf.printState();
            if (!predictPos.empty()) {
                std::vector<float> floatPredictPos(predictPos.begin(), predictPos.end());
                
                // 检查新点是否与已存储的点过于接近
                const float MIN_DISTANCE = 0.05f; // 最小距离阈值（单位：米）
                bool tooClose = false;
                
                for (const auto& existingPoint : predictPoints) {
                    float dx = existingPoint[0] - floatPredictPos[0];
                    float dy = existingPoint[1] - floatPredictPos[1];
                    float dz = existingPoint[2] - floatPredictPos[2];
                    float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                    
                    if (distance <= MIN_DISTANCE) {
                        tooClose = true;
                        break;
                    }
                }
                // 只有当点太密集时才添加
                if (!tooClose) {
                    if (predictPoints.size() >= WINDOW_SIZE) {
                        // 找到与新点距离最近的点
                        double min_dist = std::numeric_limits<double>::max();
                        int min_idx = 0;
                        
                        for(size_t i = 0; i < predictPoints.size(); i++) {
                            double dx = predictPoints[i][0] - floatPredictPos[0];
                            double dy = predictPoints[i][1] - floatPredictPos[1];
                            double dz = predictPoints[i][2] - floatPredictPos[2];
                            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                            
                            if(dist < min_dist) {
                                min_dist = dist;
                                min_idx = i;
                            }
                        }
                        
                        // 删除距离最近的点
                        predictPoints.erase(predictPoints.begin() + min_idx);
                    }
                    predictPoints.push_back(floatPredictPos);
                }
                emptyDataCount = 0;
            }
        }else{
            emptyDataCount++;
            // 如果在窗口期内累积了足够多的空帧，清空所有点和显示
            if (emptyDataCount >= CLEAR_THRESHOLD) {
                predictPoints.clear();
                viewer->clearAll();  // 清空显示的抛物线
                emptyDataCount = 0;
            }
        
            pose["x"] = 0.0f;
            pose["y"] = 0.0f;
            pose["z"] = 0.0f;
            pose["distance"] = 0.0f;
            predictPos.clear();
            // std::cout << "没有检测到球" << std::endl;
        }
        // 添加安全检查
        if (!predictPos.empty()) {
            std::cout << std::fixed << std::setprecision(6);
            std::cout << "实际值: x: " << pose["x"] << " y: " << pose["y"] << " z: " << pose["z"] << std::endl;
            std::cout << "预测值: x: " << predictPos[0] << " y: " << predictPos[1] << " z: " << predictPos[2] << std::endl;
            std::cout << std::defaultfloat;

            // 只在有预测值时显示预测信息
            cv::putText(frame, "predict x: " + std::to_string(predictPos[0]), cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
            cv::putText(frame, "predict y: " + std::to_string(predictPos[1]), cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
            cv::putText(frame, "predict z: " + std::to_string(predictPos[2]), cv::Point(10, 140), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
            if (predictPos.size() > 3) {  // 确保有distance值
                cv::putText(frame, "predict distance" + std::to_string(predictPos[3]), cv::Point(10, 160), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
            }
        }

        // 始终显示实际值
        cv::putText(frame, "x: " + std::to_string(pose["x"]), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        cv::putText(frame, "y: " + std::to_string(pose["y"]), cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        cv::putText(frame, "z: " + std::to_string(pose["z"]), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        cv::putText(frame, "distance" + std::to_string(pose["distance"]), cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

        // 只在有足够点时显示
        if (predictPoints.size() >= 3) {
            // 转换为厘米显示
            std::vector<std::vector<float>> displayPoints;
            for (const auto& p : predictPoints) {
                displayPoints.push_back({
                    p[0] * 100.0f,  // 米转厘米
                    p[1] * 100.0f,
                    p[2] * 100.0f
                });
            }
            viewer->setPoints(displayPoints);
        }

        // cv::circle(frame, cv::Point2f(static_cast<int>(pose["x"]), static_cast<int>(pose["y"])), 5, cv::Scalar(0, 0, 255), 2);
        // cv::circle(frame, cv::Point2f(static_cast<int>(predictPos[0]), static_cast<int>(predictPos[1])), 5, cv::Scalar(255, 0, 0), 2);

//帧率计数
        fpsCaculator.updateFrame();
        double fps = fpsCaculator.getAverageFPS();
        // cv::putText(frame, "FPS: " + std::to_string(static_cast<int>(fps)), cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        std::cout << "FPS: " << static_cast<int>(fps) << std::endl;    

        cv::imshow("frame", frame);
        cv::imshow("paramPage", paramPage);
        cv::waitKey(1);

        // 处理Qt事件
        QApplication::processEvents();
    });
    timer->start(10);  // 约60fps的更新率

    fpsCaculator.stop();

    // 主事件循环
    return app.exec();
}   



    