#include <iostream>
#include <chrono>
#include <deque>
#include <atomic>
#include <thread>
#include <mutex>


class FPSCalculator {
public:
    FPSCalculator(int queue_size = 10) 
        : max_queue_size(queue_size), fps(0.0), avg_fps(0.0), running(false) {}

    // 启动 FPS 计算线程
    void start() {
        running = true;
        fps_thread = std::thread(&FPSCalculator::calculateFPS, this);
    }

    // 停止 FPS 计算线程
    void stop() {
        running = false;
        if (fps_thread.joinable()) {
            fps_thread.join(); // 等待线程结束
        }
    }

    // 更新帧率，传入处理图像的时间戳
    void updateFrame() {
        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_time - last_frame_time;
        fps = 1.0 / elapsed_seconds.count(); // 计算当前帧率

        // 将当前帧率添加到队列
        std::lock_guard<std::mutex> lock(fps_mutex); // 保护队列
        fps_queue.push_back(fps);
        if (fps_queue.size() > max_queue_size) {
            fps_queue.pop_front(); // 保持队列大小
        }

        // 计算队列中帧率的平均值
        double sum = 0.0;
        for (double val : fps_queue) {
            sum += val;
        }
        avg_fps = sum / fps_queue.size();

        // 更新最后一帧的时间
        last_frame_time = end_time;
    }

    // 获取当前的平均 FPS
    double getAverageFPS() {
        std::lock_guard<std::mutex> lock(fps_mutex); // 线程安全访问平均 FPS
        return avg_fps;
    }

private:
    int max_queue_size;             // 队列的最大大小
    double fps;                     // 当前帧率
    double avg_fps;                 // 平均帧率
    std::deque<double> fps_queue;   // 存储帧率的队列
    std::thread fps_thread;         // 计算帧率的线程
    std::atomic<bool> running;      // 标记是否运行
    std::chrono::steady_clock::time_point last_frame_time; // 上一帧的时间
    std::mutex fps_mutex;           // 保护 FPS 计算的互斥锁

    // 计算帧率的线程函数
    void calculateFPS() {
        last_frame_time = std::chrono::steady_clock::now(); // 初始化起始时间戳
        while (running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 每100ms计算一次FPS
        }
    }
};
