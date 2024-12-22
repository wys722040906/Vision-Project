#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <future>

/*
    // std::vector<cv::Mat> frames(5);  // 存储多个帧的容器
    // std::vector<std::future<void>> futures(5);  // 存储每个任务的future对象
    // std::vector<Rubbiish> results(5);  // 存储每帧处理结果的容器
    // ThreadPool threadPool(5);  // 线程池，容量为5


        for(auto& x : frames){
            cap >> x;
        }
        for(size_t i = 0; i < frames.size(); i++){
            futures[i] = threadPool.enqueue([i, &frames, &results, &GreenRubbish](){
                cv::Mat frameGray;
                GreenRubbish.imgPreProcess(frames[i],frameGray);
                Rubbiish GreenRubbishRect =  GreenRubbish.detectObject(frameGray, frames[i]);  
                results[i] = GreenRubbishRect;
            });
        }
        for(auto& x : futures){
            x.get();
        }
        results.erase(
            std::remove_if(results.begin(), results.end(), 
                [](const Rubbiish& rect) { return !rect.status; }), 
            results.end()
        );


*/



// 线程池类
class ThreadPool {
public:
    // 构造函数，指定线程数量
    ThreadPool(size_t numThreads) : stop(false) {
        // 启动指定数量的线程
        for (size_t i = 0; i < numThreads; ++i) {
            workers.emplace_back([this] {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(queueMutex);
                        condition.wait(lock, [this] { return stop || !tasks.empty(); });
                        if (stop && tasks.empty()) return;
                        task = std::move(tasks.front());
                        tasks.pop();
                    }
                    task();
                }
            });
        }
    }

    // 提交任务
    template <typename F>
    std::future<void> enqueue(F&& f) {
        auto task = std::make_shared<std::packaged_task<void()>>(std::forward<F>(f));
        std::future<void> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            tasks.push([task] { (*task)(); });
        }
        condition.notify_one();
        return res;
    }

    // 停止线程池
    void stopAll() {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread& worker : workers) {
            worker.join();
        }
    }

    ~ThreadPool() {
        if (!stop) stopAll();
    }

private:
    std::vector<std::thread> workers;  // 工作线程
    std::queue<std::function<void()>> tasks;  // 任务队列
    std::mutex queueMutex;  // 任务队列的互斥锁
    std::condition_variable condition;  // 条件变量，用于通知空闲线程
    bool stop;  // 是否停止线程池
};