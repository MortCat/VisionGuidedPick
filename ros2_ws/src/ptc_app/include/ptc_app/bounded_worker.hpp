#pragma once
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

namespace ptc_app
{
// One serial worker for model/geometry/I/O work; owners handle rejected enqueue.
class BoundedWorker
{
public:
  explicit BoundedWorker(std::size_t capacity) : capacity_(capacity),thread_([this] {run();}) {}
  ~BoundedWorker() {shutdown();}
  BoundedWorker(const BoundedWorker &)=delete;
  BoundedWorker & operator=(const BoundedWorker &)=delete;
  bool enqueue(std::function<void()> work)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || pending_.size()>=capacity_) {return false;}
    pending_.push_back(std::move(work)); condition_.notify_one(); return true;
  }
  void shutdown()
  {
    {std::lock_guard<std::mutex> lock(mutex_); stopping_=true; pending_.clear();}
    condition_.notify_one(); if (thread_.joinable()) {thread_.join();}
  }
private:
  void run()
  {
    for (;;) {
      std::function<void()> work;
      {std::unique_lock<std::mutex> lock(mutex_); condition_.wait(lock,[this] {return stopping_ || !pending_.empty();});
        if (stopping_) {return;} work=std::move(pending_.front()); pending_.pop_front();}
      work(); // Each submitted boundary task owns its error/result handling.
    }
  }
  std::size_t capacity_; std::mutex mutex_; std::condition_variable condition_;
  std::deque<std::function<void()>> pending_; bool stopping_{false}; std::thread thread_;
};
}  // namespace ptc_app
