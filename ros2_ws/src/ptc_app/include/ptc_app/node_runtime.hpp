#pragma once
#include "ptc_app/settings_loader.hpp"
#include "ptc_core/state/event_context.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <mutex>
#include <deque>
#include <functional>

namespace ptc_app
{
SettingsLoadResult load_node_settings(rclcpp::Node & node);
class TimeAdapter
{
public:
  ptc_core::TimeContext sample(rclcpp::Node &, const ptc_core::SystemSettings &);
private:
  ptc_core::Nanoseconds last_source_{0},last_change_{0}; std::uint64_t epoch_{1}; bool initialized_{false};
};
// Worker completion is marshalled back into the single owner executor.
class CompletionQueue
{
public:
  explicit CompletionQueue(std::size_t capacity) : capacity_(capacity) {}
  bool post(std::function<void()> completion);
  void drain();
  bool overflowed() const;
  void clear_fault(); // Explicit owner reset only, after old work is invalidated.
private:
  std::size_t capacity_; mutable std::mutex mutex_; std::deque<std::function<void()>> completions_; bool overflowed_{false};
};
}  // namespace ptc_app
