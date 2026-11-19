#include "ptc_app/node_runtime.hpp"
#include <sstream>
#include <stdexcept>
namespace ptc_app
{
void CompletionQueue::clear_fault(){std::lock_guard<std::mutex> lock(mutex_);overflowed_=false;}
SettingsLoadResult load_node_settings(rclcpp::Node & node)
{
  const auto path=node.has_parameter("settings_file")?node.get_parameter("settings_file").as_string():node.declare_parameter<std::string>("settings_file","config/settings.yaml");
  auto loaded=SettingsLoader{}.load(path);
  if (!loaded.snapshot || loaded.snapshot->get().schema_version!=2) {
    std::ostringstream message; message<<"application requires ready schema v2: ";
    for (const auto & error : loaded.errors) {message<<error.path<<": "<<error.message<<"; ";}
    throw std::runtime_error(message.str());
  }
  if (node.get_namespace()!=std::string("/")+loaded.snapshot->get().runtime.workstation_namespace) {
    throw std::runtime_error("launch namespace differs from settings runtime.namespace");
  }
  const bool use_sim=node.has_parameter("use_sim_time") ? node.get_parameter("use_sim_time").as_bool() : false;
  if (use_sim!=loaded.snapshot->get().runtime.use_sim_time) {throw std::runtime_error("launch use_sim_time differs from Settings");}
  return loaded;
}
ptc_core::TimeContext TimeAdapter::sample(rclcpp::Node & node, const ptc_core::SystemSettings & settings)
{
  const auto mono=std::chrono::duration_cast<ptc_core::Nanoseconds>(std::chrono::steady_clock::now().time_since_epoch());
  const ptc_core::Nanoseconds source{node.now().nanoseconds()};
  if (!initialized_ || source!=last_source_) {
    if (initialized_ && source<last_source_) {++epoch_;}
    last_source_=source; last_change_=mono; initialized_=true;
  }
  const bool paused=settings.runtime.use_sim_time && (source.count()==0 ||
    mono-last_change_>=*ptc_core::seconds_to_duration(settings.vision.frame_max_age_sec));
  return {source,mono,epoch_,paused};
}
bool CompletionQueue::post(std::function<void()> completion)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (completions_.size()>=capacity_) {overflowed_=true; return false;}
  completions_.push_back(std::move(completion)); return true;
}
void CompletionQueue::drain()
{
  std::deque<std::function<void()>> work;
  {std::lock_guard<std::mutex> lock(mutex_); work.swap(completions_);}
  for (auto & completion : work) {completion();}
}
bool CompletionQueue::overflowed() const {std::lock_guard<std::mutex> lock(mutex_); return overflowed_;}
}  // namespace ptc_app
