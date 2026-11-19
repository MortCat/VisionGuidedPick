#include "ptc_core/state/event_context.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace ptc_core
{
std::optional<Nanoseconds> seconds_to_duration(double seconds)
{
  if (!std::isfinite(seconds) || seconds < 0.0 || (seconds > 0 && seconds < 1e-9) || seconds >= 9223372036.0) {return std::nullopt;}
  return Nanoseconds(static_cast<std::int64_t>(seconds * 1e9));
}
std::optional<Nanoseconds> deadline_after(Nanoseconds now, double seconds)
{
  const auto duration = seconds_to_duration(seconds);
  if (now.count() < 0 || !duration || duration->count() > std::numeric_limits<std::int64_t>::max() - now.count()) {
    return std::nullopt;
  }
  return now + *duration;
}
bool valid_time(const TimeContext & t)
{return t.epoch != 0 && t.source_now.count() >= 0 && t.monotonic_now.count() >= 0;}
bool source_is_fresh(Nanoseconds stamp, const TimeContext & t, double max_age, double future)
{
  const auto age = seconds_to_duration(max_age); const auto tolerance = seconds_to_duration(future);
  if (!valid_time(t) || stamp.count() < 0 || !age || !tolerance) {return false;}
  if (stamp > t.source_now) {return stamp - t.source_now <= *tolerance;}
  return t.source_now - stamp < *age;
}
bool same_cycle(const EventKey & a, const EventKey & b)
{return a.epoch != 0 && a.epoch == b.epoch && a.cycle_id != 0 && a.cycle_id == b.cycle_id &&
    a.track_id == b.track_id && !a.settings_revision.empty() && a.settings_revision == b.settings_revision;}
bool same_operation(const EventKey & a, const EventKey & b)
{return a.epoch != 0 && a.epoch == b.epoch && a.cycle_id == b.cycle_id && a.track_id == b.track_id &&
    !a.settings_revision.empty() && a.settings_revision == b.settings_revision && a.operation_id != 0 &&
    a.operation_id == b.operation_id && a.session_id == b.session_id;}
bool same_track_identity(const EventKey & a, const EventKey & b)
{return a.epoch != 0 && a.epoch == b.epoch && a.track_id != 0 && a.track_id == b.track_id &&
    !a.settings_revision.empty() && a.settings_revision == b.settings_revision;}
bool valid_observation_key(const EventKey & k, const TimeContext & t, const std::string & revision)
{return valid_time(t) && k.epoch == t.epoch && k.track_id != 0 && k.sequence != 0 && k.settings_revision == revision;}
int event_priority(bool fault, bool cancel, bool deadline)
{return fault ? 0 : (cancel ? 1 : (deadline ? 2 : 3));}
bool expired(Nanoseconds deadline, const TimeContext & t)
{return deadline.count() > 0 && t.monotonic_now >= deadline;}
bool append_history(std::deque<TransitionRecord> & history, TransitionRecord record, std::size_t capacity)
{
  if (capacity == 0) {return false;}
  if (history.size() >= capacity) {history.pop_front();}
  history.push_back(std::move(record)); return true;
}
bool valid_event(const EventKey & k, const TimeContext & t, const std::string & revision)
{
  return valid_time(t) && k.epoch == t.epoch && k.operation_id != 0 &&
    k.sequence != 0 && k.settings_revision == revision;
}
bool begin_operation(OperationContext & c, const EventKey & k, const TimeContext & t,
  const std::string & revision, double timeout)
{
  const auto deadline = deadline_after(t.monotonic_now, timeout);
  if (!valid_event(k, t, revision) || t.paused || !deadline || c.inhibited) {return false;}
  c.key = k; c.deadline = *deadline; c.last_monotonic = t.monotonic_now; return true;
}
bool emit_command(OperationContext & c, std::vector<Command> & output, CommandKind kind, const std::string & producer)
{
  if (c.next_command_id == 0 || c.next_command_id == std::numeric_limits<std::uint64_t>::max()) {
    c.inhibited = true; return false;
  }
  Command command; command.kind = kind; command.key = c.key; command.command_id = c.next_command_id++;
  command.producer = producer; command.deadline = c.deadline; output.push_back(command); return true;
}
TransitionRecord record_decision(OperationContext & c, const std::string & machine, int from, int to,
  const EventKey & key, const TimeContext & t, const DecisionStatus & status, std::size_t capacity)
{
  TransitionRecord record{machine, std::to_string(from), std::to_string(to), status.reason, key,
    t.source_now, t.monotonic_now, status.disposition};
  append_history(c.history, record, capacity);
  return record;
}
bool EventQueue::push(const EventKey & event)
{
  if (capacity == 0 || events.size() >= capacity) {overflowed = true; return false;}
  events.push_back(event); return true;
}
std::optional<EventKey> EventQueue::pop()
{
  if (events.empty()) {return std::nullopt;}
  auto event = events.front(); events.pop_front(); return event;
}
}  // namespace ptc_core
