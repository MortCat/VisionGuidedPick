#include "state_test_support.hpp"
#include <cstring>
#include <algorithm>
using namespace ptc_core;
using namespace ptc_test;

// Independent contract oracle: A=applied, R=rejected, I=ignored.
// Canonical states use valid identity/time but absent external device/view evidence.
// Positive guard branches are exercised by domain and full-cycle tests.
void expect(const DecisionStatus & status, char expected, const char * machine, int phase, int event)
{
  classified(status);
  const char actual = status.disposition == Disposition::kApplied ? 'A' : status.disposition == Disposition::kRejected ? 'R' : 'I';
  if (actual != expected) {throw std::runtime_error(std::string(machine)+" phase "+std::to_string(phase)+" event "+std::to_string(event)+
      " expected "+expected+" got "+actual+": "+status.reason);}
}
void tests()
{
  const auto cfg = settings(); std::size_t count = 0;
  const char * track_rows[] = {"RAAAA","RAAAA","RAAAA","IIIIA","IIIIA"};
  for (int p = 0; p < static_cast<int>(TrackPhase::kCount); ++p) {
    auto s = track(); s.phase = static_cast<TrackPhase>(p);
    for (int k = 0; k < static_cast<int>(TrackEventKind::kCount); ++k) {
      TrackEvent e; e.kind = static_cast<TrackEventKind>(k); e.key = key(11,2);
      const auto d = advance_track(s,e,cfg,time()); expect(d.status,track_rows[p][k],"track",p,k);
      if (d.status.disposition != Disposition::kApplied) {CHECK(d.state.phase == s.phase);} ++count;
    }
  }
  const char * motion_rows[] = {"RARIIIIRIIIIIII","RARIIIIRIIIIIII","RARIIIIRIIIIIII","AARAAIIRIAAIIIA","AARIIRARIAAIIIA","AARIIIIRIAARAII","IARIIIIRIIIIIRI","RARIIIIRIIIIIII","AARIIIIRRAAIIIA"};
  for (int p = 0; p <= static_cast<int>(ArmMotionState::kFollowing); ++p) {
    MotionState s; s.phase = static_cast<ArmMotionState>(p); s.context.key = key(40); s.context.deadline = *seconds_to_duration(10);
    s.operation_deadline = s.update_deadline = s.context.deadline; s.request = {{time().source_now,"base",{}},0.5,0.5,*seconds_to_duration(5)};
    s.active = p == 3 || p == 4 || p == 5 || p == 8; s.context.inhibited = p == 6;
    CHECK(std::strlen(motion_rows[p]) == static_cast<std::size_t>(MotionEventKind::kCount));
    for (int k = 0; k < static_cast<int>(MotionEventKind::kCount); ++k) {
      MotionEvent e; e.kind = static_cast<MotionEventKind>(k); e.key = key(40,2);
      if (e.kind == MotionEventKind::kPlanSucceeded) {e.scene = scene(); e.arm = arm();}
      if (e.kind == MotionEventKind::kTargetUpdated) {e.scene = scene(); e.arm = arm();}
      const auto d = advance_motion(s,e,cfg,time()); expect(d.status,motion_rows[p][k],"motion",p,k);
      if (d.status.disposition != Disposition::kApplied) {CHECK(d.state.phase == s.phase); CHECK(d.commands.empty());} ++count;
    }
  }
  const char * grasp_rows[] = {"RIIIIIIIIIIIIAIRI","IAIIIIIIIIIAAAIIA","IIAAIIIIIIIAAAIIA","IIIIRIIIIIIAAAIIA","IIIIRIRARIIIAAIRA","IIIIIIIIIRAAAAIIA","IIIIIIIIIIIIIAIRI","IIIIIIIIIIIIIAIRI"};
  for (int p = 0; p < static_cast<int>(GraspPhase::kCount); ++p) {
    GraspState s; s.phase = static_cast<GraspPhase>(p); s.context.key = key(40); s.context.deadline = *seconds_to_duration(10);
    s.possession = p == 4 || p == 5 ? Possession::kHolding : p == 3 || p == 7 ? Possession::kUnknown : Possession::kEmpty;
    s.scene_attached = s.lifted = p == 4;
    CHECK(std::strlen(grasp_rows[p]) == static_cast<std::size_t>(GraspEventKind::kCount));
    for (int k = 0; k < static_cast<int>(GraspEventKind::kCount); ++k) {
      GraspEvent e; e.kind = static_cast<GraspEventKind>(k); e.key = key(40,2);
      const auto d = advance_grasp(s,e,cfg,time()); expect(d.status,grasp_rows[p][k],"grasp",p,k);
      if (d.status.disposition != Disposition::kApplied) {CHECK(d.state.phase == s.phase); CHECK(d.commands.empty());}
      if (s.possession == Possession::kHolding) {for (const auto & command : d.commands) {CHECK(command.kind != CommandKind::kOpenGripper);}} ++count;
    }
  }
  const char * task_rows[] = {"RIIIIIIIIIIRIII","IAIAIIIAAAIIAIR","IAIIIIIAAAIIAII","IAAAIIIAAAIIAAI","IAAIRIIAAAIIAII","IAIARIIAAAIIAII","IAIARRIAAAIIAII","IIIIIIAIIIIIIII","IAIIIIIAAARIIII","IIIIIIIAIIRRIII"};
  for (int p = 0; p < static_cast<int>(TaskPhase::kCount); ++p) {
    if (p == 2) {continue;} // Reserved TaskStatus phase value.
    TaskState s; s.phase = static_cast<TaskPhase>(p); s.context.key = key(40); s.expected_operation_id = 40;
    s.context.deadline = s.cycle_deadline = *seconds_to_duration(10);
    s.grasp.phase = GraspPhase::kEmpty; s.grasp.possession = Possession::kEmpty;
    if (p == 5 || p == 6) {s.grasp.phase = GraspPhase::kHolding; s.grasp.possession = Possession::kHolding;}
    CHECK(std::strlen(task_rows[p]) == static_cast<std::size_t>(TaskEventKind::kCount));
    for (int k = 0; k < static_cast<int>(TaskEventKind::kCount); ++k) {
      TaskEvent e; e.kind = static_cast<TaskEventKind>(k); e.key = key(40,2);
      const auto d = advance_task(s,e,cfg,time()); expect(d.status,task_rows[p][k],"task",p,k);
      if (d.status.disposition != Disposition::kApplied) {CHECK(d.state.phase == s.phase); CHECK(d.commands.empty());} ++count;
    }
  }
  const char * scene_rows[] = {"RRAARIRRRARA","RRAIRIRRRARR","RRAIRARRRARR","RRAIRIRRRARR","RRRRRRRRRARR"};
  for (int p = 0; p <= static_cast<int>(AttachmentPhase::kUnknown); ++p) {
    auto s = scene(); s.attachment = static_cast<AttachmentPhase>(p); s.context.key = key(40); s.context.deadline = *seconds_to_duration(10);
    s.attached_track_id = p == 0 ? 0 : 1; s.context.inhibited = p == 4;
    std::string row = scene_rows[p]; row.erase(std::remove(row.begin(),row.end(),' '),row.end());
    CHECK(row.size() == static_cast<std::size_t>(SceneEventKind::kCount));
    for (int k = 0; k < static_cast<int>(SceneEventKind::kCount); ++k) {
      SceneEvent e; e.kind = static_cast<SceneEventKind>(k); e.key = key(40,2);
      const auto d = advance_scene(s,e,cfg,time()); expect(d.status,row[k],"scene",p,k);
      if (d.status.disposition != Disposition::kApplied) {CHECK(d.state.attachment == s.attachment); CHECK(d.commands.empty());} ++count;
    }
  }
  std::cout << count << " state/event classification cases verified.\n";
}
int main() {return run(tests);}
