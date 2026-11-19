#include <cassert>
#include <cmath>
#include "ptc_core/vision/observe_track_model.hpp"
int main() {
  ptc_core::ObserveTrackModel tracker(3.5);
  ptc_core::ObserveDetection rabbit{"Rabbit", {10,10,30,40}, {}};
  auto a = tracker.update({rabbit}, 1);
  assert(a.size()==1 && !a[0].geometry && !a[0].velocity);
  rabbit.box = {12,10,32,40};
  rabbit.points_world = {{1,2,3},{1,2,4}};
  auto b = tracker.update({rabbit}, 2);
  assert(b[0].id==a[0].id && b[0].geometry && !b[0].velocity);
  rabbit.points_world = {{1.1,2,3},{1.1,2,4}};
  auto c = tracker.update({rabbit}, 3);
  assert(c[0].id==a[0].id && c[0].velocity);
  assert(std::abs((*c[0].velocity)[0]-.1)<1e-10);
  auto twice=tracker.update({rabbit,rabbit},4);
  assert(twice[0].id!=twice[1].id);
  tracker.reset();
  auto reset=tracker.update({rabbit},.1);
  assert(reset[0].id!=a[0].id && !reset[0].velocity);
  assert(tracker.update({},.2).empty());
  assert(tracker.update({rabbit},5)[0].id!=reset[0].id);
  assert(!ptc_core::estimate_observed_geometry({{NAN,NAN,NAN}}));


  const ptc_core::ObserveBeltPrior belt{.06, .035};
  ptc_core::ObserveTrackModel rejoin(3.5, belt);
  ptc_core::ObserveDetection before_gap{"Rabbit", {10,10,30,40}, {{.6028,-2.1408,.7893}}};
  const auto old = rejoin.update({before_gap}, 222.9167)[0].id;
  const ptc_core::ObserveDetection after_gap{"Rabbit", {200,200,220,230}, {{.6047,-2.0986,.7806}}};
  assert(rejoin.update({after_gap}, 223.75)[0].id == old);
  auto distinct = after_gap;
  distinct.points_world = {{.68,-2.07,.78}};
  assert(rejoin.update({distinct}, 224.35)[0].id != old);

  ptc_core::ObserveTrackModel ambiguous(3.5, belt);
  auto other = before_gap;
  other.box = {80,10,100,40};
  other.points_world = {{.62,-2.1408,.78}};
  const auto competing = ambiguous.update({before_gap, other}, 222.9167);
  const auto unresolved = ambiguous.update({after_gap}, 223.75)[0];
  assert(unresolved.id != competing[0].id && unresolved.id != competing[1].id);
}
