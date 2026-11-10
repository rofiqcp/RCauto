#include "rcauto_cpp/core.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace rcauto;
Observation safe(bool crossing=false,std::string signal="UNKNOWN") {
  Observation o;
  o.error=.1;o.confidence=.9;o.lane_age=.03;o.hazard=false;o.hazard_age=.03;
  o.intersection=crossing;o.signal=signal;o.signal_age=.03;return o;
}
int main() {
  // Range obstacle clears on the next tick; independent vision hazard persists.
  assert(combined_hazard(false, true));
  assert(!combined_hazard(false, false));
  assert(combined_hazard(true, false));
  assert(combined_hazard(true, true));
  Pid pid;
  assert(pid.calculate(.3,.05)>0);
  pid.reset();assert(pid.calculate(-.3,.05)<0);
  try{pid.calculate(std::numeric_limits<double>::quiet_NaN(),.05);return 9;}
  catch(const std::invalid_argument&){}
  // Invalid direct API gains must fail before corrupting the active PID state.
  for (const Gains bad : {Gains{-1, 0, 0, 0.5},
                          Gains{0.5, -1, 0, 0.5},
                          Gains{0.5, 0, -1, 0.5},
                          Gains{0.5, 0, 0, 0},
                          Gains{0.5, 0, 0, 1.1},
                          Gains{std::numeric_limits<double>::quiet_NaN(), 0, 0, 0.5},
                          Gains{0.5, std::numeric_limits<double>::infinity(), 0, 0.5}}) {
    bool rejected = false;
    try { pid.set_gains(bad); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
    rejected = false;
    try { Pid invalid(bad); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
  }
  assert(std::isfinite(pid.calculate(0.2, 0.05)));
  pid.set_gains({0.8,0.04,0.2,0.6});
  for(int i=0;i<100;++i)assert(std::abs(pid.calculate(1,.05))<=.6);
  Fuzzy fuzzy;assert(fuzzy.calculate(.3,.05)>0);
  fuzzy.reset();assert(fuzzy.calculate(-.3,.05)<0);
  fuzzy.set(.5,.3,.75);
  assert(std::abs(fuzzy.calculate(1,.05))<=.75);
  StateMachine s;
  Limits lim;
  assert(s.update(safe(),1,false,lim).first==State::Disarmed);
  Observation stale=safe();stale.hazard_age=10;
  assert(s.update(stale,1,true,lim).first==State::SensorStop);
  auto hazard=safe(true,"GREEN");hazard.hazard=true;
  assert(s.update(hazard,1,true,lim).first==State::ObstacleStop);
  for(int i=0;i<15;++i) {
    auto result=s.update(safe(true,"UNKNOWN"),3+i*.1,true,lim);
    assert(result.first==State::WaitSignal&&result.second==0);
  }
  assert(s.update(safe(true,"GREEN"),5,true,lim).second==0);
  assert(s.update(safe(true,"GREEN"),5.7,true,lim).first==State::Crossing);
  assert(s.update(safe(false,"GREEN"),6.3,true,lim).first==State::Follow);
  assert(s.update(safe(),7,false,lim).first==State::Disarmed);
  assert(s.update(safe(),8,true,lim).first==State::Follow);
  StateMachine timeout;
  timeout.update(safe(true,"GREEN"),10,true,lim);
  timeout.update(safe(true,"GREEN"),10.8,true,lim);
  assert(timeout.update(safe(true,"GREEN"),14,true,lim).second==0);
  assert(timeout.update(safe(true,"GREEN"),15,true,lim).second==0);
  timeout.update(safe(),16,false,lim);
  assert(timeout.update(safe(),17,true,lim).first==State::Follow);
  for (double bad_age : {-0.01, std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::infinity()}) {
    StateMachine guard;
    auto o = safe(); o.lane_age = bad_age;
    assert(guard.update(o, 20, true, lim).first == State::SensorStop);
    assert(guard.update(safe(), 21, true, lim).first == State::Follow);
    guard.update(safe(), 22, false, lim);
    assert(guard.update(safe(), 23, true, lim).first == State::Follow);
    o = safe(true, "GREEN"); o.signal_age = bad_age;
    assert(StateMachine{}.update(o, 20, true, lim).first == State::SensorStop);
  }
  StateMachine clock_guard;
  clock_guard.update(safe(true, "GREEN"), 30, true, lim);
  assert(clock_guard.update(safe(true, "GREEN"), 29, true, lim).first == State::SensorStop);
  StateMachine crossing_dropout;
  crossing_dropout.update(safe(true, "GREEN"), 40, true, lim);
  assert(crossing_dropout.update(safe(true, "GREEN"), 40.7, true, lim).first == State::Crossing);
  auto invalid_crossing = safe(true, "GREEN");
  invalid_crossing.hazard_age = std::numeric_limits<double>::quiet_NaN();
  assert(crossing_dropout.update(invalid_crossing, 40.8, true, lim).first == State::SensorStop);
  assert(crossing_dropout.update(safe(true, "GREEN"), 40.9, true, lim).second == 0);
  crossing_dropout.update(safe(), 41, false, lim);
  assert(crossing_dropout.update(safe(), 41.1, true, lim).first == State::Follow);
  StateMachine global_clock;
  assert(global_clock.update(safe(), 50, true, lim).first == State::Follow);
  assert(global_clock.update(safe(), 49.9, true, lim).first == State::SensorStop);
  assert(global_clock.update(safe(), 51, true, lim).second == 0);
  assert(global_clock.update(safe(), 52, false, lim).first == State::Disarmed);
  assert(global_clock.update(safe(), 53, true, lim).first == State::Follow);
  StateMachine crossing_hazard;
  crossing_hazard.update(safe(true, "GREEN"), 60, true, lim);
  assert(crossing_hazard.update(safe(true, "GREEN"), 60.7, true, lim).first == State::Crossing);
  auto blocked_crossing = safe(true, "GREEN");
  blocked_crossing.hazard = true;
  assert(crossing_hazard.update(blocked_crossing, 60.8, true, lim).second == 0);
  assert(crossing_hazard.update(safe(true, "GREEN"), 60.9, true, lim).second == 0);
  crossing_hazard.update(safe(), 61, false, lim);
  assert(crossing_hazard.update(safe(), 61.1, true, lim).first == State::Follow);
  // A rollback coincident with invalid sensor data must latch the stop.
  StateMachine masked_rollback;
  assert(masked_rollback.update(safe(), 70, true, lim).first == State::Follow);
  auto invalid_rollback = safe();
  invalid_rollback.lane_age = std::numeric_limits<double>::infinity();
  assert(masked_rollback.update(invalid_rollback, 69, true, lim).first == State::SensorStop);
  assert(masked_rollback.update(safe(), 71, true, lim).first == State::SensorStop);
  assert(masked_rollback.update(safe(), 72, false, lim).first == State::Disarmed);
  assert(masked_rollback.update(safe(), 73, true, lim).first == State::Follow);
  // Once an intersection is latched, disappearing intersection detection
  // must not bypass the traffic-light freshness watchdog.
  StateMachine lost_intersection;
  assert(lost_intersection.update(safe(true, "UNKNOWN"), 80, true, lim).first == State::WaitSignal);
  auto lost_signal = safe(false, "GREEN");
  lost_signal.signal_age = 9.0;
  assert(lost_intersection.update(lost_signal, 80.1, true, lim).first == State::SensorStop);
  assert(lost_intersection.update(safe(false, "GREEN"), 80.2, true, lim).first == State::WaitSignal);
  assert(lost_intersection.update(safe(false, "GREEN"), 80.9, true, lim).first == State::Crossing);
  auto stale_during_crossing = safe(false, "GREEN");
  stale_during_crossing.signal_age = 9.0;
  assert(lost_intersection.update(stale_during_crossing, 81.0, true, lim).first == State::SensorStop);
  assert(lost_intersection.update(safe(), 81.1, true, lim).first == State::SensorStop);
  assert(lost_intersection.update(safe(), 81.2, false, lim).first == State::Disarmed);
  assert(lost_intersection.update(safe(), 81.3, true, lim).first == State::Follow);
  // An obstacle must invalidate accumulated green dwell time.
  StateMachine interrupted_green;
  assert(interrupted_green.update(safe(true, "GREEN"), 90, true, lim).first == State::WaitSignal);
  auto interrupted = safe(true, "GREEN"); interrupted.hazard = true;
  assert(interrupted_green.update(interrupted, 90.5, true, lim).first == State::ObstacleStop);
  assert(interrupted_green.update(safe(true, "GREEN"), 90.7, true, lim).first == State::WaitSignal);
  assert(interrupted_green.update(safe(true, "GREEN"), 91.0, true, lim).first == State::WaitSignal);
  assert(interrupted_green.update(safe(true, "GREEN"), 91.4, true, lim).first == State::Crossing);
  // A red signal detected during crossing must not permit automatic resume.
  auto red_crossing = safe(true, "RED");
  assert(interrupted_green.update(red_crossing, 91.5, true, lim).first == State::SensorStop);
  assert(interrupted_green.update(safe(true, "GREEN"), 92, true, lim).second == 0);
  assert(interrupted_green.update(safe(), 92.1, false, lim).first == State::Disarmed);
  assert(interrupted_green.update(safe(), 92.2, true, lim).first == State::Follow);
  // Simultaneous lane, obstacle and signal loss while crossing must latch a stop.
  StateMachine simultaneous_dropout;
  assert(simultaneous_dropout.update(safe(true, "GREEN"), 100, true, lim).first == State::WaitSignal);
  assert(simultaneous_dropout.update(safe(true, "GREEN"), 100.7, true, lim).first == State::Crossing);
  auto missing_sensors = safe(false, "UNKNOWN");
  missing_sensors.lane_age = 5.0;
  missing_sensors.hazard_age = 5.0;
  missing_sensors.signal_age = 5.0;
  assert(simultaneous_dropout.update(missing_sensors, 100.8, true, lim).first == State::SensorStop);
  assert(simultaneous_dropout.update(safe(true, "GREEN"), 101, true, lim).first == State::SensorStop);
  assert(simultaneous_dropout.update(safe(), 101.1, false, lim).first == State::Disarmed);
  assert(simultaneous_dropout.update(safe(), 101.2, true, lim).first == State::Follow);
  // Range input must reject malformed metadata, not just the measurement.
  assert(rcauto::range_covers_stop_zone(0.1, 2.0, 0.5));
  assert(rcauto::range_covers_stop_zone(0.1, 0.5, 0.5));
  assert(!rcauto::range_covers_stop_zone(0.1, 0.3, 0.5));
  assert(!rcauto::range_covers_stop_zone(0.1, 2.0, 0.0));
  assert(!rcauto::range_covers_stop_zone(0.1, std::numeric_limits<double>::quiet_NaN(), 0.5));
  assert(!rcauto::range_covers_stop_zone(0.1, 2.0, std::numeric_limits<double>::infinity()));
  assert(!rcauto::range_covers_stop_zone(0.6, 2.0, 0.5));
  assert(!rcauto::range_covers_stop_zone(-0.1, 2.0, 0.5));
  assert(!rcauto::range_covers_stop_zone(std::numeric_limits<double>::quiet_NaN(), 2.0, 0.5));
  assert(rcauto::range_sample_healthy(0.0, 0.4, true, 0.1, 2.0, 0.5));
  assert(rcauto::range_sample_healthy(0.4, 0.4, true, 0.1, 2.0, 0.5));
  assert(!rcauto::range_sample_healthy(0.401, 0.4, true, 0.1, 2.0, 0.5));
  assert(!rcauto::range_sample_healthy(-0.01, 0.4, true, 0.1, 2.0, 0.5));
  assert(!rcauto::range_sample_healthy(std::numeric_limits<double>::quiet_NaN(), 0.4, true, 0.1, 2.0, 0.5));
  assert(!rcauto::range_sample_healthy(0.1, 0.4, false, 0.1, 2.0, 0.5));
  assert(!rcauto::range_sample_healthy(0.1, 0.4, true, 0.6, 2.0, 0.5));
  assert(!rcauto::range_sample_healthy(0.1, 0.0, true, 0.1, 2.0, 0.5));
  assert(rcauto::valid_range_sample(0.5, 0.1, 2.0));
  assert(!rcauto::valid_range_sample(0.5, 2.0, 0.1));
  assert(!rcauto::valid_range_sample(0.5, -1.0, 2.0));
  assert(!rcauto::valid_range_sample(0.5, 0.1, std::numeric_limits<double>::infinity()));
  assert(!rcauto::valid_range_sample(std::numeric_limits<double>::quiet_NaN(), 0.1, 2.0));
  assert(!rcauto::valid_range_sample(0.5, std::numeric_limits<double>::quiet_NaN(), 2.0));
  rcauto::Fuzzy invalid_fuzzy;
  bool rejected_nan_limit = false;
  try { invalid_fuzzy.set(0.65, 0.25, std::numeric_limits<double>::quiet_NaN()); }
  catch (const std::invalid_argument&) { rejected_nan_limit = true; }
  assert(rejected_nan_limit);
  StateMachine invalid_clock;
  assert(invalid_clock.update(safe(), -1, true, lim).first == State::SensorStop);
  assert(invalid_clock.update(safe(), 74, true, lim).first == State::SensorStop);
  assert(invalid_clock.update(safe(), 75, false, lim).first == State::Disarmed);
  StateMachine nan_clock;
  assert(nan_clock.update(safe(), std::numeric_limits<double>::quiet_NaN(), true, lim).first == State::SensorStop);
  // Camera preview must trigger braking before the vehicle enters a curve.
  const double straight_risk = camera_risk(0.01, 0.02, 0.0, 0.95, 1.0);
  const double curve_risk = camera_risk(0.10, 0.70, 0.75, 0.95, 1.0);
  assert(curve_risk > straight_risk);
  VisualSpeedPid speed_pid;
  speed_pid.set_gains(Gains{1.9,0.0,0.06,1.0});
  const double fast = speed_pid.calculate(straight_risk, 0.05);
  speed_pid.reset();
  const double slow = speed_pid.calculate(curve_risk, 0.05);
  assert(fast > slow && fast <= 1.0 && slow >= 0.25);
  FuzzyThrottle fuzzy_speed;
  fuzzy_speed.set_aggressiveness(1.0);
  const double fuzzy_fast = fuzzy_speed.calculate(straight_risk, 0.05);
  fuzzy_speed.reset();
  const double fuzzy_slow = fuzzy_speed.calculate(curve_risk, 0.05);
  assert(fuzzy_fast > fuzzy_slow);
  assert(fuzzy_slow >= 0.25 && fuzzy_fast <= 1.0);
  bool bad_throttle_rejected = false;
  try { fuzzy_speed.set_aggressiveness(-1); }
  catch (const std::invalid_argument&) { bad_throttle_rejected = true; }
  assert(bad_throttle_rejected);
  std::cout<<"PASS: PID, fuzzy, NaN, stale watchdog, obstacle, signal, FSM, E-stop, crossing timeout, invalid ages and clock rollback\n";
}
