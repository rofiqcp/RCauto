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
  Pid pid;
  assert(pid.calculate(.3,.05)>0);
  pid.reset();assert(pid.calculate(-.3,.05)<0);
  try{pid.calculate(std::numeric_limits<double>::quiet_NaN(),.05);return 9;}
  catch(const std::invalid_argument&){}
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
  assert(s.update(safe(false),6.3,true,lim).first==State::Follow);
  s.emergency_stop();assert(s.update(safe(),7,true,lim).second==0);
  assert(s.update(safe(),8,true,lim).first==State::Estop);
  s.reset_disarmed();
  assert(s.update(safe(),9,false,lim).first==State::Disarmed);
  StateMachine timeout;
  timeout.update(safe(true,"GREEN"),10,true,lim);
  timeout.update(safe(true,"GREEN"),10.8,true,lim);
  assert(timeout.update(safe(true,"GREEN"),14,true,lim).second==0);
  std::cout<<"PASS: PID, fuzzy, NaN, stale watchdog, obstacle, signal, FSM, E-stop, crossing timeout\n";
}
