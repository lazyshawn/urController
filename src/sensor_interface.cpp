#include "../include/sensor_interface.h"

Force force;
ForceSensor forceSensor;
extern ThreadManager threadManager;

/*************************************************************************
* @class: Force;
*************************************************************************/
ForceData Force::get_copy(void) {
  std::scoped_lock guard(force_mutex);
  return forceData;
}

void Force::update(ForceData* ForceData_){
  std::scoped_lock guard(force_mutex);
  forceData = *ForceData_;
}

/*************************************************************************
* @func: sensor_thread_function;
*************************************************************************/
void sensor_thread_function(void) {
  ForceData forceData;

  forceSensor.init_dev();
  forceData.rcvStart = true;

  while (threadManager.process != THREAD_EXIT) {
    forceSensor.read_force(forceData.forceMat, forceData.timestamp);
    force.update(&forceData);
  }
  std::cout << "Thread terminated: sensor_thread" << std::endl;
}

