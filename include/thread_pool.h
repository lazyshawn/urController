/* servo.h
 * @Created on: Apr 20, 2018
 * @Author    : shawn
 */
#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <sys/mman.h>  // 内存管理
#include <thread>   // 线程管理
#include <mutex>
#include <condition_variable>
#include "system_time.h"
#include "sensor_interface.h"
#include "camera_interface.h"
#include "ur5e_interface.h"

#define THREAD_INIT 0
#define THREAD_EXIT 255

struct ThreadManager {
  ThreadManager() {
    process = THREAD_INIT;
    device.camera = false;
    device.sensor = false;
    device.robot  = false;
  }
  struct Device {
    bool camera, sensor, robot;
  }device;
  int process;
};

void master_thread_function(void);
void servo_function(UrDriver* ur);

#endif

