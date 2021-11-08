/* servo.h
 * @Created on: Apr 20, 2018
 * @Author    : shawn
 */
#ifndef THREAD_POOL_H
#define THREAD_POOL_H

// 主线程优先级
#define MY_PRIORITY (49)

#include <sys/mman.h>  // 内存管理
#include <thread>   // 线程管理
#include "common.h"
#include "data_exchange.h"
#include "trajectory.h"
#include "path_planning.h"
#include "system_time.h"
#include "user_interface.h"
#include "ur_kinematics.h"
#include "ur_driver.h"
#include "robotiq_driver.h"
#include "sensor_interface.h"

#define THREAD_INIT 0
#define THREAD_EXIT 255

struct ThreadManager {
  ThreadManager() {
    process = THREAD_INIT;
    device.camera = false;
    device.sensor = false;
  }
  struct Device {
    bool camera, sensor;
  }device;
  int process;
};

void master_thread_function(void);
void servo_function(UrDriver* ur, RobotiQ* rbtQ);
void display(void);
void interface(void);

#endif

