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


/* **************** 线程管理 **************** */
// 只在最顶层使用: master/user_interface
// 线程状态
#define THREAD_INIT 0
#define THREAD_EXIT 255
// 节点状态
#define OFF 0
#define ON 1
#define IDLE 2
struct ThreadManager {
  ThreadManager() {
    memset(&device, 0, sizeof(Device));
    process = THREAD_INIT;
    // device.camera = false;
    // device.sensor = false;
    // device.robot  = false;
  }
  struct Device {
    bool camera, sensor, robot;
  }device;
  int process;
};

void master_thread_function(void);
void servo_function(UrDriver* ur);

#endif

