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
// 调用其他线程的函数
#include "sensor_interface.h"
#include "camera_interface.h"
#include "ur5e_interface.h"
#include "gripper_interface.h"


/* **************** 线程管理 **************** */
// 只在最顶层使用: master/user_interface
// 线程状态
#define THREAD_INIT 0
#define THREAD_EXIT 255
#define THREAD_SETUP 1
#define THREAD_INIT_GRASP 2
#define THREAD_PIVOT 3
// 节点状态
#define OFF 0
#define ON 1
#define IDLE 2
struct ThreadManager {
  ThreadManager() {
    memset(this, 0, sizeof(ThreadManager));
  }
  struct Device {
    uint8_t camera, sensor, robot;
  }device;
  uint8_t process;
};

void master_thread_function(void);
void init_grasp(void);
void pick_and_place(Mat4d tranMat);

#endif

