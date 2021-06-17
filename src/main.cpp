/* *********************************************************
 * ==>> 主函数
 * *********************************************************/

#include "../include/common.h"
#include "../include/trajectory.h"
#include "../include/data_exchange.h"
#include "../include/ur_kinematics.h"
#include "../include/system_time.h"
#include "../include/matrix.h"
#include "../include/ur_driver.h"
#include "../include/thread_pool.h"

extern Config config;
// 线程结束标志
struct shm_interface shm_servo_inter;
// 线程之间的的互斥锁-全局
pthread_mutex_t servo_inter_mutex = PTHREAD_MUTEX_INITIALIZER;

int main(int argc, char** argv) {
  // UR通信的条件变量
  std::condition_variable rt_ur_msg_cond, ur_msg_cond;
  // UR的IP地址(静态)
  std::string ur_ip = "10.249.181.201";
  // UR的通信端口
  unsigned int ur_post = 50007;
  // 系统时间
  struct timespec t;
  // Data structure to describe a process' schedulability
  struct sched_param param;
  // 一个伺服周期内的纳秒数
  int interval = 8000000; /* 8 ms*/
  shm_servo_inter.status_control = INIT_C;
  // 系统状态备份
  SVO svoLocal;

#ifndef ROBOT_OFFLINE
  /* connect to ur robot */
  std::cout << "Connecting to " << ur_ip << std::endl;
  UrDriver urRobot(rt_ur_msg_cond, ur_msg_cond, ur_ip, ur_post);
  urRobot.start();
  urRobot.setServojTime(0.008);
  sleep(1);
  urRobot.uploadProg();
#else
  UrDriver* urRobot;
  std::vector<double> jnt_angle = {0, -90, 90, -90, -90, 0};
  for (int i=0; i<6; ++i) {
    jnt_angle[i] *= Deg2Rad;
    svoLocal.RefTheta.t[i] = jnt_angle[i];
  }
#endif
  // 同步全局变量
  config.update(&svoLocal);

  /* Declare ourself as a real time task */
  // 设置线程的优先级(最大为99)
  param.sched_priority = MY_PRIORITY;
  // 设置线程的调度策略和调度参数
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    perror("sched_setscheduler failed\n");
    exit(-1);
  }

  /* Lock memory */
  // 防止内存交换
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    perror("mlockall failed");
    exit(-2);
  }

  /* 一秒后启动控制回路 */
  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_sec++;

  /* Start background thread*/
  std::thread interface_thread(interface);
  std::thread display_thread(display);

  /* Get ready */
  SaveDataReset();
  ResetTime();

  while (shm_servo_inter.status_control == INIT_C) {
    /* wait until next shot */
    // 休眠到下个周期开始
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    /* 伺服线程 */
#ifndef ROBOT_OFFLINE
    servo_function(&urRobot);
#else
    servo_function(urRobot);
#endif

    /* calculate next shot */
    // 设置下一个线程恢复的时间
    t.tv_nsec += interval;

    // 时间进位
    while (t.tv_nsec >= NSEC_PER_SEC) {
      t.tv_nsec -= NSEC_PER_SEC;
      t.tv_sec++;
    }
  } // while (1)
  printf("Program end\n");

  // 等待线程结束
  interface_thread.join();
  display_thread.join();

  // 保存实验数据
  ExpDataWrite();

  // close UR
#ifndef ROBOT_OFFLINE
  urRobot.halt();
#endif
  return 0;
} // main()

