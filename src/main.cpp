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

#define MY_PRIORITY (49)
#define MAX_SAFE_STACK (8 * 1024)

// 线程结束标志
struct shm_interface shm_servo_inter;
// 线程之间的的互斥锁-全局
pthread_mutex_t servo_inter_mutex = PTHREAD_MUTEX_INITIALIZER;
// 全局的共享变量
extern SVO pSVO;
extern pthread_mutex_t mymutex;

std::condition_variable rt_ur_msg_cond, ur_msg_cond;
// UR的IP地址(静态)
std::string host_name = "10.249.181.201";

UrDriver *testUr;

int main(int argc, char** argv) {
  // 线程标识符
  pthread_t interface_thread, display_thread;
  // 系统时间
  struct timespec t;
  // Data structure to describe a process' schedulability
  struct sched_param param;
  // 一个伺服周期内的纳秒数
  int interval = 8000000; /* 8 ms*/
  int loop_flag = 0;
  shm_servo_inter.status_control = INIT_C;

  /* connect to ur robot */
  std::cout << "Connecting to " << host_name << std::endl;
  UrDriver urRobot(rt_ur_msg_cond, ur_msg_cond, host_name, 50007, 0.016, 12, 0.08, 0);

#ifndef ROBOT_OFFLINE
  urRobot.start();
  urRobot.setServojTime(0.008);
  // for test
  testUr = &urRobot;
#endif

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

  /* start interface thread*/
  if (pthread_create(&interface_thread, NULL, interface_function, NULL)) {
    perror("interface_thread create\n");
    exit(1);
  }
  /* start display thread*/
  if (pthread_create(&display_thread, NULL, display_function, NULL)) {
    perror("Display_thread create\n");
    exit(1);
  }

  /* reset save buffer */
  SaveDataReset();

#ifndef ROBOT_OFFLINE
  urRobot.uploadProg();
#endif
  while (1) {
    if (shm_servo_inter.status_control == EXIT_C) {
      printf("Program end\n");
      break;
    }

    /* wait until next shot */
    // 休眠到下个周期开始
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    /* do the stuff */
    /* reset */
    if (loop_flag == 0) {
      ResetTime();
      loop_flag = 1;
    }

    /* 伺服线程 */
    servo_function(&urRobot);

    /* calculate next shot */
    // 设置下一个线程恢复的时间
    t.tv_nsec += interval;

    // 时间进位
    while (t.tv_nsec >= NSEC_PER_SEC) {
      t.tv_nsec -= NSEC_PER_SEC;
      t.tv_sec++;
    }
  } // while (1)

  /* 结束子线程 */
  if (pthread_join(interface_thread, NULL)) {
    perror("pthread_join at interface_thread\n");
    exit(1);
  }

  if (pthread_join(display_thread, NULL)) {
    perror("pthread_join at displya_thread\n");
  }

  // 保存实验数据
  ExpDataWrite();

  // close UR
#ifndef ROBOT_OFFLINE
  urRobot.halt();
#endif
  exit(1);
} // main()

