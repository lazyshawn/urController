/*************************************************************************
 * ==>> 主函数
 * ref: https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/start
*************************************************************************/
#include "../include/common.h"
#include "../include/data_exchange.h"
#include "../include/system_time.h"
#include "../include/ur_driver.h"
#include "../include/thread_pool.h"
#include "../include/robotiq_driver.h"
#include "../include/print_status.h"
#include "../include/user_interface.h"
#include "../include/ur5e_interface.h"

// Defined from dataExchange.cpp
extern urConfig urconfig;
extern PathQueue pathQueue;
// 线程管理标识
extern struct ThreadManager threadManager;

int main(int argc, char** argv) {
  /* Lock memory | 防止内存交换 */
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    perror("mlockall failed");
    exit(-2);
  }
  /* Start background thread*/
  std::thread interface_thread(interface_thread_function);

  /* Get ready */
  SaveDataReset();
  ResetTime();

  master_thread_function();

  // 等待线程结束
  interface_thread.join();

  // 保存实验数据
  ExpDataWrite();

  return 0;
} // main()

