#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H
#include "sensor_driver.h"

#include "thread_pool.h"

/* 传感器采集的共享数据 */
// 共享数据的结构体
struct ForceData {
  int forceMat[2][8][3], timestamp[2][8];
  bool rcvStart;
};
// 封装共享数据的类
class Force {
private:
  ForceData forceData;
  std::mutex force_mutex;
  std::condition_variable force_cond;

public:
  // 复制 Config
  ForceData get_copy(void);
  // 更新 Config
  void update(ForceData* ForceData_);
};

/* 传感器线程 */
void sensor_thread_function(void);

#endif

