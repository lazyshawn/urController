#ifndef UR_INTERFACE_H
#define UR_INTERFACE_H
#include "ur5e_driver.h"

#include <deque>
#include <sys/mman.h>  // 内存管理
#include "common.h"
#include "thread_pool.h"

// 主线程优先级
#define MY_PRIORITY (49)

class urConfig {
public:
  struct Data {
    Data () {
      path.status = PATH_INIT;
    }

    double time;
    PATH path;
    THETA curTheta, refTheta;
    Mat4d tranMat;
  };

  // 复制 Config
  Data get_data(void);
  // 更新 Config
  void update(Data* Data_);

private:
  Data data;
  std::mutex config_mutex;
  std::condition_variable config_cond;
};

class PathQueue {
private:
  std::deque<PATH> data;
  mutable std::mutex path_mutex;
  std::condition_variable path_cond;

public:
  PathQueue(){};
  void push(PATH path);
  void wait_and_pop(PATH& path);
  bool try_pop(PATH& path);
  bool try_pop(PATH& path, double time, ARRAY orig);
  bool empty() const;
  int size();
  // for debug
  void wait();
  void notify_one();
};

/*************************************************************************
 * 机械臂伺服程序
*************************************************************************/
void robot_thread_function(void);

void servo_function(UrDriver* ur);

void calc_ref_joint(urConfig::Data& urConfigData);

/*************************************************************************
 * 轨迹规划
*************************************************************************/
void add_displacement(PATH& path, NUMBUF& inputData);
void add_joint_destination(PATH& path, NUMBUF& inputData);
void add_cartesion_destination(PATH& path, NUMBUF& inputData, THETA curTheta);
void robot_go_home(THETA curTheta);
void pivot_about_points(TRIARR& state, TRIARR command, double time);
void instant_command(THETA curTheta);
void go_to_pose(Mat4d tran, THETA curTheta, double time);
bool wait_for_path_clear(void);
#endif

