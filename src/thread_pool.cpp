/*************************************************************************
 * ==>> 伺服主程序
*************************************************************************/
#include "../include/thread_pool.h"

// 线程管理标识
struct ThreadManager threadManager;
extern urConfig urconfig;
extern PathQueue pathQueue;
extern Force force;
extern ObjState objState;


/*************************************************************************
 * @func: master_thread_function
 * @brief: Master线程，收集线程数据，分配线程任务
*************************************************************************/
void master_thread_function(void) {
  // Global data
  urConfig::Data urConfigData;
  ForceData forceData;
  ObjState::Data objStateData;

  Eigen::Matrix<double,4,4> goalPose;

  while (threadManager.process != THREAD_EXIT) {
    urConfigData = urconfig.get_data();
    forceData = force.get_copy();
    objStateData = objState.get_data();

    if (objStateData.flag == true) {
      goalPose = urConfigData.tranMat * objStateData.markerPose;
      goalPose(2,3) = urConfigData.tranMat(2,3);  // 高度不变
      std::cout << goalPose << std::endl;
      go_to_pose(goalPose, urConfigData.curTheta);
      break;
    }
  }
  std::cout << "Thread terminated: master_thread" << std::endl;
}



