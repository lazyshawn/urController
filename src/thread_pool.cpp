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
extern RobotiQ rbtQ;


/*************************************************************************
 * @func: master_thread_function
 * @brief: Master线程，收集线程数据，分配线程任务
*************************************************************************/
void master_thread_function(void) {
  init_grasp();

  std::cout << "Thread terminated: master_thread" << std::endl;
}

void init_grasp(void) {
  // Global data
  urConfig::Data urConfigData;
  ForceData forceData;
  ObjState::Data objStateData;
  // Local Variables
  Eigen::Matrix<double,4,4> goalPose;

  while (threadManager.process != THREAD_EXIT) {
    urConfigData = urconfig.get_data();
    forceData = force.get_copy();
    objStateData = objState.get_data();

    // Pick_and_Place
    if (objStateData.flag == true) {
      goalPose = urConfigData.tranMat * objStateData.markerPose;
      pick_and_place(goalPose);
      break;
    }
  }
}

void pick_and_place(Mat4d tranMat) {
  urConfig::Data urConfigData =  urconfig.get_data();
  ObjState::Data objStateData = objState.get_data();
  Mat4d goalPose, objPose;
  double peakHigh = 300, valleyHigh = 160;
  goalPose << 1,0,0,350, 0,-1,0,DH_D4, 0,0,-1,peakHigh, 0,0,0,1;

  // Hover
  objPose = urConfigData.tranMat * objStateData.markerPose;
  objPose(2,3) = peakHigh;
  go_to_pose(objPose, urConfigData.curTheta, 5);
  // Pick
  objPose(2,3) = valleyHigh;
  go_to_pose(objPose, urConfigData.curTheta, 5);
  // 等待机械臂运动完成
  if(!wait_for_path_clear()) return;
  urConfigData = urconfig.get_data();
  rbtQ.open_to_cmd(80);
  sleep(1);
  objPose(2,3) = peakHigh;
  go_to_pose(objPose, urConfigData.curTheta, 5);
  goalPose(2,3) = peakHigh;
  go_to_pose(goalPose, urConfigData.curTheta, 5);
  goalPose(2,3) = valleyHigh;
  go_to_pose(goalPose, urConfigData.curTheta, 5);
  // Place
  if(!wait_for_path_clear()) return;
  urConfigData = urconfig.get_data();
  rbtQ.open_to_cmd(10);
  sleep(1);
  goalPose(2,3) = peakHigh;
  go_to_pose(goalPose, urConfigData.curTheta, 5);
  robot_go_home(urConfigData.curTheta);
}

