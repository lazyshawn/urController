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
extern CRGGripper gripper;


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

      gripper.go(40, 30);
      Arr3d cmdState;
      cmdState = {0, -19, -32*deg2rad};
      plane_screw(cmdState, 10);
      // for (int i=0; i<10; ++i) {
      //   int pos = 44+i;
      //   gripper.go(pos, 30);
      //   sleep(1);
      // }
      // sleep(3);
      gripper.go(52, 10);
      break;
    }

    if (threadManager.process == THREAD_PIVOT) {
      urConfig::Data urConfigData =  urconfig.get_data();
      int grip = 42, release = 44;
      // THETA targetTheta = {0, -75.31, 116.18, -98.89, -90, 90};  // 旋转之前 52
      // THETA targetTheta = {0, -71.88, 112.93, -92.58, -90, 90};  // 旋转 55
      // THETA targetTheta = {0, -85.8, 126.92, -117.02, -90, 90};  // 绕#1手指旋转 59
      THETA targetTheta = {0, -80.24, 119.79, -107.87, -90, 90};  // #1手指旋转、平移 59
      for (int i=0; i<6; ++i) {targetTheta[i] *= deg2rad;}

      go_to_joint(targetTheta, 8);
      // gripper.go(52,10);
      gripper.go(59,10);
      if(!wait_for_path_clear()) return;

      threadManager.process = THREAD_INIT;
    }
  }
}

void pick_and_place(Mat4d tranMat) {
  urConfig::Data urConfigData =  urconfig.get_data();
  ObjState::Data objStateData = objState.get_data();
  Mat4d goalPose, objPose;
  double peakHigh = 300, valleyHigh = 145;
  int grip = 42, release = 44;
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
  gripper.go(grip, 51);
  objPose(2,3) = peakHigh;
  go_to_pose(objPose, urConfigData.curTheta, 5);

  goalPose(2,3) = peakHigh;
  go_to_pose(goalPose, urConfigData.curTheta, 5);
  goalPose(2,3) = valleyHigh;
  go_to_pose(goalPose, urConfigData.curTheta, 5);
  // Place
  if(!wait_for_path_clear()) return;
  urConfigData = urconfig.get_data();
  gripper.go(release);
  goalPose(2,3) = 169;
  go_to_pose(goalPose, urConfigData.curTheta, 4);

  if(!wait_for_path_clear()) return;
}

