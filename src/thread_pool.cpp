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

  // while (threadManager.process != THREAD_EXIT) {
  //   urConfigData = urconfig.get_data();
  //   forceData = force.get_copy();
  //   objStateData = objState.get_data();
  //
  //   if(objStateData.flag != true) continue;
  //   break;
  // }
  // imshow("Display color Image", objStateData.img);
  // cv::waitKey(2000);
  // cv::destroyAllWindows();
  std::cout << "Thread terminated: master_thread" << std::endl;
}


/*************************************************************************
 * @func  : servo_function
 * @brief : 伺服主程序_对路径进行插值，并发送运动指令
 * @param : UrDriver*_机械臂驱动类的指针
 * @return: void
*************************************************************************/
void servo_function(UrDriver* ur, RobotiQ* rbtQ) {
  // Copy global urConfig::Data
  urConfig::Data urConfigData = urconfig.get_data();
  Mat3d oriMat;
  Vec6d pose;
  std::vector<double> jnt_angle(6);

  // Get the current time
  urConfigData.time = GetCurrentTime();

  /* Obtain the current state of ur */
#ifndef ROBOT_OFFLINE
  jnt_angle = ur->rt_interface_->robot_state_->getQActual();
  // copy the UR state
  for (int i = 0; i < 6; i++) urConfigData.curTheta[i] = jnt_angle[i];
#else
  for (int i=0; i<6; ++i) {
    jnt_angle[i] = urConfigData.curTheta[i] = urConfigData.refTheta[i];
  }
#endif

  /* Update the kinematics calculation*/
  // 计算各关节角的正余弦值
  calcJnt(urConfigData.curTheta);
  // 返回末端夹持点的位置坐标
  pose = ur_kinematics(oriMat);
  // Current position of the end_link
  for (int i = 0; i < 6; i++) {urConfigData.curPos[i] = pose(i);}

  /* Pop path info */
  if (urConfigData.path.complete) {
    // 当前路径运行完成，且队列中没有新的路径
    if (pathQueue.empty()) {
      urconfig.update(&urConfigData);
      return;
    }
    // 尝试弹出路径
    pathQueue.try_pop(urConfigData.path, urConfigData.time, urConfigData.curTheta);
    // 弹出时初始化参考角度
    for (int i=0; i<6; ++i) urConfigData.refTheta[i] = urConfigData.curTheta[i];
  }
  // pathQueue.wait();  // For debug: wake up by pathQueue.notify_one()

  /* 计算轨迹插补点(关节角目标值) */
  calc_ref_joint(urConfigData);

#ifndef ROBOT_OFFLINE
  // 机械臂执行运动指令
  for (int i=0; i<6; ++i) jnt_angle[i] = urConfigData.refTheta[i];
  ur->servoj(jnt_angle, 1);
  if (urConfigData.path.fingerPos>=0) rbtQ->open_to_cmd(urConfigData.path.fingerPos);
#endif
  /* 记录待保存的数据 */
  ExpDataSave(&urConfigData);

  // Update global urConfig::Data
  urconfig.update(&urConfigData);
}


