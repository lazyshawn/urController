/* *********************************************************
 * ==>> 伺服主程序
 * *********************************************************/

#include "../include/thread_pool.h"

// 线程结束标志
extern struct shm_interface shm_servo_inter;
// Defined from dataExchange.cpp
extern Config config;
extern Path_queue path_queue;


/* 
 * @func  : servo_function
 * @brief : 伺服主程序_对路径进行插值，并发送运动指令
 * @param : UrDriver*_机械臂驱动类的指针
 * @return: void
 */
void servo_function(UrDriver* ur) {
  // Copy global SVO
  SVO svoLocal = config.getCopy();
  MATRIX_D hnd_ori = Zeros(3, 3);
  MATRIX_D pos = MatD61(0, 0, 0, 0, 0, 0);
  std::vector<double> jnt_angle(6);

  // Get the current time
  svoLocal.time = GetCurrentTime();

  /* Obtain the current state of ur */
#ifndef ROBOT_OFFLINE
  jnt_angle = ur->rt_interface_->robot_state_->getQActual();
  // copy the UR state
  for (int i = 0; i < 6; i++) svoLocal.curTheta[i] = jnt_angle[i];
#else
  for (int i=0; i<6; ++i) {
    jnt_angle[i] = svoLocal.curTheta[i] = svoLocal.refTheta[i];
  }
#endif

  /* Update the kinematics calculation*/
  // 计算各关节角的正余弦值
  calcJnt(svoLocal.curTheta);
  // 返回末端夹持点的位置坐标
  pos = ur_kinematics(hnd_ori);
  // Current position of the end_link
  for (int i = 0; i < 6; i++) {svoLocal.curPos[i] = pos(i+1, 1);}

  /* Pop path info */
  if (svoLocal.path.complete) {
    // 当前路径运行完成，且队列中没有新的路径
    if (path_queue.empty()) {
      config.update(&svoLocal);
      return;
    }
    // 尝试弹出路径
    path_queue.try_pop(svoLocal.path, svoLocal.time, svoLocal.curTheta);
    // 弹出时初始化参考角度
    for (int i=0; i<6; ++i) svoLocal.refTheta[i] = svoLocal.curTheta[i];
  }
  // path_queue.wait();  // For debug: wake up by path_queue.notify_one()

  /* 计算轨迹插补点(关节角目标值) */
  calc_ref_joint(svoLocal);

#ifndef ROBOT_OFFLINE
  // 机械臂执行运动指令
  for (int i=0; i<6; ++i) jnt_angle[i] = svoLocal.refTheta[i];
  ur->servoj(jnt_angle, 1);
#endif
  /* 记录待保存的数据 */
  ExpDataSave(&svoLocal);

  // Update global SVO
  config.update(&svoLocal);
}


/* 
 * @func  : display
 * @brief : 显示线程_以一定间隔显示机械臂运行时的状态信息
 * @param : void
 * @return: void
 */
void display(void) {
  SVO svoLocal;
  int printLen = 8;  // 输出数据长度 [9999.99s -999.99mm, -359.99deg]
  // 以一定间隔显示机械臂运行时的状态信息
  while (shm_servo_inter.status_control == INIT_C) {
    // delay for 25 microseconds
    usleep(25000);
    svoLocal = config.getCopy();
    // 若当前没有正在运行的路径则跳过循环
    if (svoLocal.path.complete) continue;
    // 固定输出格式，显示小数点后两位，左对齐
    std::cout << setiosflags(std::ios::fixed) << std::setprecision(2)
      << setiosflags(std::ios::right);
    // 角度伺服模式
    if (svoLocal.path.angleServo==ON) {
      std::cout << "Time:" << std::setw(printLen) << svoLocal.time
        << " | Ref[deg]: "
        << std::setw(printLen) << svoLocal.refTheta[0] * Rad2Deg
        << std::setw(printLen) << svoLocal.refTheta[1] * Rad2Deg
        << std::setw(printLen) << svoLocal.refTheta[2] * Rad2Deg
        << std::setw(printLen) << svoLocal.refTheta[3] * Rad2Deg
        << std::setw(printLen) << svoLocal.refTheta[4] * Rad2Deg
        << std::setw(printLen) << svoLocal.refTheta[5] * Rad2Deg
        << " | Cur[deg]: "
        << std::setw(printLen) << svoLocal.curTheta[0] * Rad2Deg
        << std::setw(printLen) << svoLocal.curTheta[1] * Rad2Deg
        << std::setw(printLen) << svoLocal.curTheta[2] * Rad2Deg
        << std::setw(printLen) << svoLocal.curTheta[3] * Rad2Deg
        << std::setw(printLen) << svoLocal.curTheta[4] * Rad2Deg
        << std::setw(printLen) << svoLocal.curTheta[5] * Rad2Deg
        << std::endl;
    } else {
    // 当前有路径在运行, 且为速度伺服模式
      std::cout << "Time:" << std::setw(printLen) << svoLocal.time
        << "\nCurrent position of the end_link\n"
        << "   X[mm]   Y[mm]   Z[mm]  Alpha[deg]   Beta[deg]   Gama[deg]\n"
        << std::setw(printLen) << svoLocal.curPos[0]
        << std::setw(printLen) << svoLocal.curPos[1]
        << std::setw(printLen) << svoLocal.curPos[2] << "    "
        << std::setw(printLen) << svoLocal.curPos[3] * Rad2Deg << "    "
        << std::setw(printLen) << svoLocal.curPos[4] * Rad2Deg << "    "
        << std::setw(printLen) << svoLocal.curPos[5] * Rad2Deg
        << "\nInstant velocity of the end_link\n"
        << "   X[mm]   Y[mm]   Z[mm]  Alpha[deg]   Beta[deg]   Gama[deg]\n"
        << std::setw(printLen) << svoLocal.path.velocity[0]
        << std::setw(printLen) << svoLocal.path.velocity[1]
        << std::setw(printLen) << svoLocal.path.velocity[2] << "    "
        << std::setw(printLen) << svoLocal.path.velocity[3] * Rad2Deg << "    "
        << std::setw(printLen) << svoLocal.path.velocity[4] * Rad2Deg << "    "
        << std::setw(printLen) << svoLocal.path.velocity[5] * Rad2Deg
        << std::endl;
    }
  } // while (shm_servo_inter.status_control == INIT_C)
  std::cout << "Program end: interface_function." << std::endl;
} // void display(void)


/* 
 * @func  : interface
 * @brief : GUI界面_处理人机交互
 * @param : void
 * @return: void
 */
void interface(void) {
  SVO svoLocal = config.getCopy();
  PATH pathLocal;
  char command;

  display_menu();
  while (shm_servo_inter.status_control == INIT_C) {
    // Wait command
    std::cout << "Please press any key" << std::endl;
    std::cin >> command;
    svoLocal = config.getCopy();
    // Run command
    switch (command) {
    // hand reference setting
    case 'c': case 'C': add_hand_path(pathLocal); break;
    // joint reference setting
    case 'j': case 'J': add_joint_path(pathLocal); break;
    // Start
    case 's': case 'S': path_queue.push(pathLocal); break;
    // Show the information of robot
    case 'i': case 'I': display_current_information(svoLocal); break;
    // Exit
    case 'e': case 'E': shm_servo_inter.status_control = EXIT_C; break;
    // Show menu
    case 'm': case 'M': display_menu(); break;
    // Next shot. Set for debug.
    case 'n': case 'N': path_queue.notify_one(); break;
    // Invalid command
    default: std::cout << "==>> Unknow command." << std::endl; break;
    }
    config.update(&svoLocal);
  } // while (shm_servo_inter.status_control == INIT_C)
  std::cout << "Program end: display_function." << std::endl;
} // void interface(void)

