/* *********************************************************
 * ==>> 伺服主程序
 * *********************************************************/

#include "../include/thread_pool.h"

// 线程结束标志
extern struct shm_interface shm_servo_inter;
// Defined from dataExchange.cpp
extern Config config;

/* 伺服主程序 */
void servo_function(UrDriver *ur) {
  // Copy global SVO
  SVO svoLocal = config.getCopy();
  PATH path;

  // Matrix_D: 元素为double类型的矩阵
  MATRIX_D hnd_ori = Zeros(3, 3);
  MATRIX_D pos = MatD61(0, 0, 0, 0, 0, 0);
  JACOBIAN* jcbn = new JACOBIAN;

  std::vector<double> jnt_angle(6);
  std::vector<double> jnt_angleD(6);

  // Get the current time
  svoLocal.Time = GetCurrentTime();

  /* Get target information */
  // 从其他线程/进程获取目标信息
  // memcpy(coordinate, shm_addr, 72);
  // for (i = 0; i < 18; i++) {
  //   servoP.markpos.t[i] = coordinate[i];
  //   Curpos(i, 0) = coordinate[i];
  // }

  /* Obtain the current state of ur */
#ifndef ROBOT_OFFLINE
  jnt_angle = ur->rt_interface_->robot_state_->getQActual();
  jnt_angleD = ur->rt_interface_->robot_state_->getQdActual();
  // copy the UR state
  for (i = 0; i < 6; i++) {
    servoP.CurTheta.t[i] = jnt_angle[i];
    // ensure the robot do not move after setup
    servoP.RefTheta.t[i] = jnt_angle[i]; 
    servoP.CurDTheta.t[i] = jnt_angleD[i];
  }
#else
  for (int i=0; i<6; ++i) {
    jnt_angle[i] = svoLocal.CurTheta[i] = svoLocal.RefTheta[i];
  }
#endif

  /* Update the kinematics calculation*/
  // 计算各关节角的正余弦值
  calcJnt(jnt_angle);
  // 返回末端夹持点的位置坐标
  pos = ur_kinematics(hnd_ori);
  for (int i = 0; i < 6; i++) {
    // Current position of the end_link
    svoLocal.CurPos[i] = pos(i+1, 1);
  }

  /* Pop path info */
  // ************* should be noticed *************
  if (svoLocal.NewPathFlag == ON) { // 新路径
    if (svoLocal.PathtailFlag == OFF) {  // 路径未结束
      int ret = GetTrjBuff(&path);
      if (ret == 0) {
        svoLocal.Path = path;  // 取出新的路劲信息
        SetStartTime(svoLocal.Time);
      } else {
        svoLocal.PathtailFlag = ON;
        std::cout << "svoLocal.PathtailFlag = ON" << std::endl;
      }
      // the case GetOffsettime > 1/servoP.path.Freq
      svoLocal.NewPathFlag = ON;
    }
  } // if (servoP.NewPathFlag == ON)

  // Set path.orig
  if (svoLocal.NewPathFlag == ON) {
    for (int i = 0; i < 6; i++) {
      svoLocal.Path.Orig[i] = svoLocal.CurTheta[i];
    }
    svoLocal.NewPathFlag = OFF;
  }
  /* 计算轨迹插补点(关节角目标值) */
  // 角度的伺服控制
  if (svoLocal.ServoFlag == ON && svoLocal.PosOriServoFlag == OFF)
    CalcJntRefPath(GetOffsetTime(), &svoLocal.Path, svoLocal.RefTheta,
                   svoLocal.RefDTheta);
  // 末端位姿的伺服控制
  if (svoLocal.PosOriServoFlag == ON && svoLocal.ServoFlag == ON) {
    double dt = 0.01;
    double v = 0, w = 45*Deg2Rad;
    double offsetTime = GetOffsetTime();
    MATRIX_D dq = MatD61(0,0,0,0,0,0);
    if (svoLocal.Path.Freq * offsetTime <= 1) {
      MATRIX_D jcb = ur_jacobian(jcbn);
      MATRIX_D dhnd = MatD61(v*svoLocal.Path.Freq*dt,0,0,w*svoLocal.Path.Freq*dt,0,0);
      MATRIX_D ijcb(6,6, (double *)jcbn->invJcb);
      dq = ijcb*dhnd;
    }

    double delQ[6];
    delQ <<= dq;
    for (int i=0; i<6; ++i) {
      delQ[i] = (delQ[i]<1.4*Deg2Rad) ? delQ[i] : 0;
      svoLocal.RefTheta[i] = svoLocal.CurTheta[i] + delQ[i];
    }
  } // if (servoP.PosOriServoFlag == ON && servoP.ServoFlag == ON)
  // Update global SVO
  config.update(&svoLocal);

#ifndef ROBOT_OFFLINE
  // 机械臂执行运动指令
  ur->servoj(svoLocal.RefTheta.t, 1);
#endif
  /* 记录待保存的数据 */
  if (svoLocal.ServoFlag == ON) {
    ExpDataSave(&svoLocal);
  }
}

void display(void) {
  SVO svoLocal;

  while (shm_servo_inter.status_control == INIT_C) {
    svoLocal = config.getCopy();
    
    // 以一定间隔显示机械臂运行时的状态信息
    if ((svoLocal.PosOriServoFlag == ON) && (svoLocal.ServoFlag == ON) &&
        (GetOffsetTime() < (1.0 / (double)svoLocal.Path.Freq))) {
      printf("\n");
      printf("TIME:%0.1f\n", svoLocal.Time);
      printf("Current position of hand:\n");
      printf("X[m]\tY[m]\tZ[m]\tAlpha[deg]\tBeta[deg]\tGama[Deg]\n");
      printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
             svoLocal.CurPos[0], svoLocal.CurPos[1], svoLocal.CurPos[2],
             svoLocal.CurPos[3] * Rad2Deg, svoLocal.CurPos[4] * Rad2Deg,
             svoLocal.CurPos[5] * Rad2Deg);
      printf("Reference position of hand:\n");
      printf("X[m]\tY[m]\tZ[m]\tAlpha[deg]\tBeta[deg]\tGama[Deg]\n");
      printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
             svoLocal.RefPos[0], svoLocal.RefPos[1], svoLocal.RefPos[2],
             svoLocal.RefPos[3] * Rad2Deg, svoLocal.RefPos[4] * Rad2Deg,
             svoLocal.RefPos[5] * Rad2Deg);
    }
    if ((svoLocal.PosOriServoFlag == OFF) && (svoLocal.ServoFlag == ON) &&
        (GetOffsetTime() < (1.0 / (double)svoLocal.Path.Freq))) {
      printf("\n");
      printf("T: %.2f Ref[deg]: %.2f %.2f %.2f %.2f %.2f %.2f Jnt[deg]:%.2f "
             "%.2f %.2f %.2f %.2f %.2f\r", svoLocal.Time,
             svoLocal.RefTheta[0] * Rad2Deg, svoLocal.RefTheta[1] * Rad2Deg,
             svoLocal.RefTheta[2] * Rad2Deg, svoLocal.RefTheta[3] * Rad2Deg,
             svoLocal.RefTheta[4] * Rad2Deg, svoLocal.RefTheta[5] * Rad2Deg,
             svoLocal.CurTheta[0] * Rad2Deg, svoLocal.CurTheta[1] * Rad2Deg,
             svoLocal.CurTheta[2] * Rad2Deg, svoLocal.CurTheta[3] * Rad2Deg,
             svoLocal.CurTheta[4] * Rad2Deg, svoLocal.CurTheta[5] * Rad2Deg);
    }
    usleep(25000); // delay for 25 microseconds
  } // while (shm_servo_inter.status_control == INIT_C)

  std::cout << "Program end: interface_function." << std::endl;
} // void display(void)

void interface(void) {
  SVO svoLocal = config.getCopy();
  char command;
  MATRIX_D hndPos = MatD61(0, 0, 0, 0, 0, 0);
  MATRIX_D hndOri = Zeros(3, 3);

  printf("Executing interface function\n");
  DisplayMenu();

  while (shm_servo_inter.status_control == INIT_C) {
    svoLocal = config.getCopy();

    // Wait command
    printf("Please hit any key\n");
    std::cin >> command;
    // Run command
    switch (command) {
    case 'c': case 'C':
      //求出的结果是末端的位置Matrix61
      hndPos = ur_kinematics(hndOri);
      printf("Current position of hand:\n");
      printf("POS::%.2f[mm],%.2f[mm],%.2f[mm],%.2f[deg], %.2f[deg], %.2f[deg]\n",
             hndPos(1, 1), hndPos(2, 1), hndPos(3, 1), hndPos(4, 1) * Rad2Deg,
             hndPos(5, 1) * Rad2Deg, hndPos(6, 1) * Rad2Deg);
      break;
    case 'g': case 'G':// gain setting
      // ChangeGain(&interface_svo.Gain);
      break;
    case 'p': case 'P':// joint reference setting
      printf("-----------------Now you are in JntSvoMode------------------\n");
      printf("Set the path frequency,path mode and goal joint position\n");
      svoLocal.PosOriServoFlag = OFF;
      ChangePathData(&svoLocal.Path);
      break;
    case 's': case 'S':
      printf("Start\n");
      if (svoLocal.PosOriServoFlag == ON) {
        SetPosOriSvo(&svoLocal);
      } else {
        SetJntSvo(&svoLocal);
      }
      svoLocal.ServoFlag = ON;
      break;
    // Show the information of robot
    case 'i': case 'I': DisplayCurrentInformation(svoLocal); break;
    // Demo function
    case 'd': case 'D':
      printf("---------------Now you are in PosOriServoMode!---------------\n");
      // PosOriServo(&svoLocal.PosOriServoFlag);
      svoLocal.PosOriServoFlag = ON;
      ChangeHandData(&svoLocal.Path);
      break;
    case 'e': case 'E': shm_servo_inter.status_control = EXIT_C; break;
    // Show menu
    case 'm': case 'M': DisplayMenu(); break;
    default: printf("Unknow command.\n"); break;
    }
    config.update(&svoLocal);
  } // while (shm_servo_inter.status_control == INIT_C)
  std::cout << "Program end: display_function." << std::endl;
} // void interface(void)

