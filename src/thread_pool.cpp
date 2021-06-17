/* *********************************************************
 * ==>> 伺服主程序
 * *********************************************************/

#include "../include/thread_pool.h"

int cnt = 0;
float coordinate[18];

// 线程结束标志
extern struct shm_interface shm_servo_inter;
// Defined from dataExchange.cpp
extern Config config;

/* 伺服主程序 */
void servo_function(UrDriver *ur) {
  int i, ret;
  double curtime;
  float P[6];          // 末端夹持点(TCP)的位置坐标
  SVO svoLocal;
  SVO_SAVE svoSave;
  PATH path;

  // Matrix_D: 元素为double类型的矩阵
  MATRIX_D hnd_ori = Zeros(3, 3);
  MATRIX_D pos = MatD61(0, 0, 0, 0, 0, 0);
  MATRIX_D rotMatrix = MatD61(0, 0, 0, 0, 0, 0);
  MATRIX_D err_pos = MatD61(0, 0, 0, 0, 0, 0);
  MATRIX_D ref_pos = MatD61(0, 0, 0, 0, 0, 0);
  MATRIX_D DJnt = MatD61(0, 0, 0, 0, 0, 0);
  MATRIX_D B0 = Zeros(3, 3);
  MATRIX_D PosErrorTrn = Zeros(6, 6);
  MATRIX_D jcb = Zeros(6, 6);

  JACOBIAN jcb1;
  JACOBIAN *jcbn = &jcb1;
  std::vector<double> jnt_angle(6);
  std::vector<double> jnt_angleD(6);
  std::vector<double> com_jnt_angle(6);

  /* Copy global SVO */
  svoLocal = config.getCopy();

  // Get the current time
  curtime = GetCurrentTime();
  svoLocal.Time = curtime;

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
    // 计算各关节角的正余弦值
    // ensure the robot do not move after setup
    servoP.RefTheta.t[i] = jnt_angle[i]; 
    servoP.CurDTheta.t[i] = jnt_angleD[i];
  }
#else
  for (int i=0; i<6; ++i) {
    svoLocal.CurTheta.t[i] = svoLocal.RefTheta.t[i];
    jnt_angle[i] = svoLocal.CurTheta.t[i];
  }
#endif

  /* Update the kinematics calculation*/
  calcJnt(jnt_angle);
  // 返回末端夹持点的位置坐标
  pos = ur_kinematics(hnd_ori);
  for (i = 0; i < 6; i++) P[i] = pos(i+1, 1);

  for (i = 0; i < 6; i++) {
    // The start position of the tcp during next path
    svoLocal.CurPos.t[i] = pos(i+1, 1);
  }

  /* Pop path info */
  // ************* should be noticed *************
  if (svoLocal.NewPathFlag == ON) { // 新路径
    if (svoLocal.PathtailFlag == OFF) {  // 路径未结束
      ret = GetTrjBuff(&path);
      if (ret == 0) {
        svoLocal.Path = path;  // 取出新的路劲信息
        SetStartTime(curtime);
      } else {
        svoLocal.PathtailFlag = ON;
      }
      // the case GetOffsettime > 1/servoP.path.Freq
      svoLocal.NewPathFlag = ON;
    }
  } // if (servoP.NewPathFlag == ON)

  // Set path.orig
  if (svoLocal.NewPathFlag == ON) {
    for (i = 0; i < 6; i++) {
      svoLocal.Path.Orig[i] = svoLocal.CurTheta.t[i];
    }
    svoLocal.NewPathFlag = OFF;
  }
  /* 计算轨迹插补点(关节角目标值) */
  // 角度的伺服控制
  if (svoLocal.ServoFlag == ON && svoLocal.PosOriServoFlag == OFF)
    CalcJntRefPath(GetOffsetTime(), &svoLocal.Path, &svoLocal.RefTheta,
                   &svoLocal.RefDTheta);
  // 末端位姿的伺服控制
  if (svoLocal.PosOriServoFlag == ON && svoLocal.ServoFlag == ON) {
    double dt = 0.01;
    double v = 0, w = 45*Deg2Rad;
    double offsetTime = GetOffsetTime();
    MATRIX_D dq = MatD61(0,0,0,0,0,0);
    if (svoLocal.Path.Freq * offsetTime <= 1) {
      jcb = ur_jacobian(jcbn);
      MATRIX_D dhnd = MatD61(v*svoLocal.Path.Freq*dt,0,0,w*svoLocal.Path.Freq*dt,0,0);
      MATRIX_D ijcb(6,6, (double *)jcbn->invJcb);
      dq = ijcb*dhnd;
    }

    double delQ[6];
    delQ <<= dq;
    for (int i=0; i<6; ++i) {
      delQ[i] = (delQ[i]<1.4*Deg2Rad) ? delQ[i] : 0;
      svoLocal.RefTheta.t[i] = svoLocal.CurTheta.t[i] + delQ[i];
    }
  } // if (servoP.PosOriServoFlag == ON && servoP.ServoFlag == ON)

  for (i = 0; i < 6; i++) com_jnt_angle[i] = svoLocal.RefTheta.t[i];
#ifndef ROBOT_OFFLINE
  // 机械臂执行运动指令
  ur->servoj(com_jnt_angle, 1);
#endif
  /* 记录待保存的数据 */
  if (svoLocal.ServoFlag == ON) {
    if (cnt % EXP_DATA_INTERVAL == 0) {
      svoSave.Time = GetCurrentTime();
      svoSave.CurTheta = svoLocal.CurTheta;
      svoSave.CurDTheta = svoLocal.CurDTheta;
      svoSave.CurPos = svoLocal.CurPos;
      svoSave.RefPos = svoLocal.RefPos;
      svoSave.RefTheta = svoLocal.RefTheta;
      svoSave.Gain = svoLocal.Gain;
      svoSave.Path = svoLocal.Path;
      svoSave.markpos = svoLocal.markpos;
      svoSave.distogripper = svoLocal.distogripper;
      svoSave.deltaTheta = svoLocal.deltaTheta;
      svoSave.errpos = svoLocal.errpos;
      svoSave.refmarkpos = svoLocal.refmarkpos;
      svoSave.xianweiTheta = svoLocal.xianweiTheta;
      ExpDataSave(&svoSave);
    }
  }
  /* Interact with the GUI */
  // SvoWriteFromServo(&servoP);
  config.update(&svoLocal);
  cnt++;
}

void display(void) {
  int status_interface, i;
  int loop_counter = 0;
  SVO svoLocal;
  double time;
  double jnt_angle[6];

  do {
    time = GetCurrentTime();
    svoLocal = config.getCopy();

    if ((svoLocal.PosOriServoFlag == ON) && (svoLocal.ServoFlag == ON) &&
        (GetOffsetTime() < (1.0 / (double)svoLocal.Path.Freq))) {
      printf("\n");
      printf("TIME:%0.1f\n", time);
      printf("Current position of hand:\n");
      printf("X[m]\tY[m]\tZ[m]\tAlpha[deg]\tBeta[deg]\tGama[Deg]\n");
      printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
             svoLocal.CurPos.t[0], svoLocal.CurPos.t[1],
             svoLocal.CurPos.t[2], svoLocal.CurPos.t[3] * Rad2Deg,
             svoLocal.CurPos.t[4] * Rad2Deg,
             svoLocal.CurPos.t[5] * Rad2Deg);
      printf("Reference position of hand:\n");
      printf("X[m]\tY[m]\tZ[m]\tAlpha[deg]\tBeta[deg]\tGama[Deg]\n");
      printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
             svoLocal.RefPos.t[0], svoLocal.RefPos.t[1],
             svoLocal.RefPos.t[2], svoLocal.RefPos.t[3] * Rad2Deg,
             svoLocal.RefPos.t[4] * Rad2Deg,
             svoLocal.RefPos.t[5] * Rad2Deg);
    }
    if ((svoLocal.PosOriServoFlag == OFF) && (svoLocal.ServoFlag == ON) &&
        (GetOffsetTime() < (1.0 / (double)svoLocal.Path.Freq))) {
      printf("\n");
      printf("T: %.2f Ref[deg]: %.2f %.2f %.2f %.2f %.2f %.2f Jnt[deg]:%.2f "
             "%.2f %.2f %.2f %.2f %.2f\r", time,
             svoLocal.RefTheta.t[0] * Rad2Deg, svoLocal.RefTheta.t[1] * Rad2Deg,
             svoLocal.RefTheta.t[2] * Rad2Deg, svoLocal.RefTheta.t[3] * Rad2Deg,
             svoLocal.RefTheta.t[4] * Rad2Deg, svoLocal.RefTheta.t[5] * Rad2Deg, 
             svoLocal.CurTheta.t[0] * Rad2Deg, svoLocal.CurTheta.t[1] * Rad2Deg,
             svoLocal.CurTheta.t[2] * Rad2Deg, svoLocal.CurTheta.t[3] * Rad2Deg,
             svoLocal.CurTheta.t[4] * Rad2Deg,svoLocal.CurTheta.t[5] * Rad2Deg);
    }
    usleep(25000); // delay for 25 microseconds
  } while (shm_servo_inter.status_control != EXIT_C);
  printf("\nEnd of Display Function\n");
} // void *display_function(void *param)

void interface(void) {
  SVO svoLocal;
  int interface_counter = 0;
  int end = 1;
  int command;

  MATRIX_D A = MatD31(1, 2, 3), B = MatD31(4, 5, 6);
  MATRIX_D testp = MatD61(0, 0, 0, 0, 0, 0);
  MATRIX_D testOri = Zeros(3, 3);
  double c;
  std::vector<double> cur_jnt_ang(6);
  std::vector<double> test_com_ang(6);
  printf("Executing interface function\n");

  DisplayMenu();

  do {
    // Wait command
    printf("Please hit any key\n");

    command = getchar();
    switch (command) {
    case 'c':
    case 'C':
      //求出的结果是末端的位置Matrix61
      testp = ur_kinematics(testOri);
      printf("Current position of hand:\n");
      printf("POS::%.2f[mm],%.2f[mm],%.2f[mm],%.2f[deg], %.2f[deg], %.2f[deg]\n",
             testp(1, 1), testp(2, 1), testp(3, 1), testp(4, 1) * Rad2Deg,
             testp(5, 1) * Rad2Deg, testp(6, 1) * Rad2Deg);
      break;
    case 'g': // gain setting
    case 'G':
      // ChangeGain(&interface_svo.Gain);
      break;
    case 'p': // joint reference setting
    case 'P':
      printf("-----------------Now you are in JntSvoMode------------------\n");
      printf("Set the path frequency,path mode and goal joint position\n");
      svoLocal.PosOriServoFlag = OFF;
      ChangePathData(&svoLocal.Path);
      break;
    case 's':
    case 'S':
      printf("Start\n");
      if (svoLocal.PosOriServoFlag == ON)
        SetPosOriSvo(&svoLocal);
      else
        SetJntSvo(&svoLocal);
      break;
    case 'i':
    case 'I': // Show the information of robot
      DisplayCurrentInformation();
      break;
    case 'd':
    case 'D': // Demo function
      // JntDemoFunction(&interface_svo);
      printf("---------------Now you are in PosOriServoMode!---------------\n");
      PosOriServo(&svoLocal.PosOriServoFlag);
      ChangeHandData(&svoLocal.Path);
      break;
    case 'e':
    case 'E':
      end = 0;
      shm_servo_inter.status_control = EXIT_C;
      break;
    default:
      DisplayMenu();
      DisplayCurrentInformation();
      break;
    }
    interface_counter++;
  } while (end);
  printf("End of Experiment\n");
}

