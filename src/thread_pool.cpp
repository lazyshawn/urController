/* *********************************************************
 * ==>> 伺服主程序
 * *********************************************************/

#include "../include/thread_pool.h"

/*Eigen matrix*/
MatrixXf Refpos(18, 1);
MatrixXf Curpos(18, 1);
MatrixXf errorpos(18, 1);
MatrixXf jacobiE(6, 6);
MatrixXf DeltaJnt(6, 1);
MatrixXf DeltaJntG(6, 1);
MatrixXf GripperToPoint(1, 6);
MatrixXf Radius(18, 1);
/*Eigen matrix*/
int cnt = 0;
float coordinate[18];

// 线程结束标志
extern struct shm_interface shm_servo_inter;
// 全局的共享变量
extern SVO pSVO;

/* 伺服主程序 */
void servo_function(UrDriver *ur) {
  int i, ret;
  double curtime;
  float P[6];          // 末端夹持点(TCP)的位置坐标
  SVO servoP;
  SVO_SAVE servoSave;
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
  SvoReadFromServo(&servoP);

  // Get the current time
  curtime = GetCurrentTime();
  servoP.Time = curtime;

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
#endif
  // copy the UR state
  for (i = 0; i < 6; i++) {
    servoP.CurTheta.t[i] = jnt_angle[i];
    // 计算各关节角的正余弦值
    calcJnt(jnt_angle);
    // ensure the robot do not move after setup
    servoP.RefTheta.t[i] = jnt_angle[i]; 
    servoP.CurDTheta.t[i] = jnt_angleD[i];
  }

  /* Update the kinematics calculation*/
  // 返回末端夹持点的位置坐标
  pos = ur_kinematics(hnd_ori);
  for (i = 0; i < 6; i++) P[i] = pos(i+1, 1);

  // 计算tcp到末端标记点的距离
  GripperToPoint = ComputeDistance(coordinate, 18, P, 6);
  // 计算点在执行器坐标系下的半径
  Radius = ComputeRadiusVector(coordinate, 18, P, 6);

  for (i = 0; i < 6; i++) {
    servoP.distogripper.t[i] = GripperToPoint(0, i);
    // The start position of the tcp during next path
    servoP.CurPos.t[i] = pos(i+1, 1);
  }

  /* Pop path info */
  // ************* should be noticed *************
  if (servoP.NewPathFlag == ON) { // 新路径
    if (servoP.PathtailFlag == OFF) {  // 路径未结束
      ret = GetTrjBuff(&path);
      if (ret == 0) {
        servoP.Path = path;  // 取出新的路劲信息
        SetStartTime(curtime);
      } else {
        servoP.PathtailFlag = ON;
      }
      // the case GetOffsettime > 1/servoP.path.Freq
      servoP.NewPathFlag = ON;
    }
  } // if (servoP.NewPathFlag == ON)

  // Set path.orig
  if (servoP.NewPathFlag == ON) {
    for (i = 0; i < 6; i++) {
      servoP.Path.Orig[i] = servoP.CurTheta.t[i];
    }
    servoP.NewPathFlag = OFF;
  }
  /* 计算轨迹插补点(关节角目标值) */
  // 角度的伺服控制
  if (servoP.ServoFlag == ON && servoP.PosOriServoFlag == OFF)
    CalcJntRefPath(GetOffsetTime(), &servoP.Path, &servoP.RefTheta,
                   &servoP.RefDTheta);
  // 末端位姿的伺服控制
  if (servoP.PosOriServoFlag == ON && servoP.ServoFlag == ON) {
    double offsetTime = GetOffsetTime();
    MATRIX_D dq = MatD61(0,0,0,0,0,0);
    if (servoP.Path.Freq * offsetTime <= 1) {
      jcb = ur_jacobian(jcbn);
      MATRIX_D dhnd = MatD61(1,0,0,0,0,0);
      MATRIX_D ijcb(6,6, (double *)jcbn->invJcb);
      dq = ijcb*dhnd;
    }

    double delQ[6];
    delQ <<= dq;
    for (int i=0; i<6; ++i) {
      servoP.RefTheta.t[i] = servoP.CurTheta.t[i] + delQ[i];
    }
  } // if (servoP.PosOriServoFlag == ON && servoP.ServoFlag == ON)

  for (i = 0; i < 6; i++) com_jnt_angle[i] = servoP.RefTheta.t[i];
#ifndef ROBOT_OFFLINE
  // 机械臂执行运动指令
  ur->servoj(com_jnt_angle, 1);
#endif
  /* 记录待保存的数据 */
  if (servoP.ServoFlag == ON) {
    if (cnt % EXP_DATA_INTERVAL == 0) {
      servoSave.Time = GetCurrentTime();
      servoSave.CurTheta = servoP.CurTheta;
      servoSave.CurDTheta = servoP.CurDTheta;
      servoSave.CurPos = servoP.CurPos;
      servoSave.RefPos = servoP.RefPos;
      servoSave.RefTheta = servoP.RefTheta;
      servoSave.Gain = servoP.Gain;
      servoSave.Path = servoP.Path;
      servoSave.markpos = servoP.markpos;
      servoSave.distogripper = servoP.distogripper;
      servoSave.deltaTheta = servoP.deltaTheta;
      servoSave.errpos = servoP.errpos;
      servoSave.refmarkpos = servoP.refmarkpos;
      servoSave.xianweiTheta = servoP.xianweiTheta;
      ExpDataSave(&servoSave);
    }
  }
  /* Interact with the GUI */
  SvoWriteFromServo(&servoP);
  cnt++;
}

void *display_function(void *param) {
  int status_interface, i;
  int loop_counter = 0;
  SVO display_svo;
  double time;
  double jnt_angle[6];

  do {
    time = GetCurrentTime();

    SvoReadFromServo(&display_svo);
    if ((display_svo.PosOriServoFlag == ON) && (display_svo.ServoFlag == ON) &&
        (GetOffsetTime() < (1.0 / (double)display_svo.Path.Freq))) {
      printf("\n");
      printf("TIME:%0.1f\n", time);
      printf("Current position of hand:\n");
      printf("X[m]\tY[m]\tZ[m]\tAlpha[deg]\tBeta[deg]\tGama[Deg]\n");
      printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
             display_svo.CurPos.t[0], display_svo.CurPos.t[1],
             display_svo.CurPos.t[2], display_svo.CurPos.t[3] * Rad2Deg,
             display_svo.CurPos.t[4] * Rad2Deg,
             display_svo.CurPos.t[5] * Rad2Deg);
      printf("Reference position of hand:\n");
      printf("X[m]\tY[m]\tZ[m]\tAlpha[deg]\tBeta[deg]\tGama[Deg]\n");
      printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
             display_svo.RefPos.t[0], display_svo.RefPos.t[1],
             display_svo.RefPos.t[2], display_svo.RefPos.t[3] * Rad2Deg,
             display_svo.RefPos.t[4] * Rad2Deg,
             display_svo.RefPos.t[5] * Rad2Deg);
    }
    if ((display_svo.PosOriServoFlag == OFF) && (display_svo.ServoFlag == ON) &&
        (GetOffsetTime() < (1.0 / (double)display_svo.Path.Freq))) {
      printf("\n");
      printf("T: %.2f Ref[deg]: %.2f %.2f %.2f %.2f %.2f %.2f Jnt[deg]:%.2f "
             "%.2f %.2f %.2f %.2f %.2f\r", time,
             display_svo.RefTheta.t[0] * Rad2Deg,
             display_svo.RefTheta.t[1] * Rad2Deg,
             display_svo.RefTheta.t[2] * Rad2Deg,
             display_svo.RefTheta.t[3] * Rad2Deg,
             display_svo.RefTheta.t[4] * Rad2Deg,
             display_svo.RefTheta.t[5] * Rad2Deg, 
             display_svo.CurTheta.t[0] * Rad2Deg,
             display_svo.CurTheta.t[1] * Rad2Deg,
             display_svo.CurTheta.t[2] * Rad2Deg,
             display_svo.CurTheta.t[3] * Rad2Deg,
             display_svo.CurTheta.t[4] * Rad2Deg,
             display_svo.CurTheta.t[5] * Rad2Deg);
    }
    usleep(25000); // delay for 25 microseconds
  } while (shm_servo_inter.status_control != EXIT_C);
  printf("\nEnd of Display Function\n");
  return ((void *)0);
} // void *display_function(void *param)

void *interface_function(void *param) {
  SVO interface_svo;
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
      interface_svo.PosOriServoFlag = OFF;
      ChangePathData(&interface_svo.Path);
      break;
    case 's':
    case 'S':
      printf("Start\n");
      if (interface_svo.PosOriServoFlag == ON)
        SetPosOriSvo(&interface_svo);
      else
        SetJntSvo(&interface_svo);
      break;
    case 'i':
    case 'I': // Show the information of robot
      DisplayCurrentInformation();
      break;
    case 'd':
    case 'D': // Demo function
      // JntDemoFunction(&interface_svo);
      printf("---------------Now you are in PosOriServoMode!---------------\n");
      PosOriServo(&interface_svo.PosOriServoFlag);
      ChangeHandData(&interface_svo.Path);
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
  sleep(5);
  printf("End of Experiment\n");
  return ((void *)0);
}

