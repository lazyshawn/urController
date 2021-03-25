/* *********************************************************
 * ==>> 伺服主程序
 * *********************************************************/

#include "../include/adrobot_control.h"
#include "../include/adrobot_etc.h"
#include "../include/adrobot_interface.h"
#include "../include/adrobot_system.h"
/*shared memory header*/
#include <errno.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <vector>
/*shared memory header*/
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
extern void *shm_addr;
using namespace std;

/* 伺服主程序 */
void servo_function(UrDriver *ur) {
  int i, ret;
  double curtime;
  float P[6];
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

  JACOBIAN *jcbn;
  JACOBIAN jcb1;
  jcbn = &jcb1;
  std::vector<double> jnt_angle(6);
  std::vector<double> jnt_angleD(6);
  std::vector<double> com_jnt_angle(6);

  memcpy(coordinate, shm_addr, 72);

  // Get the current time
  curtime = GetCurrentTime();
  SvoReadFromServo(&servoP);
  servoP.Time = curtime;

  for (i = 0; i < 18; i++) {
    servoP.markpos.t[i] = coordinate[i];
    Curpos(i, 0) = coordinate[i];
  }

// obtain the current state of ur
#ifndef ROBOT_OFFLINE
  jnt_angle = ur->rt_interface_->robot_state_->getQActual();
  jnt_angleD = ur->rt_interface_->robot_state_->getQdActual();
#else
  // TEST CODE
  jnt_angle[0] = 0.0;
  jnt_angle[1] = 0.0;
  jnt_angle[2] = 0.0;
  jnt_angle[3] = 0.0;
  jnt_angle[4] = 0.0;
  jnt_angle[5] = 0.0;
#endif

  // copy the UR state
  for (i = 0; i < 6; i++) {
    servoP.CurTheta.t[i] = jnt_angle[i];
    servoP.jnk.c[i] = cos(jnt_angle[i]);
    servoP.jnk.s[i] = sin(jnt_angle[i]);
    // ensure the robot do not move after setup
    servoP.RefTheta.t[i] = jnt_angle[i]; 
    servoP.CurDTheta.t[i] = jnt_angleD[i];
  }

  /* update the kinematics calculation*/
  // 返回末端夹持点的位置坐标
  pos = ur_kinematics(&servoP.jnk, hnd_ori);

  for (i = 0; i < 6; i++) P[i] = pos(i + 1, 1);
  // 计算tcp到末端标记点的距离
  GripperToPoint = ComputeDistance(coordinate, 18, P, 6);
  // 计算点在执行器坐标系下的半径
  Radius = ComputeRadiusVector(coordinate, 18, P, 6);

  for (i = 0; i < 6; i++)
    servoP.distogripper.t[i] = GripperToPoint(0, i);
  // cout<<computeGrippersToDeformableObjectJacobian(N)<<endl;
  for (i = 0; i < 6; i++)
    servoP.CurPos.t[i] = pos(i + 1, 1); // The start position of the tcp.
  // ************* should be noticed *************
  if (servoP.NewPathFlag == ON) {
    if (servoP.PathtailFlag == OFF) {
      ret = GetTrjBuff(&path);
      if (ret == 0) {
        servoP.Path = path;
        SetStartTime(curtime);
      } else {
        servoP.PathtailFlag = ON;
      }
      // the case  GetOffsettime > 1/servoP.path.Freq
      servoP.NewPathFlag = ON;
    }
  }
  if (servoP.NewPathFlag == ON) {
    for (i = 0; i < 6; i++)
      servoP.Path.Orig[i] = servoP.CurPos.t[i];
    servoP.NewPathFlag = OFF;
  }
  if (servoP.ServoFlag == ON && servoP.PosOriServoFlag == OFF)
    CalcJntRefPath(GetOffsetTime(), &servoP.Path, &servoP.RefTheta,
                   &servoP.RefDTheta);
  if (servoP.PosOriServoFlag == ON && servoP.ServoFlag == ON) {
    CalcPosRefPath(GetOffsetTime(), &servoP.Path, &servoP.RefPos);
    CalcPointpos(GetOffsetTime(), &servoP.Path, &servoP.refmarkpos);

    for (i = 0; i < 18; i++) {
      Refpos(i, 0) = servoP.refmarkpos.t[i];
    }
    for (i = 0; i < 6; i++) {
      ref_pos(i + 1, 1) = servoP.RefPos.t[i];
    }

    B0 = CalcB0(pos(4, 1), pos(5, 1), pos(6, 1));
    PosErrorTrn = ((Eye(3) | Zeros(3, 3)) || (Zeros(3, 3) | B0));
    jcb = ur_jacobian(&servoP.jnk, jcbn);
    /*shape_servo*/
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        jacobiE(i, j) = jcb(i + 1, j + 1);
      }
    }
    //              cout<<jacobiE<<endl;
    MatrixXf j_def(18, 6);
    j_def = ComputeGripperToObjectJacobian(GripperToPoint, Radius);
    //              cout<<j_def<<endl;
    MatrixXf J(18, 6);
    J = j_def * jacobiE;
    //              cout<<J<<endl;
    JacobiSVD<MatrixXf> svd(J, ComputeThinU | ComputeThinV);
    float pinvtoler = 1.e-6; // choose your tolerance wisely
    MatrixXf singularValues_inv = svd.singularValues();
    for (long i = 0; i < J.cols(); ++i) {
      if (singularValues_inv(i) > pinvtoler)
        singularValues_inv(i) = 1.0 / singularValues_inv(i);
      else
        singularValues_inv(i) = 0;
    }
    MatrixXf pinvmat = svd.matrixV() * singularValues_inv.asDiagonal() *
                       svd.matrixU().transpose();
    //              cout<<pinvmat<<endl;
    errorpos = Refpos - Curpos;
    //           cout<<errorpos<<endl;
    for (i = 0; i < 18; i++) {
      servoP.errpos.t[i] = errorpos(i, 0);
      //               cout<<servoP.refmarkpos.t[i];
    }
    DeltaJnt = pinvmat * errorpos;
    //           cout<<DeltaJnt<<endl;
    for (i = 0; i < 6; i++)
      servoP.deltaTheta.t[i] = DeltaJnt(i, 0);
    //           cout<<DeltaJnt;
    for (i = 0; i < 6; i++) { //限位，每8ms关节运动量不超过1.4度
      if ((DeltaJnt(i, 0) * Rad2Deg) > 1.4)
        DeltaJnt(i, 0) = 0.024434609;
      else if ((DeltaJnt(i, 0) * Rad2Deg) < -1.4)
        DeltaJnt(i, 0) = -0.024434609;
    }
    for (i = 0; i < 6; i++)
      servoP.xianweiTheta.t[i] = DeltaJnt(i, 0);
    for (i = 0; i < 6; i++)
      DeltaJntG(i, 0) = DeltaJnt(i, 0) * 0.025;
    for (i = 0; i < 6; i++)
      servoP.RefTheta.t[i] = servoP.CurTheta.t[i] + DeltaJntG(i, 0);

    /*shape_servo*/
  }
  for (i = 0; i < 6; i++)
    com_jnt_angle[i] = servoP.RefTheta.t[i];
#ifndef ROBOT_OFFLINE
  ur->servoj(com_jnt_angle, 1);
#endif
  // interact with the GUI
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
    // for(int i=0;i<6;i++){
    //     servoP.CurTheta.t[i]=servoP.RefTheta.t[i];
    //     servoP.jnk.c[i]=cos(servoP.CurTheta.t[i]);
    //     servoP.jnk.s[i]=sin(servoP.CurTheta.t[i]);
    // }
  }
  SvoWriteFromServo(&servoP);
  cnt++;
}

