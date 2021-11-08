
#include "../include/trajectory.h"

/* 
 * @func  : calc_ref_joint
 * @brief : 计算轨迹插补点
 * @param : svo_共享变量
 * @return: 下一插补点处的 关节角/位姿
 */
void calc_ref_joint(SVO& svo) {
  // 当前路径执行的时间
  double offsetTime = svo.time - svo.path.beginTime;
  double freq = svo.path.freq;
  double delQ = 8*Deg2Rad;

  // 角度伺服
  if (svo.path.angleServo) {
    ARRAY orig = svo.path.orig;
    ARRAY goal = svo.path.goal;
    int interpMode = svo.path.interpMode;
    svo.path.complete = joint_interpolation(
        offsetTime, freq, interpMode, orig, goal, svo.refTheta);
  } else {
  // 速度伺服
    svo.path.complete = velocity_interpolation(
        offsetTime, freq, svo.curTheta, svo.refTheta, svo.path.velocity);
  } // if {} else {}

  // 角度限位
  for (int i=0; i<6; ++i) {
    if (svo.refTheta[i] - svo.curTheta[i] > delQ) {
      svo.refTheta[i] = svo.curTheta[i] + delQ;
    } else if (svo.refTheta[i] - svo.curTheta[i] < -delQ) {
      svo.refTheta[i] = svo.curTheta[i] - delQ;
    }
  }
} // calc_ref_joint()

/* 
 * @func  : velocity_interpolation
 * @brief : 利用 Jacobian 矩阵进行速度伺服控制 
 * @param : offsetTime_运动时间; freq_插补频率; curTheta_当前关节角; 
 *          refTheta_下一时刻关节角; velocity_速度命令
 * @return: 修改_下一时刻的关节角，返回_轨迹完成标志位
 * @remark: 用实际角度计算时，会因为角度变化太小，机械臂不动
 */
bool velocity_interpolation(double offsetTime, double freq, THETA curTheta, 
    THETA& refTheta, ARRAY velocity) {
  Vec6d velo, dq;
  double time = freq * (offsetTime+SERVO_TIME);

  for (int i=0; i<6; ++i) velo[i] = velocity[i];
  // 用参考角度计算各关节角的正余弦值
  calcJnt(refTheta);
  // 计算 Jacobian 矩阵
  Mat6d ijcb = ur_jacobian().inverse();
  dq = ijcb*velo;
  // 当前运动时间占总时间的比例大于1，即超过预定时间，插值点不变，返回已完成
  if (time >= 1) {
    // 路径停止前，给一定时间让机械臂停止
    for (int i=0; i<6; ++i) refTheta[i] = refTheta[i];
    if (time >= 1.1) {
      return true;
    }
  }
  for (int i=0; i<6; ++i) refTheta[i] += dq(i);
  return false;
} // bool velocity_interpolation()

/* 
 * @func  : joint_interpolation
 * @brief : 计算关节空间的位置插补
 * @param : offsetTime_运动时间; freq_插补频率; interpMode_插补模式; 
 *          orig_插值起点; goal_插值终点; &refVal_当前时刻插值点位置
 * @return: 修改_下一时刻的关节角，返回_轨迹完成标志位
 */
bool joint_interpolation(double offsetTime, double freq, int interpMode, 
    ARRAY orig, ARRAY goal, ARRAY& refVal) {
  // 当前运动时间占总时间的比例 | 已完成的路径占全路径的比例
  double time = freq * (offsetTime+SERVO_TIME);

  // 超过预定时间，插值点设为终点值，返回已完成
  if (time >= 1) {
    refVal = goal;
    return true;
  }
  // 按插补模式在关节空间进行轨迹插补
  switch (interpMode) {
  // sin 関数による軌道補間 | sin函数的轨迹插补
  case INTERP_SIN:
    for (int i=0; i<6; ++i) {
      refVal[i] = orig[i] + (goal[i]-orig[i])*time -
        (goal[i]-orig[i])*sin(2.0*M_PI*time)/(2.0*M_PI);
    }
    break;
  // 1 次関数による軌道補間 | 一次函数的轨迹插补
  case INTERP_1JI:
    for (int i=0; i<6; ++i) {
      refVal[i] = orig[i] + (goal[i] - orig[i]) * time;
    }
    break;
  // 3 次関数による軌道補間 | 3次函数的轨迹插补
  case INTERP_3JI:
    for (int i=0; i<6; ++i) {
      refVal[i] = orig[i] + (goal[i] - orig[i]) * time*time*(3.0-2*time);
    }
    break;
  // 5 次関数による軌道補間 | 5次函数的轨迹插补
  case INTERP_5JI:
    for (int i=0; i<6; ++i) {
      refVal[i] = orig[i] +
        (goal[i] - orig[i]) * time*time*time*(10.0+time*(-15.0+6.0*time));
    }
    break;
  // ステップ関数による軌道補間 | 阶跃函数的轨迹插补
  case INTERP_STEP: for (int i=0; i<6; ++i) {refVal[i] = goal[i];} break;
  default: printf("\n==>> Error! Unknow interpolate mode!\n"); break;
  } // switch(interpMode)
  return false;
} // bool joint_interpolation()

// 3 次関数による速度軌道補間 | 3次函数的速度插补
double Calc3JiTrajeVelo(double orig, double goal, double freq, double time) {
  double ref = 0.0;
  double time_n = freq * time;

  if (time_n <= 1)
    ref = (goal - orig) * freq * 6.0 * time_n * (1 - time_n);
  return ref;
}

// 5 次関数による速度軌道補間 | 5次函数的速度插补
double Calc5JiTrajeVelo(double orig, double goal, double freq, double time) {
  double ref = 0.0;
  double time_n = freq * time;

  if (time_n <= 1)
    ref = freq * (goal - orig) * 30.0 * time_n * time_n * (time_n - 1) *
          (time_n - 1);
  return ref;
}

// sin 関数による速度軌道補間 | sin函数的速度插补
double CalcSinTrajeVelo(double orig, double goal, double freq, double time) {
  double ref = 0.0;
  double time_n = freq * time;

  if (time_n <= 1)
    ref = freq * (goal - orig) * (1 - cos(2.0 * M_PI * time_n));
  return ref;
}

