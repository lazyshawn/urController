
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
  ARRAY orig = svo.path.orig;
  ARRAY goal = svo.path.goal;
  int interpMode = svo.path.interpMode;

  // 角度伺服
  if (svo.path.angleServo) {
    svo.path.complete = joint_interpolation(
        offsetTime, freq, interpMode, orig, goal, svo.refTheta);
  } else {
  // 速度伺服
    svo.path.complete = velocity_interpolation(
        offsetTime, freq, svo.curTheta, svo.refTheta, svo.path.velocity);
  } // if {} else {}
} // calc_ref_joint()

/* 
 * @func  : velocity_interpolation
 * @brief : 利用 Jacobian 矩阵进行速度伺服控制 
 * @param : offsetTime_运动时间; freq_插补频率; curTheta_当前关节角; 
 *          refTheta_下一时刻关节角; velocity_速度命令
 * @return: 修改_下一时刻的关节角，返回_轨迹完成标志位
 */
bool velocity_interpolation(double offsetTime, double freq, THETA curTheta, 
    THETA& refTheta, ARRAY velocity) {
  // 当前运动时间占总时间的比例大于1，即超过预定时间，插值点不变，返回已完成
  if (freq * offsetTime > 1) return true;

  JACOBIAN* jcbCompact = new JACOBIAN;
  MATRIX_D jcb = Zeros(6,6);
  MATRIX_D velo = Zeros(6,1), dq = Zeros(6,1);

  for (int i=0; i<6; ++i) {
    velo(i+1,1) = velocity[i];
  }
  // 计算 Jacobian 矩阵
  jcb = ur_jacobian(jcbCompact);
  MATRIX_D ijcb(6,6, (double *)jcbCompact->invJcb);
  dq = ijcb*velo;
  // 角度限位
  for (int i=0; i<6; ++i) {
    dq(i+1,1) = (dq(i+1,1)<1.4*Deg2Rad) ? dq(i+1,1) : 0;
    refTheta[i] = curTheta[i] + dq(i+1,1);
  }
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
  double time = freq * offsetTime;

  // 超过预定时间，插值点设为终点值，返回已完成
  if (time > 1) {
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

