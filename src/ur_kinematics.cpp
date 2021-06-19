
#include "../include/ur_kinematics.h"

// 各关节角的正余弦值
double c1, s1, c2, s2, c3, s3, c4, s4, c5, s5, c6, s6, c23, s23, c234, s234;

/* 
 * @brief: 更新各关节角的正余弦值;
 * @param: 关节角向量;
 * @return: ;
 */
int calcJnt(std::vector<double> q) {
  if (q.size() != 6) return -1;
  c1 = cos(q[0]);  s1 = sin(q[0]);
  c2 = cos(q[1]);  s2 = sin(q[1]);
  c3 = cos(q[2]);  s3 = sin(q[2]);
  c4 = cos(q[3]);  s4 = sin(q[3]);
  c5 = cos(q[4]);  s5 = sin(q[4]);
  c6 = cos(q[5]);  s6 = sin(q[5]);
  c23 = cos(q[1]+q[2]); s23 = sin(q[1]+q[2]);
  c234 = cos(q[1]+q[2]+q[3]); s234 = sin(q[1]+q[2]+q[3]);
  return 0;
}

/* 
 * @brief: M-DH求解机械臂运动学
 * @param: MATRIX_D rx_h - 旋转矩阵;
 * @return: 末端位姿向量(6x1)
 */
MATRIX_D ur_kinematics(MATRIX_D &ori_hnd) {
  double hori, vert;
  // 末端位置矩阵(3x1)
  MATRIX_D p_hnd = MatD31(0, 0, 0);
  // 轴角 & 欧拉角
  MATRIX_D AxisAngle = MatD31(0.0, 0.0, 0.0);
  MATRIX_D EulerAngle = MatD31(0.0, 0.0, 0.0);

  hori = DH_D4 + DH_D6*c5;
  vert = DH_A3*c2 + DH_A4*c23 - DH_D5*s234 + DH_D6*c234*s5;
  // 位置向量
  p_hnd(1,1) = -s1*hori + c1*vert;
  p_hnd(2,1) =  c1*hori + s1*vert;
  p_hnd(3,1) = DH_D1 - DH_A3*s2 - DH_A4*s23 - DH_D5*c234 - DH_D6*s234*s5;
  // 旋转矩阵
  ori_hnd(1,1) = c6*(s1*s5 + c1*c234*c5) - s234*c1*s6;
  ori_hnd(1,2) = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6;
  ori_hnd(1,3) = c234*c1*s5 - c5*s1;
  ori_hnd(2,1) = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6;
  ori_hnd(2,2) = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1;
  ori_hnd(2,3) = c1*c5 + c234*s1*s5;
  ori_hnd(3,1) = -c234*s6 - s234*c5*c6;
  ori_hnd(3,2) = s234*c5*s6 - c234*c6;
  ori_hnd(3,3) = -s234*s5;

  // AxisAngle = RotMat2AxisAngle(ori_hnd);
  EulerAngle = RotMat2EulerAngle(ori_hnd);
  return ((p_hnd || EulerAngle));
}

/* 
 * @brief: 计算从关节空间到笛卡尔空间的雅克比矩阵，及其逆矩阵、转置矩阵
 * @param: jnt; jcbn-雅克比矩阵结构体指针;
 * @return: 雅克比矩阵
 */
MATRIX_D ur_jacobian(JACOBIAN *jcbn) {
  double mat[6][6];
  memset(mat, 0, sizeof(double) * 36);
  MATRIX_D jcb(6, 6), ijcb(6, 6), tjcb(6, 6);

  double foo11, foo12, foo2, foo3, foo4;
  foo11 = DH_A3*c2 + DH_A4*c23 - DH_D5*s234 + DH_D6*c234*s5;
  foo12 = DH_D4 + DH_D6*c5;
  foo2 = DH_A3*s2 + DH_A4*s23 + DH_D5*c234 + DH_D6*s234*s5; 
  foo3 = DH_A4*s23 + DH_D5*c234 + DH_D6*s234*s5;
  foo4 = DH_D5*c234 + DH_D6*s234*s5;

  /* ****** 平动分量 ****** */
  // dr/dq5
  jcb(1,5) = DH_D6*(s1*s5+c1*c234*c5);
  jcb(2,5) = DH_D6*(-c1*s5+s1*c234*c5);
  jcb(3,5) = -DH_D6*s234*c5;
  // dr/dq4
  jcb(1,4) = -foo4*c1;
  jcb(2,4) = -foo4*s1;
  jcb(3,4) = DH_D5*s234 - DH_D6*c234*s5; 
  // dr/dq3
  jcb(1,3) = -foo3*c1;
  jcb(2,3) = -foo3*s1;
  jcb(3,3) = -DH_A4*c23 + jcb(4,3); 
  // dr/dq2
  jcb(1,2) = -c1*foo2;
  jcb(2,2) = -s1*foo2;
  jcb(3,2) = -DH_A3*c2 + jcb(3,3); 
  // dr/dq1
  jcb(1,1) = -s1*foo11 - c1*foo12;
  jcb(2,1) = -s1*foo12 + c1*foo11;
  /* ****** 转动分量 ****** */
  jcb(6,1) = 1;
  jcb(4,2) = jcb(4,3) = jcb(4,4) = -s1;
  jcb(5,2) = jcb(5,3) = jcb(5,4) = c1;
  jcb(4,5) = -c1*s234;
  jcb(5,5) = -s1*s234;
  jcb(6,5) = -c234;               
  jcb(4,6) = -s1*c5 + c1*c234*s5; 
  jcb(5,6) = c1*c5 + s1*c234*s5;  
  jcb(6,6) = -s234*s5;            

  // Jacobian
  (double *)jcbn->Jcb <<= jcb;
  // Transform of Jacobian
  tjcb = jcb.t();
  (double *)jcbn->TrnsJcb <<= tjcb;
  // Inverse of Jacobian
  if (calcInverse(jcb, ijcb) == 0)
    (double *)jcbn->invJcb <<= ijcb;
  return jcb;
}

int calcInverse(MATRIX_D &jcb, MATRIX_D &ijcb) {
  MATRIX_D mat(6, 6);
  mat = jcb;
  if (mat.snglr())
    return (-1);
  else {
    ijcb = mat.inverse();
  }
  return (0);
}

MATRIX_D RPY2RotMat(double alpha, double beta, double gamma) {
  MATRIX_D rotMat = Zeros(3, 3);

  rotMat = Rot(2, cos(gamma), sin(gamma)) * Rot(1, cos(beta), sin(beta)) *
           Rot(0, cos(alpha), sin(alpha));

  return rotMat;
}

MATRIX_D RotMat2EulerAngle(MATRIX_D rotMat) {
  MATRIX_D EulerAngle = Zeros(3, 1);
  double alpha, beta, gamma, r11, r21, r31, r32, r33;
  r31 = rotMat(3,1);
  r11 = rotMat(1,1);
  r21 = rotMat(2,1);
  r32 = rotMat(3,2);
  r33 = rotMat(3,3);

  beta = atan2(-r31, sqrt(r11*r11+r21*r21));
  alpha = atan2(r21/cos(beta), r11/cos(beta));
  gamma = atan2(r32/cos(beta), r33/cos(beta));

  EulerAngle = MatD31(alpha, beta, gamma);
  return EulerAngle;
}


MATRIX_D AxisAnglePi(MATRIX_D BMat) {
  double b11, b22, b33, b12, b13, b23;
  b11 = BMat(1, 1);
  b22 = BMat(2, 2);
  b33 = BMat(3, 3);
  b12 = BMat(1, 2);
  b13 = BMat(1, 3);
  b23 = BMat(2, 3);

  if ((b12 > 0) & (b13 > 0) & (b23 > 0)) {
    return (MatD31(sqrt(b11), sqrt(b22), sqrt(b33)));
  } else if (fabs(b12 * b13 * b23) > 0.0001) { // no one is zero
    if (b12 > 0) {
      return (MatD31(sqrt(b11), sqrt(b22), -sqrt(b33)));
    } else if (b13 > 0) {
      return (MatD31(sqrt(b11), -sqrt(b22), sqrt(b33)));
    } else {
      return (MatD31(-sqrt(b11), sqrt(b22), sqrt(b33)));
    }
  } else if ((fabs(b12) > 0.0001) || (fabs(b13) > 0.0001) ||
             (fabs(b23) > 0.0001)) { // one ni is zero
    if (fabs(b12) > 0.0001) {
      if (b12 > 0)
        return (MatD31(sqrt(b11), sqrt(b22), 0.0));
      else
        return (MatD31(sqrt(b11), -sqrt(b22), 0.0));
    }
    if (fabs(b13) > 0.0001) {
      if (b13 > 0)
        return (MatD31(sqrt(b11), 0.0, sqrt(b33)));
      else
        return (MatD31(sqrt(b11), 0.0, -sqrt(b33)));
    }
    if (fabs(b23) > 0.0001) {
      if (b23 > 0)
        return (MatD31(0.0, sqrt(b22), sqrt(b33)));
      else
        return (MatD31(0.0, sqrt(b22), -sqrt(b33)));
    }
  } else { // two n_i are zero
    if (fabs(b11) > 0.0001) {
      return (MatD31(sqrt(b11), 0.0, 0.0));
    } else if (fabs(b22) > 0.0001) {
      return (MatD31(0.0, sqrt(b22), 0.0));
    } else {
      return (MatD31(0.0, 0.0, sqrt(b33)));
    }
  }
}

MATRIX_D RotMat2AxisAngle(MATRIX_D rotMat) {
  double angle, cos_angle;
  MATRIX_D temp = Zeros(3, 3);
  MATRIX_D vN = MatD31(0, 0, 0);

  angle = acos(1 / 2.0 * (rotMat.tr() - 1));
  cos_angle = cos(angle);

  if (cos_angle < -0.999) { // condition that angle = pi
    MATRIX_D BMat = 0.5 * (rotMat + Eye(3));
    return (AxisAnglePi(BMat));
  } else if (cos_angle > 0.999) {
    // condition that angle = 0, axis is undetermined.
    return (Zeros(3, 1));
  } else {
    temp = 1 / (2 * sin(angle)) * (rotMat - rotMat.t());
    vN = MatD31(temp(3, 2), temp(1, 3), temp(2, 1));
    return (angle * vN);
  }
}

// MATRIX_D ur_InverseKinematics(MATRIX_D hand_p, MATRIX_D rotMat) {}

