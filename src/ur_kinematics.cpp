
#include "../include/ur_kinematics.h"

// 各关节角的正余弦值
double c1, s1, c2, s2, c3, s3, c4, s4, c5, s5, c6, s6, c23, s23, c234, s234;
// 关节角度
double q2, q3, q4;

/* 
 * @brief: 更新各关节角的正余弦值;
 * @param: 关节角向量;
 * @return: ;
 */
int calcJnt(THETA q) {
  if (q.size() != 6) return -1;
  c1 = cos(q[0]);  s1 = sin(q[0]);
  c2 = cos(q[1]);  s2 = sin(q[1]);
  c3 = cos(q[2]);  s3 = sin(q[2]);
  c4 = cos(q[3]);  s4 = sin(q[3]);
  c5 = cos(q[4]);  s5 = sin(q[4]);
  c6 = cos(q[5]);  s6 = sin(q[5]);
  c23 = cos(q[1]+q[2]); s23 = sin(q[1]+q[2]);
  c234 = cos(q[1]+q[2]+q[3]); s234 = sin(q[1]+q[2]+q[3]);
  q2 = q[1]; q3 = q[2]; q4 = q[3];
  return 0;
}

// 平面内运动的正运动学
bool plane_kinematics(std::array<double,3>& state) {
  state[0] = DH_A2*c2 + DH_A3*c23 - DH_D5*s234 - DH_D6*c234;
  state[1] = DH_D1 - DH_A2*s2 - DH_A3*s23 - DH_D5*c234 + DH_D6*s234;
  state[2] = -q2 -q3 -q4 -M_PI/2;
  return true;
}

// 平面内运动的逆解(降低为三自由度)
THETA plane_invese_kinematics(std::array<double,3>& state) {
  double x = state[0], z = state[1], q = -state[2] -M_PI/2;
  double c234 = cos(q), s234 = sin(q), c3, s3;
  double A = x + DH_D5*s234 + DH_D6*c234;
  double B = -z - DH_D5*c234 + DH_D6*s234 + DH_D1;

  // 限制工作空间后，多解问题已忽略
  THETA theta = {0, 0, 0, 0, -M_PI/2, M_PI/2};
  theta[2] = acos((A*A+B*B-DH_A2*DH_A2-DH_A3*DH_A3)/(2*DH_A2*DH_A3));
  c3 = cos(theta[2]); s3 = sin(theta[2]);
  double delta = DH_A2*DH_A2 + 2*DH_A2*DH_A3*c3 + DH_A3*DH_A3;
  double A2 = (DH_A2+DH_A3*c3)*A + DH_A3*s3*B;
  double B2 = -DH_A3*s3*A        + (DH_A2+DH_A3*c3)*B;
  theta[1] = atan2(B2/delta, A2/delta);
  theta[3] = q - theta[1] - theta[2];
  // Wrap qJoint[3] to (-pi,pi]
  if (theta[3] > M_PI) {
    theta[3] -= 2*M_PI;
  } else if (theta[3] < -M_PI) {
    theta[3] += 2*M_PI;
  }
  return theta;
}

THETA plane_jacobian() {

}

/* 
 * @brief: M-DH求解机械臂运动学
 * @param: MATRIX_D rx_h - 旋转矩阵;
 * @return: 末端位姿向量(6x1)
 */
Vec6d ur_kinematics(Mat3d& ori_hnd) {
  double hori, vert;
  // 末端位置, 轴角 , 欧拉角
  Vec3d p_hnd, axisAngle, eulerAngle;
  Vec6d pose;

  hori = DH_D4 + DH_D6*c5;
  vert = DH_A2*c2 + DH_A3*c23 - DH_D5*s234 + DH_D6*c234*s5;
  // 位置向量
  p_hnd(0) = -s1*hori + c1*vert;
  p_hnd(1) =  c1*hori + s1*vert;
  p_hnd(2) = DH_D1 - DH_A2*s2 - DH_A3*s23 - DH_D5*c234 - DH_D6*s234*s5;
  // 旋转矩阵
  ori_hnd(0,0) = c6*(s1*s5 + c1*c234*c5) - s234*c1*s6;
  ori_hnd(0,1) = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6;
  ori_hnd(0,2) = c234*c1*s5 - c5*s1;
  ori_hnd(1,0) = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6;
  ori_hnd(1,1) = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1;
  ori_hnd(1,2) = c1*c5 + c234*s1*s5;
  ori_hnd(2,0) = -c234*s6 - s234*c5*c6;
  ori_hnd(2,1) = s234*c5*s6 - c234*c6;
  ori_hnd(2,2) = -s234*s5;

  // AxisAngle = RotMat2AxisAngle(ori_hnd);
  eulerAngle = RotMat2EulerAngle(ori_hnd);
  for (int i=0; i<3; ++i) {
    pose(i) = p_hnd(i);
    pose(i+3) = eulerAngle(i);
  }
  return pose;
}

/* 
 * @brief: 计算从关节空间到笛卡尔空间的雅克比矩阵，及其逆矩阵、转置矩阵
 * @param: jnt; jcbn-雅克比矩阵结构体指针;
 * @return: 雅克比矩阵
 */
Mat6d ur_jacobian() {
  Mat6d jcb;

  double foo11, foo12, foo2, foo3, foo4;
  foo11 = DH_A2*c2 + DH_A3*c23 - DH_D5*s234 + DH_D6*c234*s5;
  foo12 = DH_D4 + DH_D6*c5;
  foo2 = DH_A2*s2 + DH_A3*s23 + DH_D5*c234 + DH_D6*s234*s5; 
  foo3 = DH_A3*s23 + DH_D5*c234 + DH_D6*s234*s5;
  foo4 = DH_D5*c234 + DH_D6*s234*s5;

  /* ****** 平动分量 ****** */
  // dr/dq5
  jcb(0,4) = DH_D6*(s1*s5+c1*c234*c5);
  jcb(1,4) = DH_D6*(-c1*s5+s1*c234*c5);
  jcb(2,4) = -DH_D6*s234*c5;
  // dr/dq4
  jcb(0,3) = -foo4*c1;
  jcb(1,3) = -foo4*s1;
  jcb(2,3) = DH_D5*s234 - DH_D6*c234*s5; 
  // dr/dq3
  jcb(0,2) = -foo3*c1;
  jcb(1,2) = -foo3*s1;
  jcb(2,2) = -DH_A3*c23 + jcb(2,3); 
  // dr/dq2
  jcb(0,1) = -c1*foo2;
  jcb(1,1) = -s1*foo2;
  jcb(2,1) = -DH_A2*c2 + jcb(2,2); 
  // dr/dq1
  jcb(0,0) = -s1*foo11 - c1*foo12;
  jcb(1,0) = -s1*foo12 + c1*foo11;
  /* ****** 转动分量 ****** */
  jcb(5,0) = 1;
  jcb(3,1) = jcb(3,2) = jcb(3,3) = -s1;
  jcb(4,1) = jcb(4,2) = jcb(4,3) = c1;
  jcb(3,4) = -c1*s234;
  jcb(4,4) = -s1*s234;
  jcb(5,4) = -c234;               
  jcb(3,5) = -s1*c5 + c1*c234*s5; 
  jcb(4,5) = c1*c5 + s1*c234*s5;  
  jcb(5,5) = -s234*s5;            

  return jcb;
}

Mat3d RPY2RotMat(double alpha, double beta, double gamma) {
  Mat3d rotMat;

  // X
  rotMat(0,0) = cos(alpha)*cos(beta);
  rotMat(1,0) = sin(alpha)*cos(beta);
  rotMat(2,0) = -sin(beta);
  // Y
  rotMat(0,1) = cos(alpha)*sin(beta)*sin(gamma) - sin(alpha)*cos(gamma);
  rotMat(1,1) = sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma);
  rotMat(2,1) = cos(beta)*sin(gamma);
  // Z
  rotMat(0,2) = cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma);
  rotMat(1,2) = sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma);
  rotMat(2,2) = cos(beta)*cos(gamma);

  return rotMat;
}

Vec3d RotMat2EulerAngle(Mat3d rotMat) {
  double alpha, beta, gamma, r11, r21, r31, r32, r33;
  r31 = rotMat(2,0);
  r11 = rotMat(0,0);
  r21 = rotMat(1,0);
  r32 = rotMat(2,1);
  r33 = rotMat(2,2);

  beta = atan2(-r31, sqrt(r11*r11+r21*r21));
  alpha = atan2(r21/cos(beta), r11/cos(beta));
  gamma = atan2(r32/cos(beta), r33/cos(beta));

  Vec3d EulerAngle(alpha, beta, gamma);
  return EulerAngle;
}


Vec3d AxisAnglePi(Mat3d BMat) {
  double b11, b22, b33, b12, b13, b23;
  b11 = BMat(0, 0);
  b22 = BMat(1, 1);
  b33 = BMat(2, 2);
  b12 = BMat(0, 1);
  b13 = BMat(0, 2);
  b23 = BMat(1, 2);

  if ((b12 > 0) & (b13 > 0) & (b23 > 0)) {
    return (Vec3d(sqrt(b11), sqrt(b22), sqrt(b33)));
  } else if (fabs(b12 * b13 * b23) > 0.0001) { // no one is zero
    if (b12 > 0) {
      return (Vec3d(sqrt(b11), sqrt(b22), -sqrt(b33)));
    } else if (b13 > 0) {
      return (Vec3d(sqrt(b11), -sqrt(b22), sqrt(b33)));
    } else {
      return (Vec3d(-sqrt(b11), sqrt(b22), sqrt(b33)));
    }
  } else if ((fabs(b12) > 0.0001) || (fabs(b13) > 0.0001) ||
             (fabs(b23) > 0.0001)) { // one ni is zero
    if (fabs(b12) > 0.0001) {
      if (b12 > 0)
        return (Vec3d(sqrt(b11), sqrt(b22), 0.0));
      else
        return (Vec3d(sqrt(b11), -sqrt(b22), 0.0));
    }
    if (fabs(b13) > 0.0001) {
      if (b13 > 0)
        return (Vec3d(sqrt(b11), 0.0, sqrt(b33)));
      else
        return (Vec3d(sqrt(b11), 0.0, -sqrt(b33)));
    }
    if (fabs(b23) > 0.0001) {
      if (b23 > 0)
        return (Vec3d(0.0, sqrt(b22), sqrt(b33)));
      else
        return (Vec3d(0.0, sqrt(b22), -sqrt(b33)));
    }
  } else { // two n_i are zero
    if (fabs(b11) > 0.0001) {
      return (Vec3d(sqrt(b11), 0.0, 0.0));
    } else if (fabs(b22) > 0.0001) {
      return (Vec3d(0.0, sqrt(b22), 0.0));
    } else {
      return (Vec3d(0.0, 0.0, sqrt(b33)));
    }
  }
}

Vec3d RotMat2AxisAngle(Mat3d rotMat) {
  double angle, cos_angle;

  angle = acos(1 / 2.0 * (rotMat(0,0)+rotMat(1,1)+rotMat(2,2) - 1));
  cos_angle = cos(angle);

  if (cos_angle < -0.999) { // condition that angle = pi
    Mat3d BMat = 0.5 * (rotMat + Eigen::MatrixXd::Identity(3,3));
    return (AxisAnglePi(BMat));
  } else if (cos_angle > 0.999) {
    // condition that angle = 0, axis is undetermined.
    return (Vec3d(0,0,0));
  } else {
    Mat3d temp = 1 / (2 * sin(angle)) * (rotMat - rotMat.transpose());
    Vec3d vN(temp(2, 1), temp(0, 2), temp(1, 0));
    return (angle * vN);
  }
}

THETA ur_InverseKinematics(Vec3d hand_p, Mat3d rotMat, THETA curTheta) {
  std::array<double,6> qJoint;
  double c1, s1, c2, s2, c3, s3, c4, s4, c5, s5, c6, s6;
  double nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz, temp1, temp2;
  nx = rotMat(0,0);  ox = rotMat(0,1);  ax = rotMat(0,2);  px = hand_p(0);
  ny = rotMat(1,0);  oy = rotMat(1,1);  ay = rotMat(1,2);  py = hand_p(1);
  nz = rotMat(2,0);  oz = rotMat(2,1);  az = rotMat(2,2);  pz = hand_p(2);

  // 关节角1 (t15_24)
  double A1 = py - ay*DH_D6, B1 = -px + ax*DH_D6;
  temp1 = atan2(DH_D4, sqrt(A1*A1+B1*B1-DH_D4*DH_D4)) - atan2(A1,B1);
  temp2 = atan2(DH_D4,-sqrt(A1*A1+B1*B1-DH_D4*DH_D4)) - atan2(A1,B1);
  qJoint[0] = fabs(temp1-curTheta[0])<fabs(temp2-curTheta[0]) ? temp1 : temp2;
  s1 = sin(qJoint[0]); c1 = cos(qJoint[0]);
  // 关节角5 (t15_22)
  temp1 =  acos(-ax*s1+ay*c1);
  temp2 = -acos(-ax*s1+ay*c1);
  qJoint[4] = fabs(temp1-curTheta[4])<fabs(temp2-curTheta[4]) ? temp1 : temp2;
  s5 = sin(qJoint[4]);
  // 关节角6 (t15_21)
  double A6 = ny*c1-nx*s1, B6 = -oy*c1+ox*s1;
  qJoint[5] = atan2(-s5,0) - atan2(A6,B6);
  // qJoint[5] = atan2(-B6/s5,-A6/s5);
  s6 = sin(qJoint[5]); c6 = cos(qJoint[5]);
  // 关节角3 (t14_14, t14_34)
  double A3 = px*c1 +py*s1 -DH_D5*(nx*c1*s6+ox*c1*c6+ny*s1*s6+oy*s1*c6) -DH_D6*(ay*s1+ax*c1);
  double B3 = pz -DH_D1 -az*DH_D6 -DH_D5*(oz*c6+nz*s6);
  temp1 =  acos((A3*A3+B3*B3-DH_A2*DH_A2-DH_A3*DH_A3)/(2*DH_A2*DH_A3));
  temp2 = -acos((A3*A3+B3*B3-DH_A2*DH_A2-DH_A3*DH_A3)/(2*DH_A2*DH_A3));
  qJoint[2] = fabs(temp1-curTheta[2])<fabs(temp2-curTheta[2]) ? temp1 : temp2;
  s3 = sin(qJoint[2]); c3 = cos(qJoint[2]);
  // 关节角2 (t14_14, t14_34)
  double A2 = -(DH_A3*DH_A3 +2*DH_A2*DH_A3*c3 +DH_A2*DH_A2);
  double B2 = -(DH_A3*c3+DH_A2)*A3 +DH_A3*s3*B3;
  double C2 = (DH_A3*c3+DH_A2)*B3 +DH_A3*s3*A3;
  qJoint[1] = atan2(C2/A2, B2/A2);
  // 关节角4 (t15_13, t15_33)
  double A4 = -(oz*c6 + nz*s6);
  double B4 = -c6*(ox*c1+oy*s1) -s6*(nx*c1+ny*s1);
  qJoint[3] = atan2(B4, A4) - qJoint[1] - qJoint[2];
  // Wrap qJoint[3] to (-pi,pi]
  if (qJoint[3] > M_PI) {
    qJoint[3] -= 2*M_PI;
  } else if (qJoint[3] < -M_PI) {
    qJoint[3] += 2*M_PI;
  }
  return qJoint;
}

