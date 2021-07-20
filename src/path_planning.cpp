#include "../include/path_planning.h"

// Shared variable
extern Config config;
extern Path_queue path_queue;

void add_joint_destination(PATH& path, NUMBUF& inputData) {
  // 角度伺服标志置位
  path.angleServo = ON;
  // 读入角度路径信息
  path.freq = 1/inputData[6];
  path.interpMode = inputData[7];
  for (int i = 0; i < 6; i++) {
    path.goal[i] = inputData[i]*Deg2Rad;
  }
  path.fingerPos = inputData[8];
  path_queue.push(path);
}

void add_displacement(PATH& path, NUMBUF& inputData) {
  double gain, temp;
  // 取消角度伺服标志
  path.angleServo = OFF;
  // 读入路径信息
  path.freq = 1/inputData[6];
  gain = path.freq * path.delT;
  for (int i=0; i<3; ++i) {
    // 平动位移量
    path.velocity[i] = inputData[i];
    // 转动位移量
    path.velocity[i+3] = inputData[i+3]*Deg2Rad;
  }
  // 归一化: 转化为一个伺服周期内的位移量
  for (int i=0; i<6; ++i) path.velocity[i] *= gain;
  path.fingerPos = inputData[8];
  path_queue.push(path);
}

void add_cartesion_destination(PATH& path, NUMBUF& inputData, THETA curTheta) {
  double oriR, oriP, oriY;
  Mat3d rotMat;
  // 角度伺服标志置位
  path.angleServo = ON;
  path.interpMode = 2;
  // 读入路径信息
  path.freq = 1/inputData[6];
  // 位置
  Vec3d handPos(inputData[0], DH_D4, inputData[2]);
  // 姿态
  oriP = inputData[3] * Deg2Rad;
  rotMat(0,0) = cos(oriP); rotMat(2,0) =  sin(oriP); rotMat(1,1) = -1;
  rotMat(0,2) = sin(oriP); rotMat(2,2) = -cos(oriP);
  path.fingerPos = inputData[8];
  path.goal = ur_InverseKinematics(handPos, rotMat, curTheta);
  path_queue.push(path);
}

void robot_go_home(THETA curTheta){
  PATH path;
  NUMBUF inputData = {400, DH_D4, 300, 0, 0, 0, 5, 0, 10};
  add_cartesion_destination(path, inputData, curTheta);
}

void pivot_about_points(TRIARR& state, TRIARR command, double time) {
  double xr = state[0], zr = state[1], qr = state[2];
  double x0 = command[0], z0 = command[1], q0 = command[2];
  double alpha0 = atan2(zr-z0, xr-x0);
  double radius = sqrt((zr-z0)*(zr-z0)+(xr-x0)*(xr-x0));

  // 按圆心角插值(圆的参数方程)
  THETA theta;
  double n = floor(time/SERVO_TIME);
  double da = command[2]/n;

  for (int i=0; i<n; ++i) {
    state[0] = x0 + radius*cos(alpha0+da*(i+1));
    state[1] = z0 + radius*sin(alpha0+da*(i+1));
    state[2] = qr + da*(i+1);
    // state[2] = qr;
    theta = plane_invese_kinematics(state);
    instant_command(theta);
  }
  printf("done\n");
}

void instant_command(THETA refTheta) {
  PATH pathLocal;
  pathLocal.fingerPos = 70;
  for (int i=0; i<6; ++i) {
    pathLocal.goal[i] = refTheta[i];
  }
  path_queue.push(pathLocal);
}

