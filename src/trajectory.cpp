
#include "../include/trajectory.h"

TRJ_BUFF PathBuff;

/* 初始化轨迹队列 */
void initTrjBuff() { PathBuff.data_num = 0; }

/* 添加轨迹队列 */
int PutTrjBuff(PATH *path) {
  // if the buffer is full
  if (PathBuff.data_num >= TRAJECTORY_LENGTH) {
    printf("Error: The path buffer is full\n");
    return 1;
  } else {
    if (PathBuff.data_num > 0) {
      for (int i = PathBuff.data_num; i > 0; i--) {
        PathBuff.Path[i] = PathBuff.Path[i-1];
      }
    }
    PathBuff.Path[0] = *path;
    PathBuff.data_num++;
  } // else
  return 0;
}

/* 弹出轨迹队列 */
int GetTrjBuff(PATH *path) {
  // if the buffer is not empty
  if (PathBuff.data_num < 1) {
    printf("Error: The path buffer is empty");
    return 1;
  } else {
    // 取出队首元素
    *path = PathBuff.Path[0];
    // 更新路径队列
    for (int i = 0; i < PathBuff.data_num; i++) {
      PathBuff.Path[i] = PathBuff.Path[i+1];
    }
    PathBuff.data_num--;
  }
  return 0;
}

// ステップ関数による軌道補間 | 阶跃函数的轨迹插补
double CalcStepTraje(double orig, double goal, double freq, double time) {
  if (time < 1.0 / freq / 2.0)
    return orig;
  return goal;
}

// 1 次関数による軌道補間 | 一次函数的轨迹插补
double Calc1JiTraje(double orig, double goal, double freq, double time) {
  double ref = goal;
  double time_n = freq * time;

  if (time_n <= 1)
    ref = orig + (goal - orig) * time_n;
  return ref;
}

// 3 次関数による軌道補間 | 3次函数的轨迹插补
double Calc3JiTraje(double orig, double goal, double freq, double time) {
  double ref = goal;
  double time_n = freq * time;

  if (time_n <= 1)
    ref = orig + (goal - orig) * time_n * time_n * (3.0 - 2 * time_n);
  return ref;
}

// 5 次関数による軌道補間 | 5次函数的轨迹插补
double Calc5JiTraje(double orig, double goal, double freq, double time) {
  double ref = goal;
  double time_n = freq * time;

  if (time_n <= 1)
    ref = orig + (goal - orig) * time_n * time_n * time_n *
          (10.0 + time_n * (-15.0 + 6.0 * time_n));
  return ref;
}

// sin 関数による軌道補間 | sin函数的轨迹插补
double CalcSinTraje(double orig, double goal, double freq, double time) {
  double ref = goal;
  double time_n = freq * time;

  if (time_n <= 1)
    ref = orig + (goal - orig) * time_n -
          (goal - orig) * sin(2.0 * M_PI * time_n) / (2.0 * M_PI);
  return ref;
}

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


/* 位置伺服: 计算轨迹插补点(笛卡尔空间) */
// curtime: 当前时间; path: 轨迹; theta: 目标角度; dtheta: 指定角速度.
void CalcJntRefPath(double curtime, PATH *path, THETA *theta, THETA *dtheta) {
  double *orig, *goal, *ref, *ref_v;

  orig = path->Orig;
  goal = path->Goal;
  ref = (double *)theta;
  ref_v = (double *)dtheta;
  for (int i = 0; i < 6; i++) {
    switch (path->Mode) {
    case JNT_PATH_5JI:
    case JNT_PATH_5JI_FF:
      ref[i] = Calc5JiTraje(orig[i], goal[i], path->Freq, curtime);
      break;
    case JNT_PATH_3JI:
    case JNT_PATH_3JI_FF:
      ref[i] = Calc3JiTraje(orig[i], goal[i], path->Freq, curtime);
      break;
    case JNT_PATH_1JI:
      ref[i] = Calc1JiTraje(orig[i], goal[i], path->Freq, curtime);
      break;
    case JNT_PATH_STEP:
      ref[i] = CalcStepTraje(orig[i], goal[i], path->Freq, curtime);
      break;
    default:
      ref[i] = CalcSinTraje(orig[i], goal[i], path->Freq, curtime);
    }
    switch (path->Mode) {
    case JNT_PATH_3JI_FF:
      ref_v[i] = Calc3JiTrajeVelo(orig[i], goal[i], path->Freq, curtime);
      break;
    case JNT_PATH_5JI_FF:
      ref_v[i] = Calc5JiTrajeVelo(orig[i], goal[i], path->Freq, curtime);
      break;
    case JNT_PATH_SIN_FF:
      ref_v[i] = CalcSinTrajeVelo(orig[i], goal[i], path->Freq, curtime);
      break;
    default:
      ref_v[i] = 0.;
    }
  }
}

/* 角度伺服: 计算轨迹插补点(关节角空间) */
// Path planning
void CalcPosRefPath(double curtime, PATH *path, POS *refpos) {
  int i;
  double *orig, *goal, *ref;

  orig = path->Orig;
  goal = path->Goal;
  ref = (double *)refpos;
  for (i = 0; i < 6; i++) {
    switch (path->Mode) {
    case JNT_PATH_5JI:
    case JNT_PATH_5JI_FF:
      ref[i] = Calc5JiTraje(orig[i], goal[i], path->Freq, curtime);
      break;
    case JNT_PATH_3JI:
    case JNT_PATH_3JI_FF:
      ref[i] = Calc3JiTraje(orig[i], goal[i], path->Freq, curtime);
      break;
    case JNT_PATH_1JI:
      ref[i] = Calc1JiTraje(orig[i], goal[i], path->Freq, curtime);
      break;
    case JNT_PATH_STEP:
      ref[i] = CalcStepTraje(orig[i], goal[i], path->Freq, curtime);
      break;
    default:
      ref[i] = CalcSinTraje(orig[i], goal[i], path->Freq, curtime);
    }
  }
}

/* 计算末端tcp位置 */
void CalcPointpos(double curtime, PATH *path, POSITION *refmarkpos) {
  int i;
  double *ref_point_pos;
  double start[18] = {
      0.856956, -0.0214573, 0.0256758, 0.814638, -0.0191454, 0.022805,
      0.773164, -0.0201868, 0.0199828, 0.732798, -0.0212884, 0.0172356,
      0.689072, -0.0226941, 0.0142593, 0.646059, -0.0249611, 0.0113292};
  double end[18] = {0.856234,   -0.022849,  0.025623,   0.813418,   -0.0299673,
                    0.0226936,  0.772993,   -0.0389335, 0.0199218,  0.733945,
                    -0.0486663, 0.0172417,  0.691313,   -0.0606541, 0.0143119,
                    0.650016,   -0.0733437, 0.0114711};
  ref_point_pos = (double *)refmarkpos;
  for (int i = 0; i < 18; i++) {
    ref_point_pos[i] = Calc1JiTraje(start[i], end[i], path->Freq, curtime);
  }
}

/* CalcPointpos */
/* deformable_jacobi | 变形雅克比 */
MatrixXf ComputeDistance(float *coordinate, int a, float *P, int b) {
  float x, y, z;
  float d[6];
  MatrixXf n(1, 6);
  x = P[0];
  y = P[1];
  MatrixXf m(18, 1);
  for (int i = 0; i < 18; i++) {
    m(i, 0) = coordinate[i];
  }
  for (int j = 0; j < 6; j++) {
    /*计算末端tcp到标记点的距离*/
    d[j] = sqrt((m(j * 3, 0) - x) * (m(j * 3, 0) - x) +
                (m(j * 3 + 1, 0) - y) * (m(j * 3 + 1, 0) - y));
  }
  for (int k = 0; k < 6; k++) {
    n(0, k) = d[k];
  }
  return n;
}

MatrixXf ComputeRadiusVector(float *coordinate, int a, float *P, int b) {
  float x, y, z; /*r=点在机器人坐标系下的坐标-夹持点在机器人坐标系下的坐标*/
  float d[18];
  MatrixXf N(18, 1);
  x = P[0];
  y = P[1];
  z = 1;
  MatrixXf PointCoordinate(18, 1);
  MatrixXf Radius(18, 1);
  for (int i = 0; i < 18; i++) {
    PointCoordinate(i, 0) = coordinate[i];
  }
  for (int j = 0; j < 6; j++) {
    Radius(j * 3, 0) = PointCoordinate(j * 3, 0) - x;
    Radius(j * 3 + 1, 0) = PointCoordinate(j * 3 + 1, 0) - y;
    Radius(j * 3 + 2, 0) = PointCoordinate(j * 3 + 2, 0) - z;
  }
  for (int k = 0; k < 6; k++) {
    N(k * 3, 0) = Radius(k * 3, 0);
    N(k * 3 + 1, 0) = Radius(k * 3 + 1, 0);
    N(k * 3 + 2, 0) = Radius(k * 3 + 2, 0);
  }
  return N;
}

MatrixXf ComputeGripperToObjectJacobian(MatrixXf &GripperToPoint,
                                        MatrixXf &Radius) {
  int num_grippers = 1;
  int num_nodes = 6;
  int num_cols = num_grippers * 6;
  int num_rows = num_nodes * 3;
  MatrixXf J(18, 6);
  MatrixXf MinimumDistance(1, 6); /*夹持器到各个点的最短距离*/
  MinimumDistance = GripperToPoint;
  MatrixXf RadiusVetor(18, 1);
  RadiusVetor = Radius;

  // Get all the data we need for a given gripper

  for (int node_ind = 0; node_ind < num_nodes; node_ind++) {
    int gripper_ind = 0;
    float translation_deformability = 45;
    float rotation_deformability = 45;
    Matrix3f J_trans;
    J_trans << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    const float dist_to_gripper = MinimumDistance(gripper_ind, node_ind);
    J.block<3, 3>(node_ind * 3, 0) =
        exp(-translation_deformability * dist_to_gripper) * J_trans;
    Matrix3f J_rot;

    // b1= RadiusVetor(node_ind*3,0);     [  0, b3,-b2
    // b2= RadiusVetor(node_ind*3+1,0);     -b3, 0, b1
    // b3= RadiusVetor(node_ind*3+2,0);      b2,-b1,0 ]

    J_rot(0, 0) = 0;
    J_rot(0, 1) = RadiusVetor(node_ind * 3 + 2, 0);
    J_rot(0, 2) = -RadiusVetor(node_ind * 3 + 1, 0);
    J_rot(1, 0) = -RadiusVetor(node_ind * 3 + 2, 0);
    J_rot(1, 1) = 0;
    J_rot(1, 2) = RadiusVetor(node_ind * 3, 0);
    J_rot(2, 0) = RadiusVetor(node_ind * 3 + 1, 0);
    J_rot(2, 1) = -RadiusVetor(node_ind * 3, 0);
    J_rot(2, 2) = 0;

    J.block<3, 3>(node_ind * 3, 3) =
        exp(-rotation_deformability * dist_to_gripper) * J_rot;

    // block（p,
    // q）可理解为一个p行q列的子矩阵，该定义表示从原矩阵中第(i,j)开始，获取一个p行q列的子矩阵，返回该子矩阵组成的临时矩阵对象，原矩阵的元素不变
  }
  return J;
}
/*deformable_jacobi*/


