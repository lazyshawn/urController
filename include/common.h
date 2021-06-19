
#ifndef COMMON_H
#define COMMON_H

#define DEBUG
#define ROBOT_OFFLINE
#define ON 1
#define OFF 0
#define INIT_C 0
#define EXIT_C 255
#define EXP_DATA_LENGTH 10000
#define EXP_DATA_INTERVAL 1
#define Rad2Deg 180.0 / M_PI
#define Deg2Rad M_PI / 180.0

struct shm_interface {
  int status_print;
  int status_control;
};
typedef double THETA;
typedef double POS;
typedef double FORCE;
typedef struct {
  double Orig[6];  // 轨迹起点
  double Goal[6];  // 轨迹终点
  double Freq;     // 单位时间运动的比例 (1/T)
  int Mode;        // 插补模式
} PATH;
// PID 控制参数
typedef struct {
  double K_vt[6];
  double K_pt[6];
  double K_it[6];
} GAIN;
typedef struct {
  double Jcb[6][6];
  double TrnsJcb[6][6];
  double invJcb[6][6];
} JACOBIAN;
// 全局的线程共享结构体
typedef struct {
  int PosOriServoFlag;             // 位姿伺服标志
  int SamplingFreq;
  double SamplingTime;
  double Time;                     // 当前时间
  int ServoFlag;                   // 伺服控制开始的标志
  int NewPathFlag;                 // 取出新路径标志
  int PathtailFlag;                // 路劲完成标志
  THETA CurTheta[6];                  // 机械臂关节角(deg)
  THETA CurDTheta[6];                 // 机械臂关节角(rad)
  THETA RefTheta[6];                  // 目标关节角(deg)
  THETA RefDTheta[6];
  POS CurPos[6];
  POS RefPos[6];
  FORCE CurForce[6];
  GAIN Gain;
  PATH Path;                      // 正在进行的路径
} SVO;

#endif

