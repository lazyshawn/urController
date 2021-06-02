
#ifndef COMMON_H
#define COMMON_H

#define DEBUG
//#define ROBOT_OFFLINE
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

typedef struct {
  double t[6];
} THETA;

typedef struct {
  double t[6];
} POS;

typedef struct {
  double t[6];
} FORCE;

typedef struct {
  double Orig[6];  // 轨迹起点
  double Goal[6];  // 轨迹终点
  double Freq;     // 单位时间运动的比例 (1/T)
  int Mode;        // 插补模式
} PATH;

typedef struct {
  double t[6];
} HND_PA;

typedef struct {
  double K_vt[6];
  double K_pt[6];
  double K_it[6];
} GAIN;

/* Mark位置向量 */
typedef struct {
  float t[18];
} MARKPOS;

typedef struct {
  double t[6];
} DISTOGRIPPER;

typedef struct {
  double t[18];
} POSITION;
// extern MatrixXd MinimumDistance(1,10);

// 关节角的余弦正弦值
typedef struct {
  double c[6];
  double s[6];
  double c23, s23, c234, s234;
} JOINTLINK;
typedef struct {
  double Jcb[6][6];
  double TrnsJcb[6][6];
  double invJcb[6][6];
} JACOBIAN;

typedef struct {
  int PosOriServoFlag;             // 位姿伺服标志
  int SamplingFreq;
  double SamplingTime;
  double Time;                     // 当前时间
  int ServoFlag;                   // 伺服控制开始的标志
  int NewPathFlag;                 // 取出新路径标志
  int PathtailFlag;                // 路劲完成标志
  THETA CurTheta;                  // 机械臂关节角(deg)
  THETA CurDTheta;                 // 机械臂关节角(rad)
  THETA RefTheta;                  // 目标关节角(deg)
  THETA RefDTheta;
  THETA deltaTheta;
  THETA xianweiTheta;
  JOINTLINK jnk;
  POS CurPos;
  POS RefPos;
  FORCE CurForce;
  GAIN Gain;
  PATH Path;                      // 正在进行的路径
  MARKPOS markpos;                // mark位置 (float t[18])
  DISTOGRIPPER distogripper;
  POSITION errpos;
  POSITION refmarkpos;
} SVO;

typedef struct {
  double Time;
  THETA CurTheta;
  THETA CurDTheta;
  THETA RefTheta;
  THETA RefDTheta;
  THETA deltaTheta;
  THETA xianweiTheta;
  POS CurPos;
  POS RefPos;
  PATH Path;
  FORCE CurForce;
  GAIN Gain;
  MARKPOS markpos;
  DISTOGRIPPER distogripper;
  POSITION errpos;
  POSITION refmarkpos;
} SVO_SAVE;

#endif

