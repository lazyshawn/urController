/*******************************************************
* @file: common.h
* @brief: 通用设置
*******************************************************/
#ifndef COMMON_H
#define COMMON_H

#define ROBOT_OFFLINE
#define INTEGRATE_CAMERA
#define INTEGRATE_SENSOR

#define ON 1
#define OFF 0
#define INIT_C 0
#define EXIT_C 255

// 实验数据的最大保存数
#define EXP_DATA_LENGTH 10000
#define EXP_DATA_INTERVAL 1
#define Rad2Deg 180.0 / M_PI
#define Deg2Rad M_PI / 180.0
// 一秒内的纳秒数
#define NSEC_PER_SEC (1000000000) /* 1 s */
// 一个伺服周期内的纳秒数
#define NSEC_PER_PERIOD (8000000) /* 8 ms */

#include <array>
#include <cmath>

extern double SERVO_TIME;

struct shm_interface {
  shm_interface() {
    status_control = INIT_C;
  }
  int status_print;
  int status_control;
};

// PID 控制参数
struct GAIN {
  double K_vt[6];
  double K_pt[6];
  double K_it[6];
};

struct JACOBIAN {
  double Jcb[6][6];
  double TrnsJcb[6][6];
  double invJcb[6][6];
};

// 六维数组
typedef std::array<double,6> ARRAY;
// 关节角_6
typedef std::array<double,6> THETA;
// 位姿: 位置_3 + 姿态_4
typedef std::array<double,7> POS;
// 键盘读入的缓冲
// 0-5                | 6    | 7          | 8
//Joint/Pose/Velocity | time | interpMode | fingerPos
typedef std::array<double,10> NUMBUF;
typedef std::array<double,3> TRIARR;

// 路径
struct PATH {
  PATH () {
    angleServo = true;
    complete = false;
    delT = (double)NSEC_PER_PERIOD / (double)NSEC_PER_SEC;
    velocity = {0,0,0,0,0,0};
    fingerPos = 0;
    freq = 1/delT;
    interpMode = 2;
    fingerPos = -1;
  }

  double beginTime;  // 开始时间
  double freq;       // 插补频率: 总时间的倒数
  bool angleServo;   // 角度伺服标志
  bool complete;     // 轨迹完成标志
  int interpMode;    // 插补模式
  ARRAY orig;        // 插值起点(角度伺服) [mm]
  ARRAY goal;        // 插值终点(角度伺服) [mm]
  ARRAY velocity;    // 速度命令(位姿伺服) [mm/s, rad/s]
  double delT;       // 伺服周期时间
  double fingerPos;  // 手指位置
};

// 全局的线程共享结构体: 系统状态
struct SVO {
  SVO () {
    path.complete = true;
  }

  double time;
  PATH path;
  THETA curTheta;
  THETA refTheta;
  POS curPos;
  POS refPos;
};

#endif

