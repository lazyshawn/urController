/*******************************************************
* @file: common.h
* @brief: 通用设置
*******************************************************/
#ifndef COMMON_H
#define COMMON_H

// #define ROBOT_OFFLINE
#define INTEGRATE_CAMERA
#define INTEGRATE_SENSOR

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

#endif

