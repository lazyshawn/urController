/* 
 * Check doc/ur_kinematics/ur.md for more details.
 */

#ifndef UR_KINEMATICS_H
#define UR_KINEMATICS_H

#include "common.h"
// #include "matrix.h"
#include <vector>
#include <eigen3/Eigen/Dense>

// Modified DH parameters for UR5
#define DH_D1 89.459
#define DH_A2 425.00
#define DH_A3 392.25
#define DH_D4 109.15
#define DH_D5 94.65
#define DH_D6 82.3 

typedef Eigen::Matrix3d Mat3d;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Matrix<double,6,1> Vec6d;
typedef Eigen::Matrix<double,6,6> Mat6d;

/* 
 * @func : calc_joint
 * @brief: 更新各关节角的正余弦值;
 * @param: 关节角向量;
 * @return: 错误(-1) | 正常执行(0);
 */
int calcJnt(THETA q);

bool plane_kinematics(std::array<double,3>& state);

THETA plane_invese_kinematics(std::array<double,3>& state);

/* 
 * @func : ur_kinematics
 * @brief: M-DH求解机械臂运动学
 * @param: 旋转矩阵(ori_hnd);
 * @return: 末端位姿向量(6x1)
 */
Vec6d ur_kinematics(Mat3d& oriMat);

/* 
 * @brief: 计算从关节空间到笛卡尔空间的雅克比矩阵，及其逆矩阵、转置矩阵
 * @param: 雅克比矩阵结构体指针(jcbn);
 * @return: 雅克比矩阵
 */
Mat6d ur_jacobian(void);

Mat3d RPY2RotMat(double alpha, double beta, double gamma);
Vec3d RotMat2AxisAngle(Mat3d rotMat);
Vec3d RotMat2EulerAngle(Mat3d rotMat);
THETA ur_InverseKinematics(Vec3d hand_p, Mat3d rotMat, THETA curTheta);
#endif

