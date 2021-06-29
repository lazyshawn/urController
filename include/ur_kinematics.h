/* 
 * Check doc/ur_kinematics/ur.md for more details.
 */

#ifndef UR_KINEMATICS_H
#define UR_KINEMATICS_H

#include "common.h"
#include "matrix.h"
#include <vector>

// Modified DH parameters for UR5
#define DH_D1 89.459
#define DH_A3 425.00
#define DH_A4 392.25
#define DH_D4 109.15
#define DH_D5 94.65
#define DH_D6 82.3 

/* 
 * @brief: 更新各关节角的正余弦值;
 * @param: 关节角向量;
 * @return: ;
 */
int calcJnt(THETA q);

/* 
 * @brief: M-DH求解机械臂运动学
 * @param: 旋转矩阵(ori_hnd);
 * @return: 末端位姿向量(6x1)
 */
MATRIX_D ur_kinematics(MATRIX_D &ori_hnd);

/* 
 * @brief: 计算从关节空间到笛卡尔空间的雅克比矩阵，及其逆矩阵、转置矩阵
 * @param: 雅克比矩阵结构体指针(jcbn);
 * @return: 雅克比矩阵
 */
MATRIX_D ur_jacobian(JACOBIAN *jcbn);

/* 
 * @brief: 检验矩阵奇异性，并计算矩阵的逆;
 * @param: 原矩阵(jcb); 逆矩阵(ijcb);
 * @return: 矩阵奇异(-1) | 无异常(0);
 */
int calcInverse(MATRIX_D &jcb, MATRIX_D &ijcb);

MATRIX_D RPY2RotMat(double alpha, double beta, double gamma);
MATRIX_D RotMat2AxisAngle(MATRIX_D rotMat);
MATRIX_D RotMat2EulerAngle(MATRIX_D rotMat);
MATRIX_D ur_InverseKinematics(MATRIX_D hand_p, MATRIX_D rotMat);
#endif

