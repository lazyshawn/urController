
#ifndef UR_KINEMATICS_H
#define UR_KINEMATICS_H

#include "common.h"
#include "matrix.h"

#define UR_ARM_OFFSET_D1 0.089459
#define UR_ARM_OFFSET_A2 0.42500
#define UR_ARM_OFFSET_A3 0.39225
#define UR_ARM_OFFSET_D4 0.10915
#define UR_ARM_OFFSET_D5 0.09465
#define UR_ARM_OFFSET_D6 0.2323
//#define UR_ARM_OFFSET_D6 0.0823

MATRIX_D ur_kinematics(JOINTLINK *jnk, MATRIX_D &p);
MATRIX_D RotMat2AxisAngle(MATRIX_D rotMat);
MATRIX_D ur_InverseKinematics(MATRIX_D hand_p, MATRIX_D rotMat);
MATRIX_D RPY2RotMat(double alpha, double beta, double gamma);
MATRIX_D RotMat2EulerAngle(MATRIX_D rotMat);
MATRIX_D CalcB0(double alpha, double beta, double gamma);
MATRIX_D ur_jacobian(JOINTLINK *jnk, JACOBIAN *jcbn);
int CalcInverse(MATRIX_D &jcb, MATRIX_D &ijcb);
#endif

