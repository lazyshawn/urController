#ifndef ADROBOT_ETC_H
#define ADROBOT_ETC_H

#include "common.h"
#include "ur_kinematics.h"

#define INTERP_SIN 0
#define INTERP_5JI 1
#define INTERP_3JI 2
#define INTERP_1JI 3
#define INTERP_STEP 4
#define JNT_PATH_SIN_FF 100
#define JNT_PATH_5JI_FF 200
#define JNT_PATH_3JI_FF 300
#define TRAJECTORY_LENGTH 20

void calc_ref_joint(SVO& svo);
bool joint_interpolation(double offsetTime, double freq, int interpMode, 
    ARRAY orig, ARRAY displacement, ARRAY& refVal);
bool velocity_interpolation(double offsetTime, double freq, THETA curTheta, 
    THETA& refTheta, ARRAY velocity);

double Calc3JiTrajeVelo(double orig, double goal, double freq, double time);
double Calc5JiTrajeVelo(double orig, double goal, double freq, double time);
double CalcSinTrajeVelo(double orig, double goal, double freq, double time);

#endif

