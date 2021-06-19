#ifndef ADROBOT_ETC_H
#define ADROBOT_ETC_H

#include "common.h"
#include "ur_kinematics.h"
#include <eigen3/Eigen/Dense>

using namespace Eigen;

#define JNT_PATH_SIN 0
#define JNT_PATH_5JI 1
#define JNT_PATH_3JI 2
#define JNT_PATH_1JI 3
#define JNT_PATH_STEP 4
#define JNT_PATH_SIN_FF 100
#define JNT_PATH_5JI_FF 200
#define JNT_PATH_3JI_FF 300
#define TRAJECTORY_LENGTH 20

typedef struct {
  int data_num;
  PATH Path[TRAJECTORY_LENGTH];
} TRJ_BUFF;

void initTrjBuff();
int PutTrjBuff(PATH* path);
int GetTrjBuff(PATH* path);
double CalcStepTraje(double orig, double goal, double freq, double time);
double Calc1JiTraje(double orig, double goal, double freq, double time);
double Calc3JiTraje(double orig, double goal, double freq, double time);
double Calc5JiTraje(double orig, double goal, double freq, double time);
double CalcSinTraje(double orig, double goal, double freq, double time);
double Calc3JiTrajeVelo(double orig, double goal, double freq, double time);
double Calc5JiTrajeVelo(double orig, double goal, double freq, double time);
double CalcSinTrajeVelo(double orig, double goal, double freq, double time);

void CalcJntRefPath(double curtime, PATH *path, THETA *theta, THETA *dtheta);
void CalcPosRefPath(double curtime,PATH *path,POS*pos);
MatrixXf ComputeDistance(float *coordinate,int a,float *P,int b);
MatrixXf ComputeRadiusVector(float *coordinate,int a,float *P,int b);
MatrixXf ComputeGripperToObjectJacobian(MatrixXf &GripperToPoint,MatrixXf &Radius);

#endif

