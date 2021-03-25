/*
 * servo.h
 *
 *  Created on: Apr 20, 2018
 *      Author: jch
 */
#ifndef ADROBOT_CONTROL_H
#define ADROBOT_CONTROL_H
#include "common.h"
#include "ur_driver.h"
#include "robot_state_RT.h"
#include "adrobot_kinematics.h"
/*Eigen header*/
#include <eigen3/Eigen/Dense>
/*Eigen header*/
using namespace Eigen;

extern void servo_function(UrDriver* ur);
extern void CalcJntRefPath(double curtime, PATH *path, THETA *theta, THETA *dtheta);
extern void CalcPosRefPath(double curtime,PATH *path,POS*pos);
extern void CalcPointpos(double curtime,PATH*path,POSITION  *refmarkpos);
extern MatrixXf ComputeDistance(float *coordinate,int a,float *P,int b);
extern MatrixXf ComputeRadiusVector(float *coordinate,int a,float *P,int b);
extern MatrixXf ComputeGripperToObjectJacobian(MatrixXf &GripperToPoint,MatrixXf &Radius);
#endif
