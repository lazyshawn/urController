#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include "common.h"
#include "ur_kinematics.h"
#include "data_exchange.h"

void add_displacement(PATH& path, NUMBUF& inputData);
void add_joint_destination(PATH& path, NUMBUF& inputData);
void add_cartesion_destination(PATH& path, NUMBUF& inputData, THETA curTheta);
void robot_go_home(THETA curTheta);
void pivot_about_points(TRIARR& state, TRIARR command, double time);
void instant_command(THETA curTheta);

#endif

