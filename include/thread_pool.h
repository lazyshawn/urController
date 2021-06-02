/*
 * servo.h
 *
 *  Created on: Apr 20, 2018
 *      Author: jch
 */
#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include "common.h"
#include "data_exchange.h"
#include "trajectory.h"
#include "system_time.h"
#include "user_interface.h"
#include "ur_kinematics.h"
#include "ur_driver.h"
#include "matrix.h"
#include <sys/mman.h>  // 内存管理
#include <pthread.h>   // 线程管理

void servo_function(UrDriver* ur);
void *display_function(void *param);
void *interface_function(void *param);

#endif

