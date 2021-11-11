/* ***********************************************************
 *     @file: data_exchange.h
 * @function: 数据保存、处理线程之间的数据交换
 *   @author: lazyshawn
 * ***********************************************************/

#ifndef DATA_EXCHANGE_H
#define DATA_EXCHANGE_H

#include "common.h"
#include "ur_kinematics.h"
#include "ur5e_interface.h"
// 文件读写
#include <fstream>
#include <iostream>
#include <iomanip>
// Mutural exclusion
#include <mutex>
#include <condition_variable>
// std::deque
void read_displacement(NUMBUF& inputData);
void read_joint_destination(NUMBUF& inputData);
void read_cartesion_destination(NUMBUF& inputData);

void ExpDataSave(urConfig::Data* data);
void SaveDataReset();
void ExpDataWrite();

#endif

