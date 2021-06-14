/* ***********************************************************
 *     @file: data_exchange.h
 * @function: 数据保存、处理线程之间的数据交换
 *   @author: lazyshawn
 * ***********************************************************/

#ifndef DATA_EXCHANGE_H
#define DATA_EXCHANGE_H

#include "common.h"
#include "trajectory.h"
#include "system_time.h"
// 文件读写
#include <fstream>
#include <iostream>
#include <iomanip>

void SvoReadFromServo(SVO *data);
void SvoWriteFromServo(SVO *data);
void SvoReadFromGui(SVO *data);
void SvoWriteFromGui(SVO *data);
void SvoReadFromDis(SVO *data);
void ChangePathData(PATH*path);
void ChangeHandData(PATH*path);
void PosOriServo(int*posoriservoflag);
void SetPosOriSvo(SVO*data);
void SetJntSvo(SVO *data);

void ExpDataSave(SVO_SAVE* data);
void SaveDataReset();
void ExpDataWrite();

#endif

