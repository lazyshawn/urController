/* ***********************************************************
 *     @file: data_exchange.h
 * @function: 数据保存、处理线程之间的数据交换
 *   @author: lazyshawn
 * ***********************************************************/

#ifndef DATA_EXCHANGE_H
#define DATA_EXCHANGE_H

#include "common.h"
// 文件读写
#include <fstream>
#include <iostream>
#include <iomanip>
// Mutural exclusion
#include <mutex>
#include <condition_variable>
// std::deque
#include <deque>

class Config {
private:
  SVO data;
  std::mutex config_mutex;
  std::condition_variable config_cond;

public:
  // 复制 Config
  SVO getCopy(void);
  // 更新 Config
  void update(SVO* SVO_);
};

class Path_queue {
private:
  std::deque<PATH> data;
  mutable std::mutex path_mutex;
  std::condition_variable path_cond;

public:
  Path_queue(){};
  void push(PATH path);
  void wait_and_pop(PATH& path);
  bool try_pop(PATH& path);
  bool try_pop(PATH& path, double time, ARRAY orig);
  bool empty() const;
  // for debug
  void wait();
  void notify_one();
};

void add_hand_path(PATH& path);
void add_joint_path(PATH& path);

void ExpDataSave(SVO* data);
void SaveDataReset();
void ExpDataWrite();

#endif

