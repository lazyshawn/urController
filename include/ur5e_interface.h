#ifndef UR_INTERFACE_H
#define UR_INTERFACE_H

#include <mutex>
#include <condition_variable>
#include <deque>

#include "../include/common.h"

class urConfig {
public:
  struct Data {
    Data () {
      path.complete = true;
    }

    double time;
    PATH path;
    THETA curTheta, refTheta;
    POS curPos, refPos;
  };

  // 复制 Config
  Data get_data(void);
  // 更新 Config
  void update(Data* Data_);

private:
  Data data;
  std::mutex config_mutex;
  std::condition_variable config_cond;
};

class PathQueue {
private:
  std::deque<PATH> data;
  mutable std::mutex path_mutex;
  std::condition_variable path_cond;

public:
  PathQueue(){};
  void push(PATH path);
  void wait_and_pop(PATH& path);
  bool try_pop(PATH& path);
  bool try_pop(PATH& path, double time, ARRAY orig);
  bool empty() const;
  // for debug
  void wait();
  void notify_one();
};

#endif

