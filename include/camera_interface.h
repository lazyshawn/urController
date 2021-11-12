#ifndef CAMERA_INTERFACE_H
#define CAMERA_INTERFACE_H
#include "camera_driver.h"

#include "thread_pool.h"

class ObjState {
public:
  struct Data {
    std::array<double,3> twist;
    cv::Mat img;
    bool flag = false;
  };
  Data get_data(void);
  void update(Data* Data_);

private:
  Data data;
  std::mutex objstate_mutex;
  std::condition_variable objstate_cond;
};

void camera_thread_function(void);

#endif

