#include "../include/camera_interface.h"

// 声明一个相机类
Camera camera;
/* 共享变量 */
ObjState objState;
/* 外部变量 */
extern ThreadManager threadManager;

/*************************************************************************
 * @func: camera_thread_function
 * @brief: 相机线程
*************************************************************************/
void camera_thread_function(void) {
  ObjState::Data objStateData;
  cv::Mat sampleFrame;
  int i;

  /* Checking whether path exists */
  camera.check_up_folder();
  /* Take photos */
  camera.sample_photos_for_calibration();
  /* self_calibrate */
  camera.self_calibrate();

  // while (threadManager.process != THREAD_EXIT) {
  //   objStateData = objState.get_data();
  //
  //   sampleFrame = camera.get_color_frame();
  //   imshow("Display color Image", sampleFrame);
  //
  //   ++i;
  //   objStateData.twist[0] = i;
  //   objStateData.img = sampleFrame;
  //   objState.update(&objStateData);
  //   if(cv::waitKey(10) == 27) break;
  // }
  // objStateData.flag = true;
  // objState.update(&objStateData);
  // cv::destroyAllWindows();
  // std::cout << "Thread terminated: camera_thread" << std::endl;
}

/*************************************************************************
 * @class: ObjState
 * @brief: 相机线程采集的数据，主要是物体状态
*************************************************************************/
ObjState::Data ObjState::get_data(void) {
  std::scoped_lock guard(objstate_mutex);
  return data;
};

void ObjState::update(Data* Data_) {
  std::scoped_lock guard(objstate_mutex);
  data = *Data_;
};


