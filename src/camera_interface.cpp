#include "../include/camera_interface.h"

/* 共享变量 */
ObjState objState;
/* 外部变量 */
extern ThreadManager threadManager;


/*************************************************************************
 * @func: camera_thread_function
 * @brief: 相机线程
*************************************************************************/
void camera_thread_function(void) {
  // 声明一个相机类
  Camera camera;
  ObjState::Data objStateData;
  cv::Mat sampleFrame;
  // 设置相机外参
  Eigen::Matrix<double,4,4> T_cam2elk;
  T_cam2elk << -1,  0, 0, 36,
    0, -1, 0, 82,
    0,  0, 1, 26,
    0,  0, 0, 1;
  // T_cam2elk << -0.9987748,  0.0360119, 0.0339432, 31,
  //   -0.0359439, -0.9993594, 0.0026104, 82,
  //   0.0340151,  0.0013871, 0.9994293, 26,
  //   0, 0, 0, 1;
  camera.set_extrMat(T_cam2elk);

  while (threadManager.process != THREAD_EXIT) {
    objStateData = objState.get_data();

    sampleFrame = camera.get_color_frame();
    if (camera.detect_marker(sampleFrame, 14, 178, objStateData.markerPose)){
      objStateData.flag = true;
      // std::cout << objStateData.markerPose << std::endl;
      objState.update(&objStateData);
      break;
    }
  }

  objState.update(&objStateData);
  cv::destroyAllWindows();
  sleep(1);
  std::cout << "Thread terminated: camera_thread" << std::endl;
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


