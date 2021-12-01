#include "../include/camera_driver.h"

/*************************************************************************
 * 录制视频
*************************************************************************/
// int main() {
//   Camera camera;
//   cv::VideoWriter outputVideo = camera.create_recorder();
//
//   while (cv::waitKey(10) < 0) {
//     cv::Mat color = camera.get_color_frame();
//     imshow("Display color Image", color);
//     //保存当前帧
//     outputVideo << color;
//     imwrite("../build/calibrate.jpg",color);
//   }
//   return 0;
// }

/*************************************************************************
 * 内参标定
*************************************************************************/
// int main() {
//   // 声明一个相机、标定类
//   Camera camera;
//
//   /* Checking whether path exists */
//   camera.check_up_folder();
//
//   /* Take photos */
//   camera.sample_photos_for_calibration();
//
//   /* self_calibrate */
//   // std::array<int,2> size = {11,8};
//   // self_calibrate(picsDir, size);
//   camera.self_calibrate();
//   return 0;
// }

/*************************************************************************
 * Marker识别
 * @ref: http://m.mamicode.com/info-detail-2148085.html
 *       https://blog.csdn.net/qq_33446100/article/details/89115983
*************************************************************************/
int main() {
  Camera camera;
  cv::Mat frame, frame_show;
  Eigen::Matrix<double,4,4> markerPose;
  // 设置相机外参
  Eigen::Matrix<double,4,4> T_cam2elk;
  T_cam2elk << -0.9987748,  0.0360119, 0.0339432, 31,
    -0.0359439, -0.9993594, 0.0026104, 82,
    0.0340151,  0.0013871, 0.9994293, 26,
    0, 0, 0, 1;
  camera.set_extrMat(T_cam2elk);

  while(true) {
    frame = camera.get_color_frame();
    if (camera.detect_marker(frame, 14, 176, markerPose)) {
    // if (camera.detect_marker(frame, 14, 300, markerPose)) {
      std::cout << markerPose << std::endl;
    };

    char key = (char)cv::waitKey(30);
    if (key == 27) break;
  }

  return 0;
}

