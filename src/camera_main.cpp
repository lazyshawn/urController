#include "../include/camera_driver.h"

// 录制视频
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

// 标定
int main() {
  // 声明一个相机、标定类
  Camera camera;
  CameraCalibrator calibrator;

  /* Checking whether path exists */
  std::string picsDir = "../build/calibration/";
  check_up_folder(picsDir);

  /* Take photos */
  sample_photos_for_calibration(camera, picsDir);

  /* self_calibrate */
  // std::array<int,2> size = {11,8};
  // self_calibrate(picsDir, size);
  cv::Size boardSize(11,8);
  calibrator.add_Chessboard_Points(picsDir, boardSize);
  return 0;
}

  // 保存内参矩阵
  // std::ofstream f_calibration;
  // int len = 16;
  // f_calibration.open("./calibration/calibration.txt");
  // if(!f_calibration.is_open()){printf("Open file failed. -- curpos\n");}
  // for (int i=0; i<3; ++i) {
  //   for (int j=0; j<3; ++j) {
  //     f_calibration << std::left
  //       << cameraMatrix.at<float>(i,j) << "\t";
  //   }
  //     f_calibration << std::endl;
  // }
  // f_calibration.close();

