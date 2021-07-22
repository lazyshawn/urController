#include "../include/camera.h"

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
  /* Checking whether path exists */
  std::string pics_dir_for_calibration = "../build/calibration/";
  check_up_folder(pics_dir_for_calibration);

  /* Take photos */
  sample_photos_for_calibration(pics_dir_for_calibration);

  /* self_calibrate */
  std::array<int,2> boardSize = {11,8};
  self_calibrate(pics_dir_for_calibration, boardSize);
  return 0;
}

