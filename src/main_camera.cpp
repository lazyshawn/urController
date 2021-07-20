#include "../include/camera.h"

int main() {
  Camera camera;
  cv::VideoWriter outputVideo = camera.create_recorder();

  while (cv::waitKey(10) < 0) {
    cv::Mat color = camera.get_color_frame();
    imshow("Display color Image", color);
    //保存当前帧
    outputVideo << color;
  }
  return 0;
}

