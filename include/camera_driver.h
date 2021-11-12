#ifndef CAMERA_DRIVER_H
#define CAMERA_DRIVER_H

#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <librealsense2/hpp/rs_types.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

class Camera {
private:
  rs2::pipeline pipe;       // 声明Realsense管道
  rs2::frameset frames;     // 创建一个rs2::frameset对象, 包含一组帧和访问它们的接口
  rs2::colorizer color_map; // 声明彩色图
  rs2::config cfg;          // 数据流信息配置文件
  rs2::pipeline_profile selection;
  rs2::frame color_frame, depth_frame;
  cv::Size size;

public:
  Camera();
  ~Camera();

  /* 标定相关的变量与函数 */
  // 标定图片储存文件夹
  std::string calibrationDir = "../build/calibration/";
  // 旋转量, 平移量
  std::vector<cv::Mat> rvecs, tvecs;
  // 内参矩阵、畸变参数
  cv::Mat cameraMatrix, disCoeffs;
  // 创建并清空标定文件夹
  void check_up_folder();
  // 采集用于标定的棋盘格图片
  void sample_photos_for_calibration();
  // 打开棋盘格图片，并提取角点
  int self_calibrate(cv::Size boardSize=cv::Size(11,8));

  /* 相机使用 */
  // 获取一帧彩色图片
  cv::Mat get_color_frame();
  // 获取一帧深度图片
  cv::Mat get_depth_frame();
  // 获取一段视频
  cv::VideoWriter create_recorder();
};


#endif

