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
#include <opencv2/aruco.hpp>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

class Camera {
private:
  rs2::pipeline pipe;       // 声明Realsense管道
  rs2::frameset frames;     // 创建一个rs2::frameset对象, 包含一组帧和访问它们的接口
  rs2::colorizer color_map; // 声明彩色图
  rs2::config cfg;          // 数据流信息配置文件
  rs2::pipeline_profile selection;
  rs2::frame color_frame, depth_frame;
  rs2_intrinsics intr;      // 相机内参
  Eigen::Matrix<double,4,4> extrMat; // 相机外参数

public:
  Camera();
  ~Camera();

  /* 标定相关的变量与函数 */
  // 标定图片储存文件夹
  std::string calibrationDir = "../build/calibration/";
  // 创建并清空标定文件夹
  void check_up_folder();
  // 设定外参
  void set_extrMat(Eigen::Matrix<double,4,4> tranMat);
  // 获取外参
  Eigen::Matrix<double,4,4> get_extrMat();

  /* 相机使用 */
  // 获取一帧彩色图片
  cv::Mat get_color_frame();
  // 获取一帧深度图片
  cv::Mat get_depth_frame();
  // 获取一段视频
  cv::VideoWriter create_recorder();
  // 获取像素点在相机坐标系下的 3D 坐标
  void pixel_to_point(float* point3d, float* pixel, float depth);
  // 检测 Marker 位姿
  int detect_marker(cv::Mat frame, int id, float depth, Eigen::Matrix<double,4,4>& markerPose);

};


#endif

