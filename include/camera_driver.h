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
#include "user_interface.h"

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
  /* 
   * @func  : Camera
   * @brief : 构造函数
   * @param : 
   */
  Camera();
  /* 析构函数 */
  ~Camera();
  /* 
   * @func  : get_color_frame
   * @brief : 获取一帧彩色图片
   * @param : 
   * @return: 
   */
  cv::Mat get_color_frame();
  /* 
   * @func  : get_depth_frame
   * @brief : 获取一帧深度图片
   * @param : 
   * @return: 
   */
  cv::Mat get_depth_frame();
  /* 
   * @func  : create_recorder
   * @brief : 生成一个视频记录对象
   * @param : 
   * @return: 
   */
  cv::VideoWriter create_recorder();
};

class CameraCalibrator {
private:
  /* 输入点: */
  // 世界坐标中的点
  std::vector<std::vector<cv::Point3f>> objectPoints;
  // 点的位置(以像素为单位)
  std::vector<std::vector<cv::Point2f>> imagePoints;
  // 输出矩阵、畸变参数
  cv::Mat cameraMatrix, disCoeffs;
  // 指定校准方式的标志
  int flag;
public:
  // 旋转量, 平移量
  std::vector<cv::Mat> rvecs, tvecs;
  // 打开棋盘格图像，并提取角点
  int add_Chessboard_Points(const std::string picsDir, cv::Size& boardSize);
  // 校准相机，返回重投影精度
  double calibrate(cv::Size & imageSize);
};

void check_up_folder(std::string dir = "../build/calibration/");

void sample_photos_for_calibration(Camera &camera, std::string pics_dir_for_calibration);

void self_calibrate(std::string pics_dir_for_calibration, std::array<int,2> boardSize);

#endif

