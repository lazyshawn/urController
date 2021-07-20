#include "../include/camera.h"

// 构造函数
Camera::Camera(){
  // Create a configuration for configuring the pipeline
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

  // 启动设备的管道配置文件, 开始传送数据流
  selection = pipe.start(cfg);
}

// 析构函数
Camera::~Camera(){
}

// 获取一帧彩色图像
cv::Mat Camera::get_color_frame() {
  frames = pipe.wait_for_frames(); // 等待下一帧
  color_frame = frames.get_color_frame();
  // 创建Opencv类,并传入数据
  cv::Mat color(cv::Size(640, 480), CV_8UC3, (void *)color_frame.get_data(),
            cv::Mat::AUTO_STEP);
  return color;
}

// 获取一帧深度图像
cv::Mat Camera::get_depth_frame() {
  frames = pipe.wait_for_frames(); // 等待下一帧
  // 获取深度图, 加颜色滤镜
  depth_frame = frames.get_depth_frame().apply_filter(color_map);
  // 创建Opencv类,并传入数据
  cv::Mat depth(cv::Size(640, 480), CV_8UC3, (void *)depth_frame.get_data(),
            cv::Mat::AUTO_STEP);
  return depth;
}

// 生成一个视频记录对象
cv::VideoWriter Camera::create_recorder() {
  // cv::VideoWriter outputVideo("test.avi",
  //     cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(640, 480));
  cv::VideoWriter outputVideo;
  cv::String videoSavePath = "../build/video.avi";
  int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  double fps = 30;
  cv::Size size = cv::Size(640, 480);
  
  outputVideo.open(videoSavePath, fourcc, fps, size);
  if (!outputVideo.isOpened()) {
    std::cout << "fail to open!" << std::endl;
  }
  return outputVideo;
}

