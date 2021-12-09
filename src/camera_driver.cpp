#include "../include/camera_driver.h"

/*************************************************************************
 * @class: Camera
*************************************************************************/
/* **************** 构造函数 **************** */
Camera::Camera(){
  // Create librealsense context for managing devices
  rs2::context ctx;
  auto &&dev = ctx.query_devices();
  std::string serialNum =  dev[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  std::cout << "\nSerial num is not specifiled. Use the first one" << std::endl;
  for (int i=0; i<dev.size(); ++i) {
      std::cout << dev[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
  }
  new(this) Camera(serialNum);
}

// 按序列号激活相机
Camera::Camera(std::string serialNum){
  cfg.enable_device(serialNum);
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

  // 启动设备的管道配置文件, 开始传送数据流
  selection = pipe.start(cfg);

  // 读取内参
  auto stream = selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
  intr = stream.get_intrinsics(); // Calibration data

  // 初始化默认外参
  extrMat << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  set_extrMat(extrMat);
}

/* **************** 析构函数 **************** */
Camera::~Camera(){
  pipe.stop();
}

/* **************** 获取一帧彩色图像 **************** */
cv::Mat Camera::get_color_frame() {
  frames = pipe.wait_for_frames(); // 等待下一帧
  color_frame = frames.get_color_frame();
  // 创建Opencv类,并传入数据
  cv::Mat color(cv::Size(640, 480), CV_8UC3, (void *)color_frame.get_data(),
            cv::Mat::AUTO_STEP);
  return color;
}

/* **************** 获取一帧深度图像 **************** */
cv::Mat Camera::get_depth_frame() {
  frames = pipe.wait_for_frames(); // 等待下一帧
  // 获取深度图, 加颜色滤镜
  depth_frame = frames.get_depth_frame().apply_filter(color_map);
  // 创建Opencv类,并传入数据
  cv::Mat depth(cv::Size(640, 480), CV_8UC3, (void *)depth_frame.get_data(),
            cv::Mat::AUTO_STEP);
  return depth;
}

/* **************** 设定外参 **************** */
void Camera::set_extrMat(Eigen::Matrix<double,4,4> tranMat) {
  extrMat = tranMat;
}
/* **************** 获取外参 **************** */
Eigen::Matrix<double,4,4> Camera::get_extrMat() {
  return extrMat;
}

/* **************** 生成一个视频记录对象 **************** */
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

/* **************** 获取像素点在相机坐标系下的 3D 坐标 **************** */
void Camera::pixel_to_point(float* point3d, float* pixel, float depth) {
  rs2_deproject_pixel_to_point(point3d, &intr, pixel, depth);
}

/* **************** 检测 Marker 位姿 **************** */
// Ref: https://blog.csdn.net/qq_33446100/article/details/89115983
int Camera::detect_marker(cv::Mat frame, int id, float depth, Eigen::Matrix<double,4,4>& markerPose) {
  cv::Mat frame_show;
  Eigen::Matrix<double,4,4> T_obj2cam = Eigen::Matrix<double,4,4>::Zero();
  // 创建字典，这里注意使用Ptr<>，不然无法显示结果
  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> marker;
  float origPixel[2], cornPixel[4][2], orig[3], corn[4][3], dx, dy, ds;

  // 检测该帧是否有标记
  // @param: 待检测的图像, 预先定义的字典对象, 检测出的图像的角点的列表,
  //         检测出的所有maker的ID列表
  cv::aruco::detectMarkers(frame, dictionary, marker, ids);

  // 只处理检测到的第一个 Marker
  if (ids.size() == 0) return 0;
  for (int i=0; i<ids.size(); ++i) {
    if (ids[i] == id) break;
    if (i == ids.size()-1) return 0;
  }

  frame.copyTo(frame_show); //复制一份
  // 如果有，则标记出来，放入另一个Mat
  // cv::aruco::drawDetectedMarkers(frame_show, marker, ids);
  // imshow("detected", frame_show);
  // 角点的空间坐标
  for (int i=0; i<3; ++i) {
    cornPixel[i][0] = marker[0][i].x;
    cornPixel[i][1] = marker[0][i].y;
    pixel_to_point(corn[i], cornPixel[i], depth);
  }
  origPixel[0] = (marker[0][0].x + marker[0][2].x) / 2;
  origPixel[1] = (marker[0][0].y + marker[0][2].y) / 2;
  pixel_to_point(orig, origPixel, depth);
  // 横坐标 x (P0 - P1)
  dx = corn[0][0] - corn[1][0];
  dy = corn[0][1] - corn[1][1];
  ds = sqrt(dx*dx + dy*dy);
  T_obj2cam(0,0) = dx/ds;
  T_obj2cam(1,0) = dy/ds;
  // 纵坐标 y
  dx = corn[1][0] - corn[2][0];
  dy = corn[1][1] - corn[2][1];
  ds = sqrt(dx*dx + dy*dy);
  T_obj2cam(0,1) = dx/ds;
  T_obj2cam(1,1) = dy/ds;
  T_obj2cam(0,3) = orig[0];
  T_obj2cam(1,3) = orig[1];
  T_obj2cam(2,3) = orig[2];
  T_obj2cam(2,2) = T_obj2cam(3,3) = 1;
  
  markerPose = extrMat*T_obj2cam;
  // std::cout << T_obj2cam << std::endl;
  return 1;
}

/* **************** 清理指定文件夹(创建/清空) **************** */
void Camera::check_up_folder () {
  /* Checking whether path exists */
  std::string dir = calibrationDir;
  std::fstream file;
  file.open(dir, std::ios::in);
  if (!file) {
    std::cout << dir << " don't exist." << std::endl;
    system(("mkdir -p " + dir).c_str());
  } else {
    std::cout << dir << " already exist." << std::endl;
    system(("rm -rf " + dir + "*.jpg").c_str());
  }
}

