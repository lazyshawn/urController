#include "../include/camera_driver.h"

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
  pipe.stop();
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

/* 
 * @func  : check_up_folder
 * @brief : 清理指定文件夹; 若文件夹不存在则创建，否则清空
 * @param : std::string pics_dir_for_calibration - 文件夹路径
 * @return: 
 */
void check_up_folder (std::string dir) {
  /* Checking whether path exists */
  // std::string dir = "../build/calibration/";
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

// 打开棋盘格图像，并提取角点
int CameraCalibrator::add_Chessboard_Points(const std::string picsDir, 
    cv::Size& boardSize){
  // Creating vector to store vectors of 3D points for each checkerboard image
  std::vector<std::vector<cv::Point3f>> objpoints;
  // Creating vector to store vectors of 2D points for each checkerboard image
  std::vector<std::vector<cv::Point2f>> imgpoints;
  // Defining the world coordinates for 3D points
  std::vector<cv::Point3f> objp;
  for (int i{0}; i < boardSize.height; i++) {
    for (int j{0}; j < boardSize.width; j++)
      objp.push_back(cv::Point3f(j, i, 0));
  }

  // Extracting path of individual image stored in a given directory
  std::vector<cv::String> images;
  // Path of the folder containing checkerboard images
  std::string path = picsDir + "*.jpg";
  cv::glob(path, images);

  cv::Mat frame, gray;
  // vector to store the pixel coordinates of detected checkerboard corners
  std::vector<cv::Point2f> corner_pts;
  bool success;

  // Looping over all the images in the directory
  for (int i{0}; i < images.size(); i++) {
    frame = cv::imread(images[i]);
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    // Finding checker board corners
    // If desired number of corners are found in the image then success = true
    success = cv::findChessboardCorners(gray, boardSize, corner_pts,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
        cv::CALIB_CB_FILTER_QUADS + cv::CALIB_CB_FAST_CHECK);
    /*
     * If desired number of corner are detected, we refine the pixel coordinates
     * and display them on the images of checker board
     */
    if (success) {
      cv::TermCriteria criteria(
          cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
      // refining pixel coordinates for given 2d points.
      cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1),
                       criteria);
      // Displaying the detected corner points on the checker board
      cv::drawChessboardCorners(frame, boardSize, corner_pts, success);
      objpoints.push_back(objp);
      imgpoints.push_back(corner_pts);
    }
    cv::imshow("Image", frame);
    cv::waitKey(0);
  }
  cv::destroyAllWindows();

  /*
   * Performing camera calibration by passing the value of known
   * 3D points (objpoints) and corresponding pixel coordinates of
   * the detected corners (imgpoints)
   */
  cv::Mat cameraMatrix, distCoeffs, R, T;
  double projection = cv::calibrateCamera(objpoints, imgpoints, boardSize, 
      cameraMatrix, distCoeffs, rvecs, T);

  // 保存内参矩阵
  cv::FileStorage fs("../build/calibration/calibration.ymal",
      cv::FileStorage::READ | cv::FileStorage::WRITE);
  fs << "cameraMatrix" << cameraMatrix;
  fs.release();

  std::cout << "cameraMatrix :\n" << cameraMatrix << std::endl;
  std::cout << "distCoeffs :\n" << distCoeffs << std::endl;
  // std::cout << "Rotation vector :\n" << R << std::endl;
  std::cout << "Translation vector :\n" << T << std::endl;

  cv::Mat rotMatrix, intrinsicMatrix;
  Rodrigues(rvecs[0], rotMatrix);
  hconcat(rotMatrix, tvecs[0], intrinsicMatrix);
  std::cout << "intrinsicMatrix :\n" << intrinsicMatrix << std::endl;

  return success;
}

// 校准相机，返回重投影精度
double CameraCalibrator::calibrate(cv::Size & imageSize){
  double reprojectionError;
  return reprojectionError;
}

void sample_photos_for_calibration(Camera &camera, std::string pics_dir_for_calibration) {
  cv::Mat sampleFrame;
  char fileName[20];
  bool keepSample = true;
  int counter = 0;
  while (keepSample) {
    sampleFrame = camera.get_color_frame();
    imshow("Display color Image", sampleFrame);
    // Wait command
    switch (cv::waitKey(10)) {
    case 'q': keepSample = false; break;
    // Press 'Space' or 'Enter' to sample
    case 10: case 13:
      counter++;
      std::cout << "Get pics No." << counter << std::endl;
      sprintf(fileName, "pic_%03d.jpg", counter);
      imwrite(pics_dir_for_calibration+fileName, sampleFrame);
      break;
    default: break;
    }
  }
  cv::destroyAllWindows();
}

