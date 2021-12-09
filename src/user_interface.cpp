#include "../include/user_interface.h"

extern urConfig urconfig;
extern ThreadManager threadManager;
extern PathQueue pathQueue;

/*************************************************************************
 * @func  : interface_thread_function
 * @brief : GUI界面_处理人机交互
*************************************************************************/
void interface_thread_function(void) {
  urConfig::Data urConfigData;
  char command;
  std::thread robot_thread, sensor_thread, camera_thread;

  display_menu();
  while (threadManager.process != THREAD_EXIT) {
    // Wait command
    command = scanKeyboard();
    // Run command
    switch (command) {
    // Start camera
    case 'c': 
      camera_thread = std::thread(camera_thread_function);
      threadManager.device.camera = true;
      break;
    // Start force sensor
    case 'f': case 'F': 
      sensor_thread = std::thread(sensor_thread_function);
      threadManager.device.sensor = true;
      break;
    // Show menu
    case 'm': case 'M': display_menu(); break;
    // Next shoot(N). Set for debug.
    case 'n': case 'N': pathQueue.notify_one(); break;
    // Robot.
    case 'r': case 'R': 
      if(threadManager.device.robot == OFF) { // 避免重复启动
        robot_thread = std::thread(robot_thread_function);
        threadManager.device.robot = ON;
      }
      teleoperate_robot();
      threadManager.device.robot = IDLE;  // 机械臂线程挂起
      break;
    case 's': case 'S':
      threadManager.process = THREAD_SETUP;
      // 机械臂
      if(threadManager.device.robot == OFF) { // 避免重复启动
        robot_thread = std::thread(robot_thread_function);
        threadManager.device.robot = IDLE;  // 机械臂线程挂起
      }
      // 相机和夹爪
      camera_thread = std::thread(camera_thread_function);
      threadManager.device.camera = true;
      sleep(2);  // 等待各线程初始化完毕
      std::cout <<  "*******************************************************\n" 
        << std::endl;
      threadManager.process = THREAD_INIT_GRASP;
      break;
    // 回车和换行
    case 10: case 13: break;
    // Exit (e/E/ESC)
    case 27: 
      threadManager.process = THREAD_EXIT;
      std::cout << "\n=== Program terminating...\n" << std::endl;
      break;
    // Invalid command
    default: std::cout << "==>> Unknow command." << std::endl; break;
    }
  } // while (threadManager.process != THREAD_EXIT)
  
  // Wait for threads
  if (threadManager.device.robot != OFF) {
    robot_thread.join();
  }
  if (threadManager.device.sensor != OFF) {
    sensor_thread.join();
  }
  if (threadManager.device.camera != OFF) {
    camera_thread.join();
  }
  std::cout << "Thread terminated: user_interface_thread" << std::endl;
} // void interface(void)


/*************************************************************************
 * @func  : scanKeyboard
 * @brief : 监听键盘事件
 * @param : void
 * @return: 键盘按键的 ASCII 码
 * @refes : https://www.cnblogs.com/SchrodingerDoggy/p/14072739.html
*************************************************************************/
int scanKeyboard() {
  // 通过tcsetattr函数设置terminal的属性来控制需不需要回车来结束输入
  struct termios new_settings;
  struct termios stored_settings;
  // 备份 termios 设置
  tcgetattr(0, &stored_settings);
  new_settings = stored_settings;
  new_settings.c_lflag &= (~ICANON);
  new_settings.c_cc[VTIME] = 0;
  tcgetattr(0, &stored_settings);
  new_settings.c_cc[VMIN] = 1;
  tcsetattr(0, TCSANOW, &new_settings);
  // 监听键盘事件
  int input = getchar();
  // 还原设置
  tcsetattr(0, TCSANOW, &stored_settings);
  return input;
}


/*************************************************************************
 * @func : display_menu;
 * @breif: 控制界面菜单
*************************************************************************/
void display_menu(void) {
  printf("\n**************Menu (please input [CR])*****************\n");
  printf("Cartesion space planning:******[c : C]\n");
  printf("Joint space planning:**********[x : x]\n");
  printf("Velocity command:**************[v : V]\n");
  printf("Start:*************************[s : S]\n");
  printf("Print state info:**************[p : P]\n");
  printf("Menu:**************************[m : M]\n");
  printf("Next shoot:********************[n : N]\n");
  printf("-------------- Navigation ------------\n");
  printf("Left/Down/Up/Right*************[hj kl]\n");
  printf("Tilt clockwise/conterclock*****[u / i]\n");
  printf("End:***************************[ ESC ]\n");
  printf("*******************************************************\n");
}


/*************************************************************************
 * @func  : display_current_information
 * @brief : 打印当前时刻系统的状态信息
*************************************************************************/
void display_current_information(urConfig::Data urConfigData) {
  TWIST twist;
  for (int i=0; i<3; ++i) {
    twist[i] = urConfigData.tranMat(i,3);
  }
  printf("\n--------------------- Current Information -----------------------\n");
  printf("==>> path frequency = %f [Hz]\n", urConfigData.path.freq);
  switch (urConfigData.path.interpMode) {
  case 0: printf("==>> path mode: Sin\n"); break;
  case 1: printf("==>> path mode: 5JI\n"); break;
  case 2: printf("==>> path mode: 3JI\n"); break;
  case 3: printf("==>> path mode: 1JI\n"); break;
  case 4: printf("==>> path mode: Step\n"); break;
  default: printf("Error path mode\n");
  }
  std::cout << urConfigData.tranMat << std::endl;
  printf("==>> Current Joint angle[deg]:\n%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
       urConfigData.curTheta[0] * rad2deg, urConfigData.curTheta[1] * rad2deg,
       urConfigData.curTheta[2] * rad2deg, urConfigData.curTheta[3] * rad2deg,
       urConfigData.curTheta[4] * rad2deg, urConfigData.curTheta[5] * rad2deg);

  printf("==>> Goal  Joint  Angle [Deg]:\n%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
       urConfigData.path.goal[0] * rad2deg, urConfigData.path.goal[1] * rad2deg,
       urConfigData.path.goal[2] * rad2deg, urConfigData.path.goal[3] * rad2deg,
       urConfigData.path.goal[4] * rad2deg, urConfigData.path.goal[5] * rad2deg);
  printf("==>> Current Position of hand:\n"
         "X[mm]\tY[mm]\tZ[mm]\tAlpha[deg]\tBeta[deg]\tGama[deg]\n");
  printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
       twist[0], twist[1], twist[2],
       twist[3]*rad2deg, twist[4]*rad2deg, twist[5]*rad2deg);

  printf("==>> Goal Displacement of hand:\n"
         "X[mm]\tY[mm]\tZ[mm]\tAlpha[deg]\tBeta[deg]\tGama[deg]\n");
  printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
      urConfigData.path.velocity[0], urConfigData.path.velocity[1], urConfigData.path.velocity[2],
      urConfigData.path.velocity[3], urConfigData.path.velocity[4], urConfigData.path.velocity[5]);
  printf("---------------------------------------------------------------\n\n");
}


/*************************************************************************
 * @func: teleoperate_robot;
 * @breif: 机械臂键盘遥操作
*************************************************************************/
void teleoperate_robot(void) {
  urConfig::Data urConfigData;
  NUMBUF inputData;
  PATH pathLocal;
  char command = 0;
  double increTime = 0.2, tiltAngle;
  TRIARR state, circleCmd;
  bool process = true;

  std::cout << 
    "\n*******************************************************\n" <<
    "=== Remote control of the robot \n" <<
    "*******************************************************" << std::endl;
  while (process) {
    // 输入数组置零
    for (int i=0; i<8; ++i) inputData[i] = 0;
    inputData[8] = -1;
    // Wait command
    command = scanKeyboard();
    urConfigData = urconfig.get_data();

    switch(command) {
    // add destination in Cartesion space (c/C)
    case 'x': case 'X':
      read_joint_destination(inputData);
      add_joint_destination(pathLocal, inputData);
      break;
    // add destination in Joint space (x/X)
    case 'c': case 'C':
      read_cartesion_destination(inputData);
      add_cartesion_destination(pathLocal, inputData, urConfigData.curTheta);
      break;
    // add velocity command (v/V)
    case 'v': case 'V':
      read_displacement(inputData);
      add_displacement(pathLocal, inputData);
      break;
    // start (s/S)
    case 's': case 'S': pathQueue.push(pathLocal); break;
    /* *** Navigation *** */
    // go home (g/G)
    case 'g': case 'G': robot_go_home(urConfigData.curTheta); break;
    // move foward -X (h/H)
    case 'h': case 'H':
      inputData[6] = increTime; inputData[0] = -1;
      add_displacement(pathLocal, inputData);
      break;
    // move foward -Z (j/J)
    case 'j': case 'J':
      inputData[6] = increTime; inputData[2] = -1;
      add_displacement(pathLocal, inputData);
      break;
    // move foward +Z (k/K)
    case 'k': case 'K':
      inputData[6] = increTime; inputData[2] = 1;
      add_displacement(pathLocal, inputData);
      break;
    // move foward +X (l/L)
    case 'l': case 'L':
      inputData[6] = increTime; inputData[0] = 1;
      add_displacement(pathLocal, inputData);
      break;
    // move foward +X (l/L)
    case 'u': case 'U':
      inputData[6] = increTime; inputData[4] = 1;
      add_displacement(pathLocal, inputData);
      break;
    // move foward +X (l/L)
    case 'i': case 'I':
      inputData[6] = increTime; inputData[4] = -1;
      add_displacement(pathLocal, inputData);
      break;
    // close robotiQ (y/Y)
    case 'y': case 'Y':
      pathLocal.angleServo = OFF;
      inputData[6] = increTime;
      inputData[8] = 80;
      add_displacement(pathLocal, inputData);
      break;
    // open robotiQ (o/O)
    case 'o': case 'O':
      pathLocal.angleServo = OFF;
      inputData[6] = increTime;
      inputData[8] = 2;
      add_displacement(pathLocal, inputData);
      break;
    // Show the information of robot
    case 'p': case 'P': display_current_information(urConfigData); break;
    // Test
    case 't': 
      tiltAngle = -M_PI/2 - urConfigData.curTheta[1] - urConfigData.curTheta[2]
        - urConfigData.curTheta[3];
      // state = {urConfigData.curTwist[0], urConfigData.curTwist[2], tiltAngle};
      circleCmd = {500, 20, 20*Deg2Rad};
      pivot_about_points(state, circleCmd, 3);
      break;
    case 27:
      process = false;
      std::cout << "=== Back to main manue. \n" << std::endl;
      break;
    // Invalid command
    default: std::cout << "==>> Unknow command." << std::endl; break;
    }
    urconfig.update(&urConfigData);
  } // while(command != 27)
  display_menu();
} // void teleoperate_robot(void)

