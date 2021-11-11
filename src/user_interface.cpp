#include "../include/user_interface.h"

extern Config config;
extern ThreadManager threadManager;
extern PathQueue pathQueue;

/*************************************************************************
 * @func  : interface_thread_function
 * @brief : GUI界面_处理人机交互
*************************************************************************/
void interface_thread_function(void) {
  SVO svoLocal;
  char command;
  std::thread sensor_thread, camera_thread;

  display_menu();
  while (threadManager.process != THREAD_EXIT) {
    // Wait command
    command = scanKeyboard();
    svoLocal = config.getCopy();
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
    case 'r': case 'R': teleoperate_robot(); break;
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
    config.update(&svoLocal);
  } // while (threadManager.process != THREAD_EXIT)
  if (threadManager.device.sensor == true) {
    sensor_thread.join();
  }
  if (threadManager.device.camera == true) {
    camera_thread.join();
  }
  std::cout << "Program end: display_function." << std::endl;
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
 * @param : svo
 * @return: 下一插补点处的 关节角/位姿
*************************************************************************/
void display_current_information(SVO svo) {
  printf("\n--------------------- Current Information -----------------------\n");
  printf("==>> path frequency = %f [Hz]\n", svo.path.freq);
  switch (svo.path.interpMode) {
  case 0: printf("==>> path mode: Sin\n"); break;
  case 1: printf("==>> path mode: 5JI\n"); break;
  case 2: printf("==>> path mode: 3JI\n"); break;
  case 3: printf("==>> path mode: 1JI\n"); break;
  case 4: printf("==>> path mode: Step\n"); break;
  default: printf("Error path mode\n");
  }
  printf("==>> Current Joint angle[deg]:\n%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
       svo.curTheta[0] * Rad2Deg, svo.curTheta[1] * Rad2Deg,
       svo.curTheta[2] * Rad2Deg, svo.curTheta[3] * Rad2Deg,
       svo.curTheta[4] * Rad2Deg, svo.curTheta[5] * Rad2Deg);

  printf("==>> Goal  Joint  Angle [Deg]:\n%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
       svo.path.goal[0] * Rad2Deg, svo.path.goal[1] * Rad2Deg,
       svo.path.goal[2] * Rad2Deg, svo.path.goal[3] * Rad2Deg,
       svo.path.goal[4] * Rad2Deg, svo.path.goal[5] * Rad2Deg);
  printf("==>> Current Position of hand:\n"
         "X[mm]\tY[mm]\tZ[mm]\tAlpha[deg]\tBeta[deg]\tGama[deg]\n");
  printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
       svo.curPos[0], svo.curPos[1], svo.curPos[2],
       svo.curPos[3]*Rad2Deg, svo.curPos[4]*Rad2Deg, svo.curPos[5]*Rad2Deg);

  printf("==>> Goal Displacement of hand:\n"
         "X[mm]\tY[mm]\tZ[mm]\tAlpha[deg]\tBeta[deg]\tGama[deg]\n");
  printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
      svo.path.velocity[0], svo.path.velocity[1], svo.path.velocity[2],
      svo.path.velocity[3], svo.path.velocity[4], svo.path.velocity[5]);
  printf("---------------------------------------------------------------\n\n");
}


/*************************************************************************
 * @func: teleoperate_robot;
 * @breif: 机械臂键盘遥操作
*************************************************************************/
void teleoperate_robot(void) {
  SVO svoLocal = config.getCopy();
  NUMBUF inputData;
  PATH pathLocal;
  char command;
  double increTime = 0.2, tiltAngle;
  TRIARR state, circleCmd;

  std::cout << 
    "*******************************************************\n" <<
    "=== Remote control of the robot " <<
    "*******************************************************" << std::endl;
  while (command != 27) {
    // 输入数组置零
    for (int i=0; i<get_array_size(inputData); ++i) inputData[i] = 0;
    inputData[8] = -1;
    // Wait command
    command = scanKeyboard();

    switch(command) {
    // add destination in Cartesion space (c/C)
    case 'x': case 'X':
      read_joint_destination(inputData);
      add_joint_destination(pathLocal, inputData);
      break;
    // add destination in Joint space (x/X)
    case 'c': case 'C':
      read_cartesion_destination(inputData);
      add_cartesion_destination(pathLocal, inputData, svoLocal.curTheta);
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
    case 'g': case 'G': robot_go_home(svoLocal.curTheta); break;
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
    case 'p': case 'P': display_current_information(svoLocal); break;
    // Test
    case 't': 
      tiltAngle = -M_PI/2 - svoLocal.curTheta[1] - svoLocal.curTheta[2]
        - svoLocal.curTheta[3];
      state = {svoLocal.curPos[0], svoLocal.curPos[2], tiltAngle};
      circleCmd = {500, 20, 20*Deg2Rad};
      pivot_about_points(state, circleCmd, 3);
      break;
    // Invalid command
    default: std::cout << "==>> Unknow command." << std::endl; break;
    }
    config.update(&svoLocal);
  } // while(command != 27)
  display_menu();
}

