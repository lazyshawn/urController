#include "../include/gripper_driver.h"

/*************************************************************************
 * Interface
*************************************************************************/
// 监听键盘事件
#include <stdio.h>
#include <termio.h>
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
 * @Main
*************************************************************************/
int main(int argc, char** argv) {
  int portIndex = 0;
  std::string targetIP = "10.249.181.200", cmd;
  CRGGripper crgGripper(targetIP, portIndex);
  float pos;

  char command;
  bool flag = true;
  while (flag) {
    command = scanKeyboard();
    switch (command) {
    case 'c':
      std::cout << "set position" << std::endl;
      std::cin >> pos;
      crgGripper.go(pos);
      break;
    case 'h':
      crgGripper.home();
    case 10: case 13: break;
    // Exit (e/E/ESC)
    case 27:
      flag = false;
      std::cout << "\n=== Program terminating...\n" << std::endl;
      break;
    default: std::cout << "==>> Unknow command." << std::endl; break;
    }
  } // while (threadManager.process != THREAD_EXIT)

  return 0;
}


