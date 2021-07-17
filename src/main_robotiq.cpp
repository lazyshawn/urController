// step 1. Activation request
// {0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x30}  - clear rAct
// {0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x72, 0xE1}  - set rAct
// step 2. Read gripper status
// {0x09, 0x04, 0x07, 0xD0, 0x00, 0x01, 0x30, 0x0F}
// step 4. Close the gripper at full speed and force
// {0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x09, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x42, 0x29}
// step 5. Read gripper status until the grasp is completed
// {0x09, 0x04, 0x07, 0xD0, 0x00, 0x03, 0xB1, 0xCE}
// step 7. Open the gripper
// {0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x09, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x72, 0x19}

#include "../include/robotiq.h"
#include <array>
#include <iostream>

int scanKeyboard(void);
void print_menu(void);

int main(int argc, char** argv) {
  char device[] = "/dev/ttyUSB0";
  uint8_t rx[11];
  RobotiQ robQ(device);
  int width;

  char command;
  while(true) {
    command = scanKeyboard();
    std::cout << std::endl;
    switch(command) {
    // 打开夹爪
    case 'o': robQ.open_to_cmd(10); break;
    // 闭合夹爪
    case 'y': robQ.open_to_cmd(60); break;
    // 打印菜单
    case 'm': print_menu(); break;
    case 'p': 
      width = robQ.get_position();
      printf("width = %d", width);
      break;
    case 27: printf("end\n"); return 0; break;
    default: std::cout << "Unknow input" << std::endl; break;
    }
  }
}

/* 
 * @func  : scanKeyboard
 * @brief : 监听键盘事件
 * @param : void
 * @return: 键盘按键的 ASCII 码
 * @refes : https://www.cnblogs.com/SchrodingerDoggy/p/14072739.html
 */
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

void print_menu(void) {
  std::cout << std::endl;
  std::cout << "Y - Close" << std::endl;
  std::cout << "O - Open" << std::endl;
}


