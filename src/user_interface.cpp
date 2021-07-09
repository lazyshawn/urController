
#include "../include/user_interface.h"

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

/* 
 * @func  : display_current_information
 * @brief : 打印当前时刻系统的状态信息
 * @param : svo
 * @return: 下一插补点处的 关节角/位姿
 */
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

