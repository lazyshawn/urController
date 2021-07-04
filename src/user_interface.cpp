
#include "../include/user_interface.h"

void display_menu(void) {
  printf("\n**************Menu (please input [CR])*****************\n");
  printf("Cartesion planning:******[c : C]\n");
  printf("Joint planning:**********[j : J]\n");
  printf("Current Info:************[i : I]\n");
  printf("Start:*******************[s : S]\n");
  printf("Add destination:*********[p : P]\n");
  printf("End:*********************[e : E]\n");
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

