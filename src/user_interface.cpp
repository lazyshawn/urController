
#include "../include/user_interface.h"

void DisplayMenu(void) {
  printf("\n**************Menu (please input [CR])*****************\n");
  printf("CurPosOfHnd:*************[c : C]\n");
  printf("Gain:********************[g : G]\n");
  printf("Path:********************[p : P]\n");
  printf("Start:*******************[s : S]\n");
  printf("Info:********************[i : I]\n");
  printf("Destination:*************[d : D]\n");
  printf("End:*********************[e : E :ESC]\n");
  printf("*******************************************************\n");
}

void DisplayCurrentInformation(SVO svoLocal) {
  printf("--------------------- Current Information -----------------------\n");
  printf("Path frequency = %f [Hz]\n", svoLocal.Path.Freq);
  switch (svoLocal.Path.Mode) {
  case 0: printf("Path mode: Sin\n"); break;
  case 1: printf("Path mode: 5JI\n"); break;
  case 2: printf("Path mode: 3JI\n"); break;
  case 3: printf("Path mode: 1JI\n"); break;
  case 4: printf("Path mode: Step\n"); break;
  default: printf("Error path mode\n");
  }
  if (svoLocal.PosOriServoFlag == OFF) {
    printf("Current Joint angle[deg]:%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
           svoLocal.CurTheta[0] * Rad2Deg, svoLocal.CurTheta[1] * Rad2Deg,
           svoLocal.CurTheta[2] * Rad2Deg, svoLocal.CurTheta[3] * Rad2Deg,
           svoLocal.CurTheta[4] * Rad2Deg, svoLocal.CurTheta[5] * Rad2Deg);

    printf("Goal  Joint  Angle [Deg]:%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
           svoLocal.Path.Goal[0] * Rad2Deg, svoLocal.Path.Goal[1] * Rad2Deg,
           svoLocal.Path.Goal[2] * Rad2Deg, svoLocal.Path.Goal[3] * Rad2Deg,
           svoLocal.Path.Goal[4] * Rad2Deg, svoLocal.Path.Goal[5] * Rad2Deg);
    printf("---------------------------------------------------------------\n");
  } else {
    printf("Current Position of "
           "hand:\nX[m]\tY[m]\tZ[m]\tAlpha[deg]\tBeta[deg]\tGama[deg]\n");
    printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
           svoLocal.CurPos[0], svoLocal.CurPos[1], svoLocal.CurPos[2],
           svoLocal.CurPos[3] * Rad2Deg, svoLocal.CurPos[4] * Rad2Deg,
           svoLocal.CurPos[5] * Rad2Deg);

    printf("Goal Position of  hand:"
           "\nX[m]\tY[m]\tZ[m]\tAlpha[deg]\tBeta[deg]\tGama[deg]\n");
    printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n",
           svoLocal.Path.Goal[0], svoLocal.Path.Goal[1], svoLocal.Path.Goal[2],
           svoLocal.Path.Goal[3] * Rad2Deg, svoLocal.Path.Goal[4] * Rad2Deg,
           svoLocal.Path.Goal[5] * Rad2Deg);
    printf("---------------------------------------------------------------\n");
  }
}

