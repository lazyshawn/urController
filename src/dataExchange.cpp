/* *********************************************************
 * ==>> 数据交换
 * *********************************************************/

#include "../include/common.h"
#include "../include/adrobot_etc.h"
#include "../include/adrobot_system.h"

pthread_mutex_t servoMutex = PTHREAD_MUTEX_INITIALIZER;
// shared variable
SVO pSVO;

/* 读取全局共享变量到线程 */
void SvoReadFromServo(SVO *data) {
  pthread_mutex_lock(&servoMutex);
  *data = pSVO;
  pthread_mutex_unlock(&servoMutex);
}
/* 将线程更新的共享变量同步到全局 */
void SvoWriteFromServo(SVO *data) {
  pthread_mutex_lock(&servoMutex);
  pSVO = *data;
  pthread_mutex_unlock(&servoMutex);
}

void ChangePathData(PATH *path) {
  int i;
  double tmp[6];
  printf("\nPath frequency [1/s] = ");
  scanf("%lf", &path->Freq);
  printf("PATH: SIN(0) 5JI(1) 3JI(2) 1JI(3) STEP(4)\n");
  printf("\nPath mode = ");
  scanf("%d", &path->Mode);

  for (i = 0; i < 6; i++) {
    printf("\nAngle of joint %d [deg] = ", i + 1);
    scanf("%lf", &tmp[i]);
  }

  for (i = 0; i < 6; i++) path->Goal[i] = tmp[i] * Deg2Rad;
}

void ChangeHandData(PATH *path) {
  printf("\nPath frequency [1/s] = ");
  scanf("%lf", &path->Freq);
  printf("PATH: SIN(0) 5JI(1) 3JI(2) 1JI(3) STEP(4)\n");
  printf("Path mode = ");
  scanf("%d", &path->Mode);
  double tmp[6];
  printf("Coordinates(X) of the hand[m]:");
  scanf("%lf", &tmp[0]);
  printf("Coordinates(Y) of the hand[m]:");
  scanf("%lf", &tmp[1]);
  printf("Coordinates(Z) of the hand[m]:");
  scanf("%lf", &tmp[2]);
  printf("Alpha of the hand[deg]:");
  scanf("%lf", &tmp[3]);
  printf("Beta of the hand[deg]:");
  scanf("%lf", &tmp[4]);
  printf("Gama(Z) of the hand[deg]:");
  scanf("%lf", &tmp[5]);
  for (int i = 0; i < 6; i++) {
    if (i < 3)
      path->Goal[i] = tmp[i];
    else
      path->Goal[i] = tmp[i] * Deg2Rad;
  }
}
void PosOriServo(int *posoriservoflag) { *posoriservoflag = ON; }

void SetPosOriSvo(SVO *data) {
  int ret;
  double time;

  pSVO.PosOriServoFlag = data->PosOriServoFlag;
  pSVO.Path = data->Path;

  initTrjBuff();
  ret = PutTrjBuff(&pSVO.Path);

  printf("ret=%d\n", ret);

  if (ret == 1) //
    printf("PathBufferPut Error\n");
  else {
    printf("PutTrjBuff is OK\n");
    printf("Goal position[m] and axis angle[deg] of the hand:\n");
    printf("X\tY\tZ\tAlpha\tBetaY\tGama\n");
    printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n", pSVO.Path.Goal[0],
           pSVO.Path.Goal[1], pSVO.Path.Goal[2],
           pSVO.Path.Goal[3] * Rad2Deg, pSVO.Path.Goal[4] * Rad2Deg,
           pSVO.Path.Goal[5] * Rad2Deg);
  }
  printf("> OUT frequency <  %f [Hz]\n", pSVO.Path.Freq);
  printf("> OUT mode <  %d\n", pSVO.Path.Mode);

  pSVO.ServoFlag = ON;
  pSVO.NewPathFlag = ON;
  pSVO.PathtailFlag = OFF;

  ResetTime();
  time = GetCurrentTime();
  SetStartTime(time);
}

void SetJntSvo(SVO *data) {
  int ret;
  double time;

  pSVO.Path = data->Path;
  pSVO.Gain = data->Gain;

  initTrjBuff();

  ret = PutTrjBuff(&pSVO.Path);

  printf("ret=%d\n", ret);

  if (ret == 1) //
    printf("PathBufferPut Error\n");
  else {
    printf("PutTrjBuff is OK\n");
    printf("Goal angles < %f, %f, %f, %f, %f, %f [deg]\n",
           pSVO.Path.Goal[0] * Rad2Deg, pSVO.Path.Goal[1] * Rad2Deg,
           pSVO.Path.Goal[2] * Rad2Deg, pSVO.Path.Goal[3] * Rad2Deg,
           pSVO.Path.Goal[4] * Rad2Deg, pSVO.Path.Goal[5] * Rad2Deg);
    printf("> OUT frequency <  %f [Hz]\n", pSVO.Path.Freq);
    printf("> OUT mode <  %d\n", pSVO.Path.Mode);

    pSVO.ServoFlag = ON;
    pSVO.NewPathFlag = ON;
    pSVO.PathtailFlag = OFF;

    ResetTime();
    time = GetCurrentTime();
    SetStartTime(time);
  }
}

