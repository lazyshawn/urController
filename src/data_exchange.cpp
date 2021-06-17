/* *********************************************************
 * ==>> 数据交换
 * *********************************************************/

#include "../include/data_exchange.h"

// 线程访问全局变量的互斥锁
pthread_mutex_t servoMutex = PTHREAD_MUTEX_INITIALIZER;
std::mutex config_mutex;
// Shared variable
Config config;

int flag_WriteData = OFF;
int flag_SaveData = OFF;

/* 读取全局共享变量到线程 */
SVO Config::getCopy(void) {
  std::scoped_lock guard(config_mutex);
  return data;
}
// void SvoReadFromServo(SVO *data) {
//   pthread_mutex_lock(&servoMutex);
//   *data = pSVO;
//   pthread_mutex_unlock(&servoMutex);
// }

/* 将线程更新的共享变量同步到全局 */
void Config::update(SVO* SVO_) {
  std::scoped_lock guard(config_mutex);
  data = *SVO_;
}
// void SvoWriteFromServo(SVO *data) {
//   pthread_mutex_lock(&servoMutex);
//   pSVO = *data;
//   pthread_mutex_unlock(&servoMutex);
// }

/* 手动添加关节角路劲 */
void ChangePathData(PATH *path) {
  double tmp[6];
  printf("Path frequency [1/s] = \n");
  scanf("%lf", &path->Freq);
  printf("PATH: SIN(0) 5JI(1) 3JI(2) 1JI(3) STEP(4)\n");
  printf("Path mode = \n");
  scanf("%d", &path->Mode);

  for (int i = 0; i < 6; i++) {
    printf("Angle of joint %d [deg] = \n", i + 1);
    scanf("%lf", &tmp[i]);
    path->Goal[i] = tmp[i] * Deg2Rad;
  }
}

/* 手动添加末端路劲 */
void ChangeHandData(PATH *path) {
  printf("Path frequency [1/s] = \n");
  scanf("%lf", &path->Freq);
  printf("PATH: SIN(0) 5JI(1) 3JI(2) 1JI(3) STEP(4)\n");
  printf("Path mode = ");
  scanf("%d", &path->Mode);
  double tmp[6];
  printf("Coordinates(X) of the hand[m]:\n");
  scanf("%lf", &tmp[0]);
  printf("Coordinates(Y) of the hand[m]:\n");
  scanf("%lf", &tmp[1]);
  printf("Coordinates(Z) of the hand[m]:\n");
  scanf("%lf", &tmp[2]);
  printf("Alpha of the hand[deg]:\n");
  scanf("%lf", &tmp[3]);
  printf("Beta of the hand[deg]:\n");
  scanf("%lf", &tmp[4]);
  printf("Gama(Z) of the hand[deg]:\n");
  scanf("%lf", &tmp[5]);
  for (int i = 0; i < 6; i++) {
    path->Goal[i] = i<3 ? tmp[i] : tmp[i]*Deg2Rad;
  }
}

void PosOriServo(int *posoriservoflag) { *posoriservoflag = ON; }

void SetPosOriSvo(SVO *data) {
  int ret;
  double time;
  SVO svoLocal;

  svoLocal = config.getCopy();
  svoLocal.PosOriServoFlag = data->PosOriServoFlag;
  svoLocal.Path = data->Path;

  // initTrjBuff();
  ret = PutTrjBuff(&svoLocal.Path);
  printf("ret=%d\n", ret);

  if (ret == 1) {
    printf("PathBufferPut Error\n");
  } else {
    printf("PutTrjBuff is OK\n");
    printf("Goal position[m] and axis angle[deg] of the hand:\n");
    printf("X\tY\tZ\tAlpha\tBetaY\tGama\n");
    printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n", svoLocal.Path.Goal[0],
           svoLocal.Path.Goal[1], svoLocal.Path.Goal[2],
           svoLocal.Path.Goal[3] * Rad2Deg, svoLocal.Path.Goal[4] * Rad2Deg,
           svoLocal.Path.Goal[5] * Rad2Deg);
  }
  printf("> OUT frequency <  %f [Hz]\n", svoLocal.Path.Freq);
  printf("> OUT mode <  %d\n", svoLocal.Path.Mode);

  svoLocal.ServoFlag = ON;
  svoLocal.NewPathFlag = ON;
  svoLocal.PathtailFlag = OFF;
  config.update(&svoLocal);

  ResetTime();
  time = GetCurrentTime();
  SetStartTime(time);
}

void SetJntSvo(SVO *data) {
  int ret;
  double time;
  SVO svoLocal = config.getCopy();

  svoLocal.Path = data->Path;
  svoLocal.Gain = data->Gain;

  // initTrjBuff();
  ret = PutTrjBuff(&svoLocal.Path);
  printf("ret=%d\n", ret);

  if (ret == 1) //
    printf("PathBufferPut Error\n");
  else {
    printf("PutTrjBuff is OK\n");
    printf("Goal angles < %f, %f, %f, %f, %f, %f [deg]\n",
           svoLocal.Path.Goal[0] * Rad2Deg, svoLocal.Path.Goal[1] * Rad2Deg,
           svoLocal.Path.Goal[2] * Rad2Deg, svoLocal.Path.Goal[3] * Rad2Deg,
           svoLocal.Path.Goal[4] * Rad2Deg, svoLocal.Path.Goal[5] * Rad2Deg);
    printf("> OUT frequency <  %f [Hz]\n", svoLocal.Path.Freq);
    printf("> OUT mode <  %d\n", svoLocal.Path.Mode);

    svoLocal.ServoFlag = ON;
    svoLocal.NewPathFlag = ON;
    svoLocal.PathtailFlag = OFF;
  config.update(&svoLocal);

    ResetTime();
    time = GetCurrentTime();
    SetStartTime(time);
  }
}

// 保存全局变量的数组
SVO_SAVE Exp_data[EXP_DATA_LENGTH];
int Exp_data_index = 0;

// 向Exp_data[]中加入新元素
void ExpDataSave(SVO_SAVE *data) {
  if (Exp_data_index < EXP_DATA_LENGTH) {
    Exp_data[Exp_data_index] = *data;
    Exp_data_index++;
  }
}

// 重新记录Exp_data[]
void SaveDataReset() { Exp_data_index = 0; }

// 将记录的数据写入文件
void ExpDataWrite() {
  SVO svoLocal = config.getCopy();
  std::ofstream f_curpos, f_refpos, f_curtheta, f_reftheta;
  int len = 16;

  f_curpos.open("../data/data.curpos");
  f_refpos.open("../data/data.refpos");
  f_curtheta.open("../data/data.curtheta");
  f_reftheta.open("../data/data.reftheta");

  if(!f_curpos.is_open()){printf("Open file failed. -- curpos\n");}
  if(!f_refpos.is_open()){printf("Open file failed. -- refpos\n");}
  if(!f_curtheta.is_open()){printf("Open file failed. -- theta\n");}
  if(!f_reftheta.is_open()){printf("Open file failed. -- reftheta\n");}

  printf("saving data ... \n");
  for (int i = 0; i < Exp_data_index; ++i) {
    f_curpos << std::left
      << std::setw(len) << Exp_data[i].Time
      << std::setw(len) << Exp_data[i].CurPos.t[0]
      << std::setw(len) << Exp_data[i].CurPos.t[1]
      << std::setw(len) << Exp_data[i].CurPos.t[2]
      << std::setw(len) << Exp_data[i].CurPos.t[3]*Rad2Deg
      << std::setw(len) << Exp_data[i].CurPos.t[4]*Rad2Deg
      << std::setw(len) << Exp_data[i].CurPos.t[5]*Rad2Deg
      << std::endl;

    f_refpos << std::left
      << std::setw(len) << Exp_data[i].Time
      << std::setw(len) << Exp_data[i].RefPos.t[0]
      << std::setw(len) << Exp_data[i].RefPos.t[1]
      << std::setw(len) << Exp_data[i].RefPos.t[2]
      << std::setw(len) << Exp_data[i].RefPos.t[3]*Rad2Deg
      << std::setw(len) << Exp_data[i].RefPos.t[4]*Rad2Deg
      << std::setw(len) << Exp_data[i].RefPos.t[5]*Rad2Deg
      << std::endl;

    f_curtheta << std::left
      << std::setw(len) << Exp_data[i].Time
      << std::setw(len) << Exp_data[i].CurTheta.t[0]*Rad2Deg
      << std::setw(len) << Exp_data[i].CurTheta.t[1]*Rad2Deg
      << std::setw(len) << Exp_data[i].CurTheta.t[2]*Rad2Deg
      << std::setw(len) << Exp_data[i].CurTheta.t[3]*Rad2Deg
      << std::setw(len) << Exp_data[i].CurTheta.t[4]*Rad2Deg
      << std::setw(len) << Exp_data[i].CurTheta.t[5]*Rad2Deg
      << std::endl;

    f_reftheta << std::left
      << std::setw(len) << Exp_data[i].Time
      << std::setw(len) << Exp_data[i].RefTheta.t[0]*Rad2Deg
      << std::setw(len) << Exp_data[i].RefTheta.t[1]*Rad2Deg
      << std::setw(len) << Exp_data[i].RefTheta.t[2]*Rad2Deg
      << std::setw(len) << Exp_data[i].RefTheta.t[3]*Rad2Deg
      << std::setw(len) << Exp_data[i].RefTheta.t[4]*Rad2Deg
      << std::setw(len) << Exp_data[i].RefTheta.t[5]*Rad2Deg
      << std::endl;
  }

  f_curpos.close();  f_refpos.close();
  f_curtheta.close(); f_reftheta.close();
  printf("Data saved.\n");
}

