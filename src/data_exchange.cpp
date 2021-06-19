/* *********************************************************
 * ==>> 数据交换
 * *********************************************************/

#include "../include/data_exchange.h"

// Shared variable
Config config;

int flag_WriteData = OFF;
int flag_SaveData = OFF;

/* 读取全局共享变量到线程 */
SVO Config::getCopy(void) {
  std::scoped_lock guard(config_mutex);
  return data;
}

/* 将线程更新的共享变量同步到全局 */
void Config::update(SVO* SVO_) {
  std::scoped_lock guard(config_mutex);
  data = *SVO_;
}

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
  data->PosOriServoFlag = data->PosOriServoFlag;
  data->Path = data->Path;

  // initTrjBuff();
  int ret = PutTrjBuff(&data->Path);
  printf("ret=%d\n", ret);

  if (ret == 1) {
    printf("PathBufferPut Error\n");
  } else {
    printf("PutTrjBuff is OK\n");
    printf("Goal position[m] and axis angle[deg] of the hand:\n");
    printf("X\tY\tZ\tAlpha\tBetaY\tGama\n");
    printf("%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\t\t%.2f\n", data->Path.Goal[0],
           data->Path.Goal[1], data->Path.Goal[2],
           data->Path.Goal[3] * Rad2Deg, data->Path.Goal[4] * Rad2Deg,
           data->Path.Goal[5] * Rad2Deg);
  }
  printf("> OUT frequency <  %f [Hz]\n", data->Path.Freq);
  printf("> OUT mode <  %d\n", data->Path.Mode);

  data->ServoFlag = ON;
  data->NewPathFlag = ON;
  data->PathtailFlag = OFF;

  ResetTime();
  SetStartTime(GetCurrentTime());
}

void SetJntSvo(SVO *data) {
  data->Path = data->Path;
  data->Gain = data->Gain;

  // initTrjBuff();
  int ret = PutTrjBuff(&data->Path);
  printf("ret=%d\n", ret);

  if (ret == 1) //
    printf("PathBufferPut Error\n");
  else {
    printf("PutTrjBuff is OK\n");
    printf("Goal angles < %f, %f, %f, %f, %f, %f [deg]\n",
           data->Path.Goal[0] * Rad2Deg, data->Path.Goal[1] * Rad2Deg,
           data->Path.Goal[2] * Rad2Deg, data->Path.Goal[3] * Rad2Deg,
           data->Path.Goal[4] * Rad2Deg, data->Path.Goal[5] * Rad2Deg);
    printf("> OUT frequency <  %f [Hz]\n", data->Path.Freq);
    printf("> OUT mode <  %d\n", data->Path.Mode);

    data->ServoFlag = ON;
    data->NewPathFlag = ON;
    data->PathtailFlag = OFF;

    ResetTime();
    SetStartTime(GetCurrentTime());
  }
}

// 保存全局变量的数组
SVO Exp_data[EXP_DATA_LENGTH];
int Exp_data_index = 0;

// 向Exp_data[]中加入新元素
void ExpDataSave(SVO *data) {
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
      << std::setw(len) << Exp_data[i].CurPos[0]
      << std::setw(len) << Exp_data[i].CurPos[1]
      << std::setw(len) << Exp_data[i].CurPos[2]
      << std::setw(len) << Exp_data[i].CurPos[3]*Rad2Deg
      << std::setw(len) << Exp_data[i].CurPos[4]*Rad2Deg
      << std::setw(len) << Exp_data[i].CurPos[5]*Rad2Deg
      << std::endl;

    f_refpos << std::left
      << std::setw(len) << Exp_data[i].Time
      << std::setw(len) << Exp_data[i].RefPos[0]
      << std::setw(len) << Exp_data[i].RefPos[1]
      << std::setw(len) << Exp_data[i].RefPos[2]
      << std::setw(len) << Exp_data[i].RefPos[3]*Rad2Deg
      << std::setw(len) << Exp_data[i].RefPos[4]*Rad2Deg
      << std::setw(len) << Exp_data[i].RefPos[5]*Rad2Deg
      << std::endl;

    f_curtheta << std::left
      << std::setw(len) << Exp_data[i].Time
      << std::setw(len) << Exp_data[i].CurTheta[0]*Rad2Deg
      << std::setw(len) << Exp_data[i].CurTheta[1]*Rad2Deg
      << std::setw(len) << Exp_data[i].CurTheta[2]*Rad2Deg
      << std::setw(len) << Exp_data[i].CurTheta[3]*Rad2Deg
      << std::setw(len) << Exp_data[i].CurTheta[4]*Rad2Deg
      << std::setw(len) << Exp_data[i].CurTheta[5]*Rad2Deg
      << std::endl;

    f_reftheta << std::left
      << std::setw(len) << Exp_data[i].Time
      << std::setw(len) << Exp_data[i].RefTheta[0]*Rad2Deg
      << std::setw(len) << Exp_data[i].RefTheta[1]*Rad2Deg
      << std::setw(len) << Exp_data[i].RefTheta[2]*Rad2Deg
      << std::setw(len) << Exp_data[i].RefTheta[3]*Rad2Deg
      << std::setw(len) << Exp_data[i].RefTheta[4]*Rad2Deg
      << std::setw(len) << Exp_data[i].RefTheta[5]*Rad2Deg
      << std::endl;
  }

  f_curpos.close();  f_refpos.close();
  f_curtheta.close(); f_reftheta.close();
  printf("Data saved.\n");
}

