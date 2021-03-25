/* *********************************************************
 * ==>> 文件操作，保存数据
 * *********************************************************/

#include "../include/common.h"

int flag_WriteData = OFF;
int flag_SaveData = OFF;

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
  FILE *file1, *file2, *file3, *file4, *file5, *file6, *file7, *file8, *file9,
      *file10, *file11, *file12;
  file1 = fopen("../data/data.Curpos", "w");
  file6 = fopen("../data/data.RefPos", "w");
  file2 = fopen("../data/data.for", "w");
  file3 = fopen("../data/data.theta", "w");
  file4 = fopen("../data/data.rtheta", "w");
  file5 = fopen("../data/data.dtheta", "w");
  file7 = fopen("../data/data.markpos", "w");      // 标记点在机器人坐标系下的坐标
  file12 = fopen("../data/data.refmarkpos", "w");
  file9 = fopen("../data/data.errpos", "w");
  file8 = fopen("../data/data.distogripper", "w"); // 标记点到夹持器的距离
  file10 = fopen("../data/data.deltajnt", "w");    // 关节变化量
  file11 = fopen("../data/data.xianweijiaodu", "w");

  printf("saving data ... \n");
  for (int i = 0; i < Exp_data_index; ++i) {
    fprintf(file1, "%.8f\t%.8f\t%.8f\t%.8f\t%.3f\t%.3f\t%.3f\n",
            Exp_data[i].Time, Exp_data[i].CurPos.t[0], Exp_data[i].CurPos.t[1],
            Exp_data[i].CurPos.t[2], Exp_data[i].CurPos.t[3] * Rad2Deg,
            Exp_data[i].CurPos.t[4] * Rad2Deg,
            Exp_data[i].CurPos.t[5] * Rad2Deg);
    fprintf(file6, "%.8f\t%.8f\t%.8f\t%.8f\t%.3f\t%.3f\t%.3f\n",
            Exp_data[i].Time, Exp_data[i].RefPos.t[0], Exp_data[i].RefPos.t[1],
            Exp_data[i].RefPos.t[2], Exp_data[i].RefPos.t[3] * Rad2Deg,
            Exp_data[i].RefPos.t[4] * Rad2Deg,
            Exp_data[i].RefPos.t[5] * Rad2Deg);
    fprintf(file2, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
            Exp_data[i].Time, Exp_data[i].CurForce.t[0],
            Exp_data[i].CurForce.t[1], Exp_data[i].CurForce.t[2],
            Exp_data[i].CurForce.t[3], Exp_data[i].CurForce.t[4],
            Exp_data[i].CurForce.t[5]);

    fprintf(file3, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
            Exp_data[i].Time, Exp_data[i].CurTheta.t[0] * Rad2Deg,
            Exp_data[i].CurTheta.t[1] * Rad2Deg,
            Exp_data[i].CurTheta.t[2] * Rad2Deg,
            Exp_data[i].CurTheta.t[3] * Rad2Deg,
            Exp_data[i].CurTheta.t[4] * Rad2Deg,
            Exp_data[i].CurTheta.t[5] * Rad2Deg);

    fprintf(file4, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
            Exp_data[i].Time, Exp_data[i].RefTheta.t[0] * Rad2Deg,
            Exp_data[i].RefTheta.t[1] * Rad2Deg,
            Exp_data[i].RefTheta.t[2] * Rad2Deg,
            Exp_data[i].RefTheta.t[3] * Rad2Deg,
            Exp_data[i].RefTheta.t[4] * Rad2Deg,
            Exp_data[i].RefTheta.t[5] * Rad2Deg);

    fprintf(file5, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
            Exp_data[i].Time, Exp_data[i].CurDTheta.t[0] * Rad2Deg,
            Exp_data[i].CurDTheta.t[1] * Rad2Deg,
            Exp_data[i].CurDTheta.t[2] * Rad2Deg,
            Exp_data[i].CurDTheta.t[3] * Rad2Deg,
            Exp_data[i].CurDTheta.t[4] * Rad2Deg,
            Exp_data[i].CurDTheta.t[5] * Rad2Deg);

    fprintf(file7,
            "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%."
            "3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
            Exp_data[i].Time, Exp_data[i].markpos.t[0],
            Exp_data[i].markpos.t[1], Exp_data[i].markpos.t[2],
            Exp_data[i].markpos.t[3], Exp_data[i].markpos.t[4],
            Exp_data[i].markpos.t[5], Exp_data[i].markpos.t[6],
            Exp_data[i].markpos.t[7], Exp_data[i].markpos.t[8],
            Exp_data[i].markpos.t[9], Exp_data[i].markpos.t[10],
            Exp_data[i].markpos.t[11], Exp_data[i].markpos.t[12],
            Exp_data[i].markpos.t[13], Exp_data[i].markpos.t[14],
            Exp_data[i].markpos.t[15], Exp_data[i].markpos.t[16],
            Exp_data[i].markpos.t[17]);
    fprintf(file8, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
            Exp_data[i].Time, Exp_data[i].distogripper.t[0],
            Exp_data[i].distogripper.t[1], Exp_data[i].distogripper.t[2],
            Exp_data[i].distogripper.t[3], Exp_data[i].distogripper.t[4],
            Exp_data[i].distogripper.t[5]);
    fprintf(file9,
            "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%."
            "3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
            Exp_data[i].Time, Exp_data[i].errpos.t[0], Exp_data[i].errpos.t[1],
            Exp_data[i].errpos.t[2], Exp_data[i].errpos.t[3],
            Exp_data[i].errpos.t[4], Exp_data[i].errpos.t[5],
            Exp_data[i].errpos.t[6], Exp_data[i].errpos.t[7],
            Exp_data[i].errpos.t[8], Exp_data[i].errpos.t[9],
            Exp_data[i].errpos.t[10], Exp_data[i].errpos.t[11],
            Exp_data[i].errpos.t[12], Exp_data[i].errpos.t[13],
            Exp_data[i].errpos.t[14], Exp_data[i].errpos.t[15],
            Exp_data[i].errpos.t[16], Exp_data[i].errpos.t[17]);
    fprintf(file10, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
            Exp_data[i].Time, Exp_data[i].deltaTheta.t[0],
            Exp_data[i].deltaTheta.t[1], Exp_data[i].deltaTheta.t[2],
            Exp_data[i].deltaTheta.t[3], Exp_data[i].deltaTheta.t[4],
            Exp_data[i].deltaTheta.t[5]);
    fprintf(file11, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
            Exp_data[i].Time, Exp_data[i].xianweiTheta.t[0],
            Exp_data[i].xianweiTheta.t[1], Exp_data[i].xianweiTheta.t[2],
            Exp_data[i].xianweiTheta.t[3], Exp_data[i].xianweiTheta.t[4],
            Exp_data[i].xianweiTheta.t[5]);
    fprintf(file12,
            "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%."
            "3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
            Exp_data[i].Time, Exp_data[i].refmarkpos.t[0],
            Exp_data[i].refmarkpos.t[1], Exp_data[i].refmarkpos.t[2],
            Exp_data[i].refmarkpos.t[3], Exp_data[i].refmarkpos.t[4],
            Exp_data[i].refmarkpos.t[5], Exp_data[i].refmarkpos.t[6],
            Exp_data[i].refmarkpos.t[7], Exp_data[i].refmarkpos.t[8],
            Exp_data[i].refmarkpos.t[9], Exp_data[i].refmarkpos.t[10],
            Exp_data[i].refmarkpos.t[11], Exp_data[i].refmarkpos.t[12],
            Exp_data[i].refmarkpos.t[13], Exp_data[i].refmarkpos.t[14],
            Exp_data[i].refmarkpos.t[15], Exp_data[i].refmarkpos.t[16],
            Exp_data[i].refmarkpos.t[17]);
  }

  // 关闭文件
  fclose(file1);
  fclose(file2);
  fclose(file3);
  fclose(file4);
  fclose(file5);
  fclose(file6);
  fclose(file7);
  fclose(file8);
  fclose(file9);
  fclose(file10);
  fclose(file11);
  fclose(file12);
  printf("Data saved.\n");
}

