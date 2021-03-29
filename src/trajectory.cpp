
#include "../include/adrobot_etc.h"

TRJ_BUFF PathBuff;

/* 初始化轨迹队列 */
void initTrjBuff() { PathBuff.data_num = 0; }

/* 添加轨迹队列 */
int PutTrjBuff(PATH *path) {
  // if the buffer is full
  if (PathBuff.data_num >= TRAJECTORY_LENGTH) {
    printf("Error: The path buffer is full\n");
    return 1;
  } else {
    if (PathBuff.data_num > 0) {
      for (int i = PathBuff.data_num; i > 0; i--) {
        PathBuff.Path[i] = PathBuff.Path[i-1];
      }
    }
    PathBuff.Path[0] = *path;
    PathBuff.data_num++;
  } // else
  return 0;
}

/* 弹出轨迹队列 */
int GetTrjBuff(PATH *path) {
  // if the buffer is not empty
  if (PathBuff.data_num < 1) {
    printf("Error: The path buffer is empty");
    return 1;
  } else {
    // 取出队首元素
    *path = PathBuff.Path[0];
    // 更新路径队列
    for (int i = 0; i < PathBuff.data_num; i++) {
      PathBuff.Path[i] = PathBuff.Path[i+1];
    }
    PathBuff.data_num--;
  }
  return 0;
}

// ステップ関数による軌道補間 | 阶跃函数的轨迹插补
double CalcStepTraje(double orig, double goal, double freq, double time) {
  if (time < 1.0 / freq / 2.0)
    return orig;
  return goal;
}

// 1 次関数による軌道補間 | 一次函数的轨迹插补
double Calc1JiTraje(double orig, double goal, double freq, double time) {
  double ref = goal;
  double time_n = freq * time;

  if (time_n <= 1)
    ref = orig + (goal - orig) * time_n;
  return ref;
}

// 3 次関数による軌道補間 | 3次函数的轨迹插补
double Calc3JiTraje(double orig, double goal, double freq, double time) {
  double ref = goal;
  double time_n = freq * time;

  if (time_n <= 1)
    ref = orig + (goal - orig) * time_n * time_n * (3.0 - 2 * time_n);
  return ref;
}

// 5 次関数による軌道補間 | 5次函数的轨迹插补
double Calc5JiTraje(double orig, double goal, double freq, double time) {
  double ref = goal;
  double time_n = freq * time;

  if (time_n <= 1)
    ref = orig + (goal - orig) * time_n * time_n * time_n *
          (10.0 + time_n * (-15.0 + 6.0 * time_n));
  return ref;
}

// sin 関数による軌道補間 | sin函数的轨迹插补
double CalcSinTraje(double orig, double goal, double freq, double time) {
  double ref = goal;
  double time_n = freq * time;

  if (time_n <= 1)
    ref = orig + (goal - orig) * time_n -
          (goal - orig) * sin(2.0 * M_PI * time_n) / (2.0 * M_PI);
  return ref;
}

// 3 次関数による速度軌道補間 | 3次函数的速度插补
double Calc3JiTrajeVelo(double orig, double goal, double freq, double time) {
  double ref = 0.0;
  double time_n = freq * time;

  if (time_n <= 1)
    ref = (goal - orig) * freq * 6.0 * time_n * (1 - time_n);
  return ref;
}

// 5 次関数による速度軌道補間 | 5次函数的速度插补
double Calc5JiTrajeVelo(double orig, double goal, double freq, double time) {
  double ref = 0.0;
  double time_n = freq * time;

  if (time_n <= 1)
    ref = freq * (goal - orig) * 30.0 * time_n * time_n * (time_n - 1) *
          (time_n - 1);
  return ref;
}

// sin 関数による速度軌道補間 | sin函数的速度插补
double CalcSinTrajeVelo(double orig, double goal, double freq, double time) {
  double ref = 0.0;
  double time_n = freq * time;

  if (time_n <= 1)
    ref = freq * (goal - orig) * (1 - cos(2.0 * M_PI * time_n));
  return ref;
}

