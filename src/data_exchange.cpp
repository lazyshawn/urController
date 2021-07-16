/* *********************************************************
 * ==>> 数据交换
 * *********************************************************/

#include "../include/data_exchange.h"

// Shared variable
Config config;
Path_queue path_queue;

/* 
 * @class : Config
 * @brief : 读写全局的共享变量
 */
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

/* 
 * @class : Path_queue
 * @brief : 用于多线程操作的路径队列
 */
// 添加新路径
void Path_queue::push(PATH path) {
  std::scoped_lock lock(path_mutex);
  data.push_back(std::move(path));
  path_cond.notify_one();
}

// 等待并弹出路径
void Path_queue::wait_and_pop(PATH& path) {
  std::unique_lock lock(path_mutex);
  path_cond.wait(lock, [this]{return !data.empty();});
  path = std::move(data.front());
  data.pop_front();
}

// 尝试弹出路径
bool Path_queue::try_pop(PATH& path) {
  std::scoped_lock lock(path_mutex);
  if (data.empty()) return false;
  path = std::move(data.front());
  data.pop_front();
  return true;
}

bool Path_queue::try_pop(PATH& path, double time, ARRAY orig) {
  std::scoped_lock lock(path_mutex);
  if (data.empty()) return false;
  path = std::move(data.front());
  path.beginTime = time;
  path.orig = orig;
  data.pop_front();
  return true;
}

// 判断路径队列是否为空
bool Path_queue::empty() const {
  std::scoped_lock lock(path_mutex);
  return data.empty();
}

// 进入等待(for debug)
void Path_queue::wait() {
  std::unique_lock lock(path_mutex);
  path_cond.wait(lock);
}

// 唤醒在等待的条件变量(for debug)
void Path_queue::notify_one() {
  std::scoped_lock lock(path_mutex);
  path_cond.notify_one();
}

/* 
 * @func  : add_joint_path
 * @brief : 从键盘录入关节角的运动路径
 * @param : path路径的引用
 * @return: void
 */
void read_joint_destination(NUMBUF& inputData) {
  printf("\n-----------------Now you are in JntSvoMode!-----------------\n");
  std::cout << "Set the path information.\n" << "Path duration [s]: ";
  std::cin >> inputData[6];
  std::cout << "PATH: SIN(0) 5JI(1) 3JI(2) 1JI(3) STEP(4)" << std::endl;
  std::cout << "Path mode = ";
  std::cin >> inputData[7];
  // 读入角度路径信息
  for (int i = 0; i < 6; i++) {
    std::cout << "Angle of joint " << i+1 << "[deg] = ";
    std::cin >> inputData[i];
  }
  printf("-------------------------------------------------------------\n");
}

/* 
 * @func  : add_hand_path
 * @brief : 从键盘录入末端位姿的运动路径
 * @param : path路径的引用
 * @return: void
 */
void read_displacement(NUMBUF& inputData) {
  double gain, temp;
  printf("\n---------------Now you are in PosOriServoMode!---------------\n");
  printf("Set the path information.\n");
  // 读入路径信息
  std::cout << "Path duration [s] = " << std::endl;
  std::cin >> inputData[6];
  // 平动位移量
  std::cout << "Translation velocity x of the end_link[mm]: ";
  std::cin >> inputData[0];
  std::cout << "Translation velocity y of the end_link[mm]: ";
  std::cin >> inputData[1];
  std::cout << "Translation velocity z of the end_link[mm]: ";
  std::cin >> inputData[2];
  // 转动位移量
  std::cout << "Angular velocity around x of the end_link[deg/s]: ";
  std::cin >> inputData[3];
  std::cout << "Angular velocity around x of the end_link[deg/s]: ";
  std::cin >> inputData[4];
  std::cout << "Angular velocity around x of the end_link[deg/s]: ";
  std::cin >> inputData[5];
  printf("-------------------------------------------------------------\n");
}

/* 
 * @func  : add_destination
 * @brief : 从键盘录入 Cartesion 空间内的目标位置
 * @param : path-路径的引用, curTheta-当前关节角度
 * @return: void
 */
void read_cartesion_destination(NUMBUF& inputData) {
  printf("\n---------------Now you are in PosOriServoMode!---------------\n");
  std::cout << "Set the destination information.\n" << std::endl;
  // 运动时间
  std::cout << "Path duration [s] = " << std::endl;
  std::cin >> inputData[6];
  // 位姿信息
  std::cout << "X position[mm]: ";           std::cin >> inputData[0];
  std::cout << "Z position[mm]: ";           std::cin >> inputData[2];
  std::cout << "Tilt angle[deg]: ";          std::cin >> inputData[3];
  std::cout << "Finger displacement [mm]: "; std::cin >> inputData[8];
  printf("-------------------------------------------------------------\n");
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
      << std::setw(len) << Exp_data[i].time
      << std::setw(len) << Exp_data[i].curPos[0]
      << std::setw(len) << Exp_data[i].curPos[1]
      << std::setw(len) << Exp_data[i].curPos[2]
      << std::setw(len) << Exp_data[i].curPos[3]*Rad2Deg
      << std::setw(len) << Exp_data[i].curPos[4]*Rad2Deg
      << std::setw(len) << Exp_data[i].curPos[5]*Rad2Deg
      << std::endl;

    f_refpos << std::left
      << std::setw(len) << Exp_data[i].time
      << std::setw(len) << Exp_data[i].refPos[0]
      << std::setw(len) << Exp_data[i].refPos[1]
      << std::setw(len) << Exp_data[i].refPos[2]
      << std::setw(len) << Exp_data[i].refPos[3]*Rad2Deg
      << std::setw(len) << Exp_data[i].refPos[4]*Rad2Deg
      << std::setw(len) << Exp_data[i].refPos[5]*Rad2Deg
      << std::endl;

    f_curtheta << std::left
      << std::setw(len) << Exp_data[i].time
      << std::setw(len) << Exp_data[i].curTheta[0]*Rad2Deg
      << std::setw(len) << Exp_data[i].curTheta[1]*Rad2Deg
      << std::setw(len) << Exp_data[i].curTheta[2]*Rad2Deg
      << std::setw(len) << Exp_data[i].curTheta[3]*Rad2Deg
      << std::setw(len) << Exp_data[i].curTheta[4]*Rad2Deg
      << std::setw(len) << Exp_data[i].curTheta[5]*Rad2Deg
      << std::endl;

    f_reftheta << std::left
      << std::setw(len) << Exp_data[i].time
      << std::setw(len) << Exp_data[i].refTheta[0]*Rad2Deg
      << std::setw(len) << Exp_data[i].refTheta[1]*Rad2Deg
      << std::setw(len) << Exp_data[i].refTheta[2]*Rad2Deg
      << std::setw(len) << Exp_data[i].refTheta[3]*Rad2Deg
      << std::setw(len) << Exp_data[i].refTheta[4]*Rad2Deg
      << std::setw(len) << Exp_data[i].refTheta[5]*Rad2Deg
      << std::endl;
  }

  f_curpos.close();  f_refpos.close();
  f_curtheta.close(); f_reftheta.close();
  printf("Data saved.\n");
}

