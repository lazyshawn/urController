#include "../include/ur5e_interface.h"

urConfig urconfig;
PathQueue pathQueue;
extern ThreadManager threadManager;

/*************************************************************************
 * @class: urConfig
 * @brief: 保存机械臂状态
*************************************************************************/
// 获取全局的机械臂状态
urConfig::Data urConfig::get_data(void) {
  std::scoped_lock guard(config_mutex);
  return data;
}

/* 将线程更新的共享变量同步到全局 */
void urConfig::update(urConfig::Data* Data_) {
  std::scoped_lock guard(config_mutex);
  data = *Data_;
}

/*************************************************************************
 * @class : Path_queue
 * @brief : 用于多线程操作的路径队列
*************************************************************************/
// 添加新路径
void PathQueue::push(PATH path) {
  std::scoped_lock lock(path_mutex);
  data.push_back(std::move(path));
  path_cond.notify_one();
}

// 等待并弹出路径
void PathQueue::wait_and_pop(PATH& path) {
  std::unique_lock lock(path_mutex);
  path_cond.wait(lock, [this]{return !data.empty();});
  path = std::move(data.front());
  data.pop_front();
}

// 尝试弹出路径
bool PathQueue::try_pop(PATH& path) {
  std::scoped_lock lock(path_mutex);
  if (data.empty()) return false;
  path = std::move(data.front());
  data.pop_front();
  return true;
}

// 尝试弹出路径并初始化
bool PathQueue::try_pop(PATH& path, double time, ARRAY orig) {
  std::scoped_lock lock(path_mutex);
  if (data.empty()) return false;
  path = std::move(data.front());
  path.beginTime = time;
  path.orig = orig;
  data.pop_front();
  return true;
}

// 判断路径队列是否为空
bool PathQueue::empty() const {
  std::scoped_lock lock(path_mutex);
  return data.empty();
}

// 进入等待(for debug)
void PathQueue::wait() {
  std::unique_lock lock(path_mutex);
  path_cond.wait(lock);
}

// 唤醒在等待的条件变量(for debug)
void PathQueue::notify_one() {
  std::scoped_lock lock(path_mutex);
  path_cond.notify_one();
}

/*************************************************************************
 * 机械臂伺服程序
*************************************************************************/
// 机械臂线程函数
void robot_thread_function(void) {
  // UR通信的条件变量
  std::condition_variable rt_ur_msg_cond, ur_msg_cond;
  // UR的IP地址(静态)
  std::string ur_ip = "10.249.181.201";
  // UR的通信端口
  unsigned int ur_post = 50007;
  // 夹爪通信串口
  char rbtqPort[] = "/dev/ttyUSB0";
  // 系统时间
  struct timespec t;
  // 线程共享变量的局部备份
  urConfig::Data urConfigData = urconfig.get_data();
  std::vector<double> jnt_angle(6);

#ifndef ROBOT_OFFLINE
  /* connect to ur robot */
  std::cout << "Connecting to " << ur_ip << std::endl;
  UrDriver urRobot(rt_ur_msg_cond, ur_msg_cond, ur_ip, ur_post);
  urRobot.start();
  usleep(100); // 机械臂初始化，确保能TCP通信连上(accept)
  urRobot.setServojTime(0.008);
  urRobot.uploadProg();
  jnt_angle = urRobot.rt_interface_->robot_state_->getQActual();
#else
  UrDriver* urRobot;
  jnt_angle = {0, -90, 90, -90, -90, 0};
  // jnt_angle = {0, -98.9, 117.8, -108.9, -90, 90};
  for (int i=0; i<6; ++i) {
    jnt_angle[i] *= Deg2Rad;
  }
#endif
  for (int i=0; i<6; ++i) {
    urConfigData.path.goal[i] = urConfigData.path.orig[i] = urConfigData.refTheta[i]
      = urConfigData.curTheta[i] = jnt_angle[i];
  }
  // 同步全局变量
  std::cout << "\n==>> Robot is ready.\n" << std::endl;
  urconfig.update(&urConfigData);

  /* Declare ourself as a real time task */
  // Data structure to describe a process' schedulability
  struct sched_param param;
  // 设置线程的优先级(最大为99)
  param.sched_priority = MY_PRIORITY;
  // 设置线程的调度策略和调度参数
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    perror("sched_setscheduler failed\n");
    exit(-1);
  }

  /* Lock memory | 防止内存交换 */
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    perror("mlockall failed");
    exit(-2);
  }

  /* 一秒后启动控制回路 */
  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_sec++;

  while (threadManager.process != THREAD_EXIT) {
    /* Wait until next shot | 休眠到下个周期开始 */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    /* 伺服线程主程序 */
#ifndef ROBOT_OFFLINE
    servo_function(&urRobot);
#else
    servo_function(urRobot);
#endif

    /* calculate next shot | 设置下一个线程恢复的时间 */
    t.tv_nsec += NSEC_PER_PERIOD;

    // 时间进位
    while (t.tv_nsec >= NSEC_PER_SEC) {
      t.tv_nsec -= NSEC_PER_SEC;
      t.tv_sec++;
    }
  } // while (1)

  // close UR
#ifndef ROBOT_OFFLINE
  urRobot.halt();
#endif
  // 保存实验数据
  // ExpDataWrite();
} // robot_thread_function()


/*************************************************************************
 * @func  : servo_function
 * @brief : 伺服主程序_对路径进行插值，并发送运动指令
 * @param : UrDriver*_机械臂驱动类的指针
 * @return: void
*************************************************************************/
void servo_function(UrDriver* ur) {
  // Copy global urConfig::Data
  urConfig::Data urConfigData = urconfig.get_data();
  std::vector<double> jnt_angle(6);

  // Get the current time
  urConfigData.time = GetCurrentTime();

  /* Obtain the current state of ur */
#ifndef ROBOT_OFFLINE
  jnt_angle = ur->rt_interface_->robot_state_->getQActual();
  // copy the UR state
  for (int i = 0; i < 6; i++) urConfigData.curTheta[i] = jnt_angle[i];
#else
  for (int i=0; i<6; ++i) {
    jnt_angle[i] = urConfigData.curTheta[i] = urConfigData.refTheta[i];
  }
#endif

  /* Update the kinematics calculation*/
  // 计算各关节角的正余弦值
  calcJnt(urConfigData.curTheta);
  // 正运动学求解
  ur_kinematics(urConfigData.tranMat);
  // Current position of the end_link

  /* Pop path info */
  if (urConfigData.path.complete) {
    // 当前路径运行完成，且队列中没有新的路径
    if (pathQueue.empty()) {
      urconfig.update(&urConfigData);
      return;
    }
    // 尝试弹出路径
    pathQueue.try_pop(urConfigData.path, urConfigData.time, urConfigData.curTheta);
    // 弹出时初始化参考角度
    for (int i=0; i<6; ++i) urConfigData.refTheta[i] = urConfigData.curTheta[i];
  }
  // pathQueue.wait();  // For debug: wake up by pathQueue.notify_one()

  /* 计算轨迹插补点(关节角目标值) */
  calc_ref_joint(urConfigData);

#ifndef ROBOT_OFFLINE
  // 机械臂执行运动指令
  for (int i=0; i<6; ++i) jnt_angle[i] = urConfigData.refTheta[i];
  ur->servoj(jnt_angle, 1);
#endif
  /* 记录待保存的数据 */
  // ExpDataSave(&urConfigData);

  // Update global urConfig::Data
  urconfig.update(&urConfigData);
}

/*************************************************************************
 * @func  : calc_ref_joint
 * @brief : 计算轨迹插补点
 * @param : urConfigData_共享变量
 * @return: 下一插补点处的 关节角/位姿
*************************************************************************/
void calc_ref_joint(urConfig::Data& urConfigData) {
  // 当前路径执行的时间
  double offsetTime = urConfigData.time - urConfigData.path.beginTime;
  double freq = urConfigData.path.freq;
  double delQ = 8*Deg2Rad;

  // 角度伺服
  if (urConfigData.path.angleServo) {
    ARRAY orig = urConfigData.path.orig;
    ARRAY goal = urConfigData.path.goal;
    int interpMode = urConfigData.path.interpMode;
    urConfigData.path.complete = joint_interpolation(
        offsetTime, freq, interpMode, orig, goal, urConfigData.refTheta);
  } else {
  // 速度伺服
    urConfigData.path.complete = velocity_interpolation(
        offsetTime, freq, urConfigData.curTheta, urConfigData.refTheta, urConfigData.path.velocity);
  } // if {} else {}

  // 角度限位
  for (int i=0; i<6; ++i) {
    if (urConfigData.refTheta[i] - urConfigData.curTheta[i] > delQ) {
      urConfigData.refTheta[i] = urConfigData.curTheta[i] + delQ;
    } else if (urConfigData.refTheta[i] - urConfigData.curTheta[i] < -delQ) {
      urConfigData.refTheta[i] = urConfigData.curTheta[i] - delQ;
    }
  }
} // calc_ref_joint()


/*************************************************************************
 * 轨迹规划
*************************************************************************/
void add_joint_destination(PATH& path, NUMBUF& inputData) {
  // 角度伺服标志置位
  path.angleServo = ON;
  // 读入角度路径信息
  path.freq = 1/inputData[6];
  path.interpMode = inputData[7];
  for (int i = 0; i < 6; i++) {
    path.goal[i] = inputData[i]*Deg2Rad;
  }
  path.fingerPos = inputData[8];
  pathQueue.push(path);
}

void add_displacement(PATH& path, NUMBUF& inputData) {
  double gain, temp;
  // 取消角度伺服标志
  path.angleServo = OFF;
  // 读入路径信息
  path.freq = 1/inputData[6];
  gain = path.freq * path.delT;
  for (int i=0; i<3; ++i) {
    // 平动位移量
    path.velocity[i] = inputData[i];
    // 转动位移量
    path.velocity[i+3] = inputData[i+3]*Deg2Rad;
  }
  // 归一化: 转化为一个伺服周期内的位移量
  for (int i=0; i<6; ++i) path.velocity[i] *= gain;
  path.fingerPos = inputData[8];
  pathQueue.push(path);
}

void add_cartesion_destination(PATH& path, NUMBUF& inputData, THETA curTheta) {
  double oriR, oriP, oriY;
  Mat4d tranMat;
  // 角度伺服标志置位
  path.angleServo = ON;
  path.interpMode = 2;
  // 读入路径信息
  path.freq = 1/inputData[6];
  // 位置
  tranMat(0,3) = inputData[0];
  tranMat(1,3) = DH_D4;
  tranMat(2,3) = inputData[2];
  // 姿态
  oriP = inputData[3] * Deg2Rad;
  tranMat(0,0) = cos(oriP); tranMat(2,0) =  sin(oriP); tranMat(1,1) = -1;
  tranMat(0,2) = sin(oriP); tranMat(2,2) = -cos(oriP);
  path.fingerPos = inputData[8];
  path.goal = ur_InverseKinematics(tranMat, curTheta);
  pathQueue.push(path);
}

void go_to_pose(Mat4d tranMat, THETA curTheta) {
  PATH path;
  // 角度伺服标志置位
  path.angleServo = ON;
  path.interpMode = 2;
  path.freq = 1.0/5;
  path.goal = ur_InverseKinematics(tranMat, curTheta);
  pathQueue.push(path);
}

void robot_go_home(THETA curTheta){
  PATH path;
  NUMBUF inputData = {400, DH_D4, 300, 0, 0, 0, 5, 0, 10};
  add_cartesion_destination(path, inputData, curTheta);
}

void pivot_about_points(TRIARR& state, TRIARR command, double time) {
  double xr = state[0], zr = state[1], qr = state[2];
  double x0 = command[0], z0 = command[1], q0 = command[2];
  double alpha0 = atan2(zr-z0, xr-x0);
  double radius = sqrt((zr-z0)*(zr-z0)+(xr-x0)*(xr-x0));

  // 按圆心角插值(圆的参数方程)
  THETA theta;
  double n = floor(time/SERVO_TIME);
  double da = command[2]/n;

  for (int i=0; i<n; ++i) {
    state[0] = x0 + radius*cos(alpha0+da*(i+1));
    state[1] = z0 + radius*sin(alpha0+da*(i+1));
    state[2] = qr + da*(i+1);
    // state[2] = qr;
    theta = plane_invese_kinematics(state);
    instant_command(theta);
  }
  printf("done\n");
}

void instant_command(THETA refTheta) {
  PATH pathLocal;
  pathLocal.fingerPos = 70;
  for (int i=0; i<6; ++i) {
    pathLocal.goal[i] = refTheta[i];
  }
  pathQueue.push(pathLocal);
}

