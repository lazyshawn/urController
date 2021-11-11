#include "../include/ur5e_interface.h"

urConfig urconfig;
PathQueue pathQueue;

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

