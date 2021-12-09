## ✨ ChangeLog
All notable changes to this project will be documented in this file (Since 2021.06.14).

Notation:
* `fix`: 修改 Bug。
* `doc`: 更新文档。
* `perf`: 功能完善。对项目或模块进行了性能优化。
* `chore`: 其他改动。修改注释或文件清理，不影响src和test代码文件的改动。
* `ci`: 持续集成。分支合并等。
* `revert`: 回滚了一些前面的代码。

<!-- The format is based on Keep a [Changelog](https://keepachangelog.com/en/1.0.0/). -->

## ✨ Logs
### 
> commit [](
)
1. `feat`: 新增根据序列号激活相机的功能，支持多相机检测。
1. `perf`: 优化角度限位功能。

### 211201
> commit [50f463b - fix: inv kinematics, swap](
)
1. `feat`: 实现了初始抓取动作(init\_grasp)。
1. `fix`: 新增`swap`函数，修复了逆运动学中部分角度选择错误的问题。

### 211127
> commit [31bb2d2 --- feat: aruco marker](
)
1. `feat`: 删除标定内参的程序，新增外参设定、 Marker 位姿的识别 (-laruco)。
1. `perf`: **机械臂状态类** 中改用齐次变换矩阵。
1. `fix`: **线程管理类** 中增加各节点状态类别: `ON/OFF/IDLE`。

### 211112
> commit [7c8ad56 --- perf: robot\_thread](
https://github.com/lazyshawn/urController/commit/7c8ad56b088e644e4645dbf36013ecdef54c57f3)
1. `perf`: 整合机械臂控制程序，驱动 + 接口。
1. `fix`: 删除多余文件，整理头文件的引用，解决交叉引用问题。

### 211111
> commit [134ad64 --- perf: urConfig](
https://github.com/lazyshawn/urController/commit/134ad647b021094a60426af57ca95c2ce6544ae5)
1. `fix`: 调整线程共享变量的声明位置，优化共享变量结构。

> commit [ce72646 --- doc: socket](
https://github.com/lazyshawn/urController/commit/ce72646b92fd85bb0c4c7ce69b68e3d567b1c581)
1. `doc`: 补充 Socket 通信文档。

### 211108
> commit [cedbd30 --- feat: integrate sensor](
https://github.com/lazyshawn/urController/commit/cedbd304aa1c3c32e67eaa89cccac29d625472ce)
1. `ci`: 集成三维力传感器 **XELA\_uSkin** (CAN通信)。

### 211029
> commit [9bc4f0b --- bk: driver/interface](
https://github.com/lazyshawn/urController/commit/9bc4f0bd6748d5445cc99e5c68188239cefda050)
1. `perf`: 修改`CMakeList.txt`，分别编译各节点驱动，生成对应的链接库。
1. `perf`: 节点程序按 `driver/interface/main` 结构编写，分别为驱动、接口、调试程序。
1. `perf`: 修改部分文件、变量名称。

### 210722 ~ 210924
> commit [40c7f0a --- bkup: calibrator](
https://github.com/lazyshawn/urController/commit/40c7f0a1fb06e95938cc8b5ed9b67d623513036e); 
[0afa677 --- backup: calibration](
https://github.com/lazyshawn/urController/commit/0afa6779a75aaa907a3f6fd38b02dcfda6c73c69)
1. `feat`: 相机内参标定程序 (**archived**) 。


### 210720
> commit [a9d5a90 --- ci: realsense](
https://github.com/lazyshawn/urController/commit/a9d5a90c2a23e44e4bea3024141de01ae20a4d42)
1. `perf`: 终端彩色信息输出 `printf_status`。
1. `ci`: 集成了 Realsense 相机，封装成类。
1. `fix`: 修复了速度指令中 Eigen 库报错的 Bug。

### 210717
> commit [69902d1 --- ci: robotiq](
https://github.com/lazyshawn/urController/commit/69902d172706a93023fd99e4f18ddc1e3d617ff5)
1. `ci`: 集成 RobotiQ 夹爪，封装成类。

### 210716
> commit [26690b1 -- perf: eigen](
https://github.com/lazyshawn/urController/commit/26690b143b7e8b56918e7c2409fe2a880a99eae4)
1. `perf`: 使用 `Eigen` 库替代自定义的 `matrix` 矩阵运算库。
1. `perf`: 新增 `path_planning` 文件，用于路径的上层规划。
1. `fix`: 修正 DH 参数下标: $alpha_{i-1}$, $a_{i-1}$。
1. `fix`: 修复了速度指令中的伺服指令(8ms)不执行的问题。
1. `feat`: 新增了求解机械臂在 X-Z 平面内运动的正逆解的函数。


### 210709
> commit [7f907c3 --- ci: robotiq](
https://github.com/lazyshawn/urController/commit/7f907c3c5a61bde9f6ae2ce623eead15ad1e9cef)
1. `feat`: 集成了 RobotiQ 夹爪。可以实现开合控制、读取夹爪位置、夹持力控制。
1. `feat`: 添加了按键判断，不再需要回车输入命令。
1. `perf`: 轨迹规划部分的代码中，输入与规划分层实现。
1. `feat`: 实现用键盘控制机械臂在 X-Z 平面内运动。

### 210629 ~ 210704
> commit [c8f2bd9 --- feat: inv kinematics](
https://github.com/lazyshawn/urController/commit/c8f2bd91e6fa93a2b79daf078d433399492fa1d3)
1. `perf`: 用户界面的使用标准输入输出流。
1. `feat`: 添加了基于逆运动学的轨迹规划。

> commit [18b573b --- doc: inverse kinematics](
https://github.com/lazyshawn/urController/commit/18b573bdc0d25c13003c38560c4a1209f8cb187b)
1. `fix`: 用参考角度计算速度伺服，解决了实际实验时速度较小时机械臂不动的问题。
1. `feat`: 添加了逆运动学推导过程。
1. `perf`: 仿真脚本中添加逆运动学函数。

### 210622
> commit [d44cb23 --- perf: reconstruct for concurrency](
https://github.com/lazyshawn/urController/commit/d44cb231d273717bc1dd01ee7cb33e45ddd82b60)
1. `fix`: 修复了路径运行完成后角度会跳跃到 0 的问题。
1. `perf`: 优化多线程结构。修改路径(PATH), 系统状态的共享变量(SVO)等数据结构，
使用 `std::array` 描述定长数组。
1. `perf`: 路径队列添加用于 「debug」 的方法，控制伺服周期的暂停和继续。
1. `perf`: 优化 `trajectory` 中插值代码的逻辑，删除了多余函数，简化伺服线程代码。
1. `perf`: 修改部分变量及函数的名称。变量: 小驼峰法。函数: 下划线连接。
1. `chore`: 修改交互菜单。

**Todos**:
* [ ] 在上层代码中使用 `std::cout` 替换 `printf`。
* [ ] 新增结构化的输出提示。
* [ ] 集成 Robotic-Q 夹爪。

### 210617 ~ 210619
> commit [9cd6417 --- chore: redundant data](
https://github.com/lazyshawn/urController/commit/9cd6417bd73d3e3f7da3dcd3176fc190c40845fa)
1. `chore`: 删除了多余的数据结构。

> commit [52d2eea --- perf: std::mutex](
https://github.com/lazyshawn/urController/commit/52d2eea5f37744a308ff5738d7bbc8db7ee1be41)
1. `perf`: 主函数中改用 C++17 标准的线程。

> commit [0bd979d --- perf: std::thread](
https://github.com/lazyshawn/urController/commit/0bd979d788751aaf8ec99529113fde5675558a27)
1. `perf`: 构造 Config 类来替代全局变量 SVO。
1. `perf`: 改用 std::mutex，优化线程中对全局变量的读写操作。


### 210614
> commit [16431e8 --- fix: jacobian for rotation](
https://github.com/lazyshawn/urController/commit/16431e873d1005f81c0eae4becbccf485270a1b2)
1. 添加修改日志。
1. `fix`: 修改 Jacobian 矩阵，实现正确的角速度控制。
1. `perf`: 修改文件写入函数，使用C++风格。
1. `perf`: 添加离线运行功能。

