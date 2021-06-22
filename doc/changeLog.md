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
### 210622
> commit
[](
)

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

### 210619
> commit
[9cd6417](
https://github.com/lazyshawn/urController/commit/9cd6417bd73d3e3f7da3dcd3176fc190c40845fa)

1. `chore`: 删除了多余的数据结构。

### 210617
> commit
[52d2eea](
https://github.com/lazyshawn/urController/commit/52d2eea5f37744a308ff5738d7bbc8db7ee1be41)

1. `perf`: 主函数中改用 C++17 标准的线程。

> commit
[0bd979d](
https://github.com/lazyshawn/urController/commit/0bd979d788751aaf8ec99529113fde5675558a27)

1. `perf`: 构造 Config 类来替代全局变量 SVO。
1. `perf`: 改用 std::mutex，优化线程中对全局变量的读写操作。


### 210614
> commit
[16431e8](
https://github.com/lazyshawn/urController/commit/16431e873d1005f81c0eae4becbccf485270a1b2)

日常维护。
1. 添加修改日志。
1. `fix`: 修改 Jacobian 矩阵，实现正确的角速度控制。
1. `perf`: 修改文件写入函数，使用C++风格。
1. `perf`: 添加离线运行功能。

