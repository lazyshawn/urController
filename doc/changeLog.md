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
### 210617
>commit
[52d2eea](
https://github.com/lazyshawn/urController/commit/52d2eea5f37744a308ff5738d7bbc8db7ee1be41)

1. `perf`: 主函数中改用 C++17 标准的线程。

>commit
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

