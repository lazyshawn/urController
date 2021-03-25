### 内核线程优先级的设置
设置优先级时需要用到struct sched\_param这个结构。
调度策略有三种：**SCHED\_NORMAL** 非实时调度策略；
**SCHED\_FIFO**实时调度策略，先到先服务；
**SCHED\_RR**实时调度策略，时间片轮转 $^{[1]}$。

```cpp
// 设置优先级
struct sched_param param;
param.sched_priority = 99;
// 设置调度策略，出错时返回-1
sched_setscheduler(current, SCHED_FIFO, &param);
```

#### Reference
1. [内核线程优先级设置的方法介绍][1]

[1]:(http://www.zzvips.com/article/117072.html)

