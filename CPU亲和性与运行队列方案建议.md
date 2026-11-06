# CPU 亲和性与运行队列

CPU 占用过高时，高优先级的 UWB 尾随会在运行队列里等到超时。下面先定义这个问题，再写现在车上是怎么跑的，然后是怎么判断该动哪里。方案一是局部优化，收短控制环自己的计算，当前用这个把超时压住，不立刻改整机的核。方案二是全局设计，按 CPU 亲和性把进程拆开，等方案一压不住、日志又表明是在排队时再上。

代码以 `uwb-priority-release-20261029T010533Z-turn-margin` 为准。`robot-voice-drive.service` 启动该目录的 `start-voice-uwb.sh`。车上若换过目录，看 `systemctl show`。

## 问题

尾随要在一圈 100 毫秒内算完。跟请求超过 150 毫秒就作废，规划超过 0.35 秒就不能再用。到点就停车，不会留到下一圈补做。

这些任务在指令上排在前面：停车、手动、然后才是跟随。这个顺序只决定谁可以发 `/cmd_vel`，不决定谁先占用 CPU。核被占满时，跟车线程和仲裁线程与语音、点云、规划进同一条运行队列，要等前面的线程让出 CPU 才能开始这一圈。等的时间算进那 100 毫秒。

跟车超时的日志是 `final_command_or_control_deadline`，仲裁是 `final_sensor_or_control_deadline`。请求太旧是 `follow_request_invalid_or_stale`。规划没赶上是 `planning` 或 `awaiting_fresh_plan`。

## 现状

没有 `CPUAffinity`，也没有 cpuset。线程可运行就能上任何一颗核，按普通调度轮转。源码里没有 `nice`，也没有 `SCHED_FIFO`。

`robot-voice-drive.service` 里有三条：

- `robot-uwb-reader.py` 读串口，写 `latest.json`
- `robot-uwb-follow.py` 读快照和激光，做安全检查，往 `/uwb_follow/priority_request` 发速度。日志在 `/home/wheeltec/uwb_uart/priority-controller.log`。主循环先 `spin_once(0.02)`，再在 8 毫秒里最多 31 次 `spin_once(0)`。队列空时这是空转
- `robot-voice-uwb.py` 读语音和跟请求，发 `/cmd_vel`。`spin_once(0.005)` 之后最多 63 次空转。非零跟请求上，对跟车已经查过的扫描再做一遍 `motion_safe`

开了 `--obstacle-profile` 时，A* 在跟车进程 `spawn` 出来的子进程里，会继承父进程的 CPU 亲和性。

同一台上还有 `robot-voice-control.service` 的两路 Vosk，`robot-voice-support.service` 的底盘、50 Hz EKF、激光、`filter-chassis-echo.py`、`pointcloud_to_laserscan`，以及 `robot-follow-scan.service`。建图、导航、相机开着时再占核。

跟车和仲裁都是单线程。一颗核够把一条线程的这一圈算完。再绑一颗核不会让同一次 `motion_safe` 变快。

## 方法

先分清这一圈是自己算超时，还是在运行队列里等别人。两种情况改的地方不一样。仓库里没有这次的 `pidstat`，所以方案二里不写死每组几颗核。

一次跟随里把三样东西按时间对齐：跟车 `priority-controller.log`（约 0.2 秒一条）、仲裁 stdout、`pidstat -t 1`。


| 记录                                       | 看什么                                         |
| ---------------------------------------- | ------------------------------------------- |
| `control_elapsed_s`                      | 这一圈的墙钟时间，含在运行队列里等的时间。跟车、仲裁各有一份              |
| `reason`                                 | 控制超时、请求过期、规划没回来，还是规划过期                      |
| `last_rejection`、`last_plan_wait`        | 安全检查是否通过、耗时多少；规划租约是扫描旧了、车走远了，还是目标动了         |
| `target_age_s`、`scan_age_s`、`odom_age_s` | 串口、激光、里程计有没有先变旧                             |
| `interlock_reasons`、`mode`               | 是不是 `scan_stale`、`odometry_stale`；当时是不是正在跟随 |
| `pidstat` 的 `%cpu` 和所在核                  | 超时那一秒，这颗核上还有谁                               |


判断顺序：

1. `control_elapsed_s` 大于 0.1，核上几乎没有别的进程。自己算超时。走方案一。
2. `control_elapsed_s` 大于 0.1，核上还有 Vosk、点云、A* 或其他进程。在排队。走方案二，离开这颗核的是当时叠在一起的那些进程。
3. 跟车和仲裁都超时，且落在同一颗核。两条线程要分开。`target_age_s` 一起变大，读串口才跟它们放在一组。
4. `planning` 或 `awaiting_fresh_plan`，A* 占用高且旁边有别人。A* 单独一组。旁边没有别人仍超时，是算法超过 0.35 秒，再绑一颗核不加快这个单线程。
5. `scan_age_s`、`scan_stale` 或 `odom_age_s`、`odometry_stale` 变大。看激光、点云过滤、跟随 scan、EKF、底盘和谁叠在一起，把传感器从那堆进程里拆出来。
6. `follow_request_invalid_or_stale` 或 `follow_request_missing`。跟车自己的 `control_elapsed_s` 也大，先处理跟车。只有仲裁大，只处理仲裁。

`final_sensor_or_control_deadline` 把算晚了、请求旧了、互锁失败写成同一个 reason，要同时看 `control_elapsed_s` 和 `interlock_reasons`。

## 方案一：局部优化

当前用这个把超时压住，不改整机的核划分。

- 跟车和仲裁去掉成批的 `spin_once(0)`，改成一次带超时的 `spin_once`。有回调就处理，队列空就不要空转。
- 仲裁对跟请求不再做整段 `motion_safe`。跟车已经用当前扫描查过。仲裁留下互锁、请求戳和发布。手动速度仍由仲裁检查。请求戳仍小于 150 毫秒。

100 毫秒、150 毫秒、0.35 秒先不放宽。空转还在时不要上 `SCHED_FIFO`。

方案一够用的标志：跟随中两个进程的 `control_elapsed_s` 稳定低于 0.1，`final_command_or_control_deadline`、`final_sensor_or_control_deadline`、`follow_request_invalid_or_stale`、`planning`、`awaiting_fresh_plan` 不再因调度出现。`scan_stale`、`odometry_stale` 仍按传感器问题处理。

现场已经是这个状态时，方案二不实施。

## 方案二：全局设计

方案一之后，方法论第 2 步及以后仍然成立，再按 CPU 亲和性重分整机。高优先级的尾随线程固定在指定的核上，其他服务不再进入这些核。

四组的 `AllowedCPUs` 互不重叠。每组几颗核用方法论里数出来的线程数填：必须同时赶上、又不能互相等的线程，各一颗。核编号在车上用 `nproc` 取，不写进仓库。


| 组   | 放什么                                                                  |
| --- | -------------------------------------------------------------------- |
| 控制  | 跟车主进程、仲裁。快照一起变旧时加上读串口                                                |
| 规划  | 只有 A* 子进程                                                            |
| 传感器 | 底盘、EKF、激光、`pointcloud_to_laserscan`、跟随 scan、`filter-chassis-echo.py` |
| 后台  | 语音、语音合成、界面、耳机，以及开着的建图、导航、相机。组内可以互相等                                  |


`robot-voice-drive.service` 的 `CPUAffinity` 只写控制组。A* 子进程会继承这份亲和性，起来后用 `taskset` 挪到规划组，再用 `pidstat` 确认它不在控制核上。`robot-voice-support.service` 和 `robot-follow-scan.service` 用传感器组。`robot-voice-control.service` 以及建图、导航、相机、界面用后台组。

改 unit 之前，把 unit、drop-in、启动脚本拷到 `/home/wheeltec/backup/cpu-layout-<时间戳>/`，不要放进正在运行的 `uwb-priority-release-20261029T010533Z-turn-margin/`。留下原路径、`sha256` 和当时的 `systemctl show robot-voice-drive.service`，拷完改只读。`daemon-reload` 后重启，核对 `/proc/<pid>/status` 的 `Cpus_allowed_list`，再按方法论那三样日志跟一次。控制组上的 `control_elapsed_s` 应低于 0.1。安全互锁该停还是停。