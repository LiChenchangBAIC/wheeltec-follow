# CPU 亲和性与运行队列

UWB 尾随在核被占满时会停。现在没有设置 CPU 亲和性，跟车、仲裁和语音、点云、规划进同一条运行队列。下面先写运行队列怎么把尾随停掉，再写现在哪些进程在抢这些核。绑到几颗核，要等车上日志把“线程自己算超时”和“在运行队列里等”分开之后才能定。测完再按数据流把进程放进互斥的 cpuset。都还没改车上的服务。

代码仍以 `uwb-priority-release-20261029T010533Z-turn-margin` 为准。`turn-margin-final-runtime.json` 里 `robot-voice-drive.service` 启动 `start-voice-uwb.sh`。车上若换过目录，以 `systemctl show` 为准。

## 运行队列里发生了什么

Linux 默认把所有可运行线程放进同一组核的运行队列，按普通调度轮转。源码里没有 `nice`、`SCHED_FIFO`、`taskset`、cpuset。尾随在调度上和语音识别、点云过滤是同一档。

核都忙的时候，跟车线程和仲裁线程要等队列前面的线程让出 CPU，才能开始这一圈。等完再算。用户态里这一圈的预算是写死的，等的时间和算的时间都算进去：

- 跟车或仲裁这一圈超过 100 毫秒就停车。跟车日志是 `final_command_or_control_deadline`，仲裁日志是 `final_sensor_or_control_deadline`。
- 跟请求时间戳超过 150 毫秒就丢。在 `PriorityControl.step()` 里丢掉时 reason 是 `follow_request_invalid_or_stale`。已经算出速度、收尾才发现过期，也记成 `final_sensor_or_control_deadline`。
- 规划结果超过 0.35 秒不能再用。还没回来是 `planning`，过期是 `awaiting_fresh_plan`，速度都是 0。

尾随等的是 CPU 运行队列。预算用完就停车，不会留到下一圈再算。

跟车和仲裁必须同一小段时间里都跑完。只让其中一个排到队头，另一个仍会把 100 毫秒或 150 毫秒用掉。UWB 快照本身允许到 0.6 秒（`--uwb-timeout .6`），控制环等不到这么久。

`final_sensor_or_control_deadline` 把“算晚了”“请求戳旧了”“互锁失败”写成同一个 reason。对照时要同时看 `control_elapsed_s` 和 `interlock_reasons`。

## 现在哪些进程在抢核

没有 cpuset，也没有 `CPUAffinity`。线程可运行就能上任何一颗核。实际叠在一起跑的，从启动脚本和车上记录看，主要是这些：

`robot-voice-drive.service` 里有三条：

- `robot-uwb-reader.py` 读串口，写 `latest.json`。这一路轻，但写晚了后面两路都拿不到新目标。
- `robot-uwb-follow.py` 读快照和激光，做 `motion_safe`，往 `/uwb_follow/priority_request` 发速度。日志在 `/home/wheeltec/uwb_uart/priority-controller.log`。主循环先 `spin_once(0.02)`，再在 8 毫秒里最多 31 次 `spin_once(0)`。队列空时这是空转，线程会一直可运行，更容易占住核。
- 同一个进程开了 `--obstacle-profile` 时，A* 在 `ProcessPoolExecutor`（`spawn`）的子进程里。子进程继承父进程的 CPU 亲和性。父进程若整组绑死，A* 会跟着进那组核，在保护核里面把跟车主线程挤到队列后面。
- `robot-voice-uwb.py` 读语音日志和跟请求，发 `/cmd_vel`。主循环 `spin_once(0.005)` 之后最多 63 次空转。非零跟请求上，对跟车已经查过的扫描再做一遍 `motion_safe`。

旁边还有：

- `robot-voice-control.service`：`robot-voice-visual.py`，两路 Vosk，自己也有 `spin_once(0)` 空转。
- `robot-voice-support.service`：底盘节点、50 Hz 的 EKF、激光、`filter-chassis-echo.py` 的 Python 逐点过滤、`pointcloud_to_laserscan`。扫描是跟车和仲裁的输入，这条链路慢了会变成 `scan_stale`、`odometry_stale`。
- `robot-follow-scan.service`：跟随专用 scan。
- 耳机相关服务，以及人开着的 RTAB/Nav2、相机、界面。这些不是每趟跟随都在，开着的时候会把剩下的核占满。

现有 `priority` 只管指令顺序：停车、手动、跟随。它不改变上面任何一个进程在运行队列里的位置。

会进运行队列，是因为两件事叠在一起。时限短的线程和算得久的线程共用核，A*、Vosk、逐点滤点云都可以把跟车、仲裁挡在队列里超过 100 毫秒。跟车和仲裁自己又用空转保持可运行，调度器一旦把核分给它们，空转会占住这颗核，同核上的另一个控制线程照样得等。

## 先测再定核数

跟车主循环和仲裁主循环都是单线程。一颗核已经够一条线程把这一圈算完。再把同一条线程绑到第二颗核上，不会让同一次 `motion_safe` 变快。需要多绑一颗，只是因为两条必须同时赶上的线程现在挤在同一颗上，或者传感器、A* 被别的进程挡住了。

绑到几颗核是测出来的：先看这一圈是自己算完就超过 100 毫秒，还是在运行队列里等别的进程。仓库里没有这次的 `pidstat`，文档里不写“某进程绑几颗”。

一次跟随里同时留三样东西，时间对齐：

- 跟车 `priority-controller.log`，大约 0.2 秒一条 JSON
- 仲裁进程 stdout
- 同一段时间的 `pidstat -t 1`，或者至少每秒的进程 `%cpu` 和当时落在哪颗核上

跟车日志里用这些字段：

| 字段 | 看什么 |
| --- | --- |
| `reason` | 这一圈是控制超时、规划没回来，还是规划过期 |
| `control_elapsed_s` | 跟车这一圈从开始到结束的墙钟时间，含排队 |
| `last_plan_wait` | 规划租约是因为扫描旧了、车走远了，还是目标动了 |
| `last_rejection` | 跟车侧 100 毫秒检查拒绝时，安全检查有没有过、耗时多少 |
| `target_age_s` | 串口快照是不是已经旧了 |
| `scan_age_s` | 激光是不是已经旧了 |
| `odom_age_s` | 里程计是不是已经旧了 |

仲裁日志里用这些字段：

| 字段 | 看什么 |
| --- | --- |
| `reason` | `final_sensor_or_control_deadline`、`follow_request_invalid_or_stale`、`follow_request_missing` |
| `control_elapsed_s` | 仲裁这一圈的墙钟时间 |
| `interlock_reasons` | 里面若已有 `scan_stale`、`odometry_stale`，停车来自传感器，不是尾随线程自己没排上 |
| `mode` | 当时是不是正在跟随 |

`pidstat` 用来回答日志回答不了的那半句：超时的那一秒，这颗核上还有谁。

判定按这个顺序，先别设 `CPUAffinity`：

1. `control_elapsed_s` 大于 0.1，同时 `pidstat` 里这条线程所在的核上几乎没有别的进程。这是自己算超时。再绑一颗核没有用，先看空转和重复的 `motion_safe`。`last_rejection` 里的耗时可以对照。
2. `control_elapsed_s` 大于 0.1，而这颗核上同时有 Vosk、点云过滤、A* 或其他服务。这是运行队列。离开这颗核的是 `pidstat` 里和它叠在一起的那些进程，不是事先点名的一份清单。
3. 跟车和仲裁的 `control_elapsed_s` 都大，而且 `pidstat` 显示它们落在同一颗核。这两条线程要拆开，各绑一颗。读串口只有在 `target_age_s` 一起变大时才跟它们放在一组，否则不必再绑一颗。
4. `reason` 是 `planning` 或 `awaiting_fresh_plan`，同时 A* 子进程 `%cpu` 很高、旁边还有别的进程。把 A* 绑到一颗其他进程进不去的核。若 A* 自己 `%cpu` 很高但旁边没有别人，仍是 `awaiting_fresh_plan`，那是规划算法超过 0.35 秒，再绑一颗核不会让这个单线程变快。
5. `scan_age_s` 变大或 `interlock_reasons` 里有 `scan_stale`。看激光、`filter-chassis-echo.py`、`pointcloud_to_laserscan`、跟随 scan 在 `pidstat` 里和谁叠在一起。叠在一起的离开传感器这些进程。`odom_age_s` 或 `odometry_stale` 同样处理 EKF 和底盘节点。
6. `follow_request_invalid_or_stale` 或 `follow_request_missing`，而跟车自己的 `control_elapsed_s` 也大。先处理跟车那一条，请求戳是被跟车拖晚的。跟车 `control_elapsed_s` 正常、只有仲裁大，只隔离仲裁。

绑几颗，等于必须同时赶上、又不能互相等的线程数。第 1 步成立时先改代码，再绑核消不掉自己算出来的超时：去掉成批 `spin_once(0)`；仲裁对跟请求不必再做跟车已经做过的整段 `motion_safe`。手动速度仍由仲裁检查。空转还在时不要上 `SCHED_FIFO`。100 毫秒、150 毫秒、0.35 秒先别放宽。

## 测完之后怎么排布

进程按数据流分成四组。哪一组要单独占核，看上一节哪一步成立。组里放几颗核，用那一步数出来的线程数填。核的编号在车上从 `nproc` 里取，不写进仓库。四组的 `AllowedCPUs` 不能有交集。

| 组 | 进程 | 宽度从哪一步来 |
| --- | --- | --- |
| 控制 | `robot-uwb-follow.py` 主进程、`robot-voice-uwb.py`。`target_age_s` 一起变大时，加上 `robot-uwb-reader.py` | 第 3、6 步。互相等的线程各一颗；`pidstat` 里没有互相挡住的，可以共一颗 |
| 规划 | A* 子进程 | 第 4 步。它和别的进程叠在一起才单独一颗。自己就把 0.35 秒用完时，不靠再绑一颗核 |
| 传感器 | 底盘节点、EKF、激光、`pointcloud_to_laserscan`、`robot-follow-scan`、`filter-chassis-echo.py` | 第 5 步。谁把 `scan_age_s` 或 `odom_age_s` 拖大，就和当时叠在同一颗核上的进程拆开 |
| 后台 | `robot-voice-visual.py`、TTS、界面、耳机，以及正在跑的 RTAB/Nav2、相机 | 剩下的核。这一组内部可以互相等 |

最小排布只建控制组：把第 2 步里叠上来的进程赶出控制核，规划组和传感器组先留在原来的核上。第 4 步或第 5 步也成立时，再把对应的组拆出来，变成四组都互斥。

A* 从跟车进程 `spawn` 出来，会继承父进程的 CPU 亲和性。`robot-voice-drive.service` 的 `CPUAffinity` 只能写成控制组的核。子进程起来之后用 `taskset` 挪到规划组，再用 `pidstat` 看它是不是还留在控制核上。

落地顺序：

1. 按下一节备份要改的 unit、drop-in 和启动脚本。
2. 把上一节数出来的宽度写成四组核列表。最小排布可以先只写控制组，其余核合成一组。
3. `robot-voice-drive.service` 的 drop-in 里，`CPUAffinity` 只含控制组。
4. `robot-voice-support.service`、`robot-follow-scan.service` 的 `AllowedCPUs` 写成传感器组。最小排布时这两项先不改。
5. `robot-voice-control.service` 以及 RTAB/Nav2、相机、界面的 `AllowedCPUs` 写成后台组，不能包含控制组的核。
6. `daemon-reload` 后重启这些服务，把 A* 子进程 `taskset` 到规划组。核对每个进程 `/proc/<pid>/status` 里的 `Cpus_allowed_list`。
7. 用同一套日志和 `pidstat` 再跟一次。控制组上的 `control_elapsed_s` 应低于 0.1。`planning`、`awaiting_fresh_plan`、`scan_stale`、`odometry_stale` 若还在，对照第 1 步看是自己算超时，还是组里仍有不该在一起的进程。安全互锁该停还是停。

## 备份

真正改 unit 之前，把要动的 unit、drop-in、启动脚本拷到 `/home/wheeltec/backup/cpu-layout-<UTC时间戳>/`。不要放进 `uwb-priority-release-20261029T010533Z-turn-margin/`。留下原路径、`sha256` 和当时的 `systemctl show robot-voice-drive.service`，拷完改只读。回退用这份副本盖回去，`daemon-reload` 后再重启。
