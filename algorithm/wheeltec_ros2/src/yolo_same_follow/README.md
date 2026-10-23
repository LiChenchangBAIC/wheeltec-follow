# YOLO 点选锁定尾随

版本二。用 YOLO 实例分割和 ByteTrack 编号跟随用户点中的那一个目标。选定之后不会自己换成别的物体；编号消失就停车。

速度仍由原来的 `visualfollow` 根据 `/object_tracker/current_position` 计算。不要同时开色块 `visualtracker`、雷达跟随，或另一个也在发 `/cmd_vel` 的节点。

## 车上准备

在 Jetson 上、已有 Ultralytics 的 Python 环境里导出 engine（必须在车上做，不能在 Mac 上编）：

```bash
cd ~/wheeltec_ros2
source /opt/ros/humble/setup.bash
# 使用机械臂那套环境时：
source ~/ros2_yolo_env/bin/activate
colcon build --packages-select yolo_same_follow --symlink-install
source install/setup.bash
ros2 run yolo_same_follow export_seg_engine --weights yolov8n-seg.pt --device 0
```

导出得到 `yolov8n-seg.engine`。相机保持已启动的那一路即可，这个 launch 不会再开一遍相机。

## 启动

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_ros2/install/setup.bash
ros2 launch yolo_same_follow yolo_same_follow.launch.py model:=/绝对路径/yolov8n-seg.engine
```

浏览器打开 `http://<车IP>:8090/`，点击要跟的物体。取消锁定后车停下，直到下一次点选。
