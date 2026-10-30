# stepper_arm (步进电机机械臂控制包)
文件修订记录：

| 版本号| 时间   | 备注  | 
| :---: | :-----: | :---: |
|V1.0    |2025-11-27  |拟制 |




## 简介

`stepper_arm` 功能包专为搭载于移动机器人底盘上的二自由度（2-DOF）机械臂设计。该功能包集成了基于简易版视觉伺服的自主色块抓取、基于 OpenCV GUI 的键盘遥控、以及基于激光雷达的避障保护功能。它实现了针对二自由度平面机械臂的自定义逆运动学解算，并利用 OpenCV 进行图像处理。

**核心特性：**
* **自主色块抓取：** 利用计算机视觉（HSV 阈值分割）识别特定颜色的物块，并通过简易版视觉伺服（PID 控制）调整机器人底盘和机械臂末端位置以完成抓取。
* **逆运动学解算：** 包含针对二自由度平面机械臂的几何逆解算法，内置关节限位和工作空间（几何圆弧）约束检查。
* **手动遥控：** 提供一个可视化的 OpenCV GUI 窗口，实时显示机械臂姿态、末端坐标及按键说明，支持通过键盘控制机械臂关节、末端直角坐标移动、夹爪开合及移动底盘。
* **避障保护：** 包含一个安全节点，实时监控激光雷达数据。若机器人运动方向 0.5 米范围内检测到障碍物，将强制停止底盘运动。

---

## 依赖项

* **ROS Humble** (已测试)
* **OpenCV** (3.x 或 4.x)
* `cv_bridge`
* `image_transport`
* `geometry_msgs`
* `std_msgs`
* `nav_msgs`
* `dynamic_reconfigure`
---

## 安装与编译

1.  将此功能包克隆到你的 catkin 工作空间的 `src` 目录下：
    ```bash
    cd ~/catkin_ws/src
    ```

2.  编译工作空间：
    ```bash
    cd ~/catkin_ws
    colcon build
    source install/setup.bash
    ```

---

## 使用说明

### 1. 逼近式色块抓取任务

该启动文件将同时启动机器人底盘驱动、摄像头节点、激光雷达、避障节点以及核心的简易视觉伺服控制节点。
```bash
ros2 launch stepper_arm arm_2_auto_pick_colorBlock.launch.py
ros2 run rqt_reconfigure rqt_reconfigure

```

### 2. 机械臂键盘控制

启动节点前需要同时启动机器人底盘驱动,通过SSH远程登录弹窗GUI界面时用键盘控制可能会出现卡顿延迟现象，建议robot_keyboard_control节点单独在客户端编译运行。

```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
ros2 run stepper_arm arm_2_keyboard_control
```

### 3. 启动机械臂摄像头
```bash
ros2 launch stepper_arm  usb_cam.launch.py
```

### 4. 加载机械臂模型
```bash
  ros2 launch robot_description  robot_display.launch.py
```

指定功能包编译：
colcon build --packages-select "功能包名"

ssh登录：
ssh -Y wheeltec@192.168.0.100

nfs挂载:
sudo mount -t nfs 192.168.0.100:/home/wheeltec/wheeltec_arm  /mnt2
nfs解除挂载:
sudo umount -t nfs 192.168.0.100:/home/wheeltec/wheeltec_arm  /mnt2
