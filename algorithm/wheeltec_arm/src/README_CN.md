<div align="center">
export DASHSCOPE_API_KEY="sk-0ca5ca7818fc411ea683951606bdccd5"

# S300机械臂ROS2工作空间使用说明书V1.0
 
轮趣科技（东莞）有限公司 
</div>

## 目录
* [工作空间概述](#工作空间概述)
* [系统要求](#系统要求)
* [环境搭建](#环境搭建)
* [安装依赖库](#安装依赖库)
* [编译工作空间](#编译工作空间)
* [功能包说明](#功能包说明)
* [常用命令](#常用命令)
* [控制真实机械臂](#控制真实机械臂)
* [运动演示](#运动演示)
* [视觉抓取](#视觉抓取)
* [目录结构](#目录结构)
* [常见问题](#常见问题)
* [安全提示](#安全提示)
* [开发指南](#开发指南)
* [技术支持](#技术支持)

## 工作空间概述
本工作空间为S300系列机械臂的ROS2完整开发环境，集成了机械臂驱动、运动规划、视觉抓取等完整功能。适用于机械臂的二次开发、教学演示和实际应用场景。

### 主要特性
- ✅ 完整的ROS2驱动支持
- ✅ MoveIt2运动规划集成
- ✅ 视觉识别与抓取
- ✅ 手眼标定功能
- ✅ 多种运动演示示例
- ✅ TracIK高性能逆运动学求解

### 支持的机器人型号
- S300 Pro
- S300 Mini


## 系统要求

### 硬件要求
- CPU： ARM64架构，建议4核以上
- 内存：建议8GB以上
- 硬盘：建议20GB以上可用空间
- 网络：用于连接机械臂控制器

### 软件要求
- 操作系统：Ubuntu 22.04 LTS
- ROS2版本：Humble Hawksbill
- Python版本：3.10+
- OpenCV版本：4.10+
- 机械臂控制器版本：1.4.3以上

## 环境搭建

### 安装依赖库
安装机械臂驱动所需的依赖库：

```bash
cd ros2_rm_robot-humble/rm_driver/lib
sudo bash lib_install.sh
```

### 编译工作空间
返回工作空间根目录进行编译：

```bash
# 假设当前在 src 目录
cd ..
colcon build --packages-select rm_ros_interfaces
source ./install/setup.bash
colcon build
```

编译成功后，需要source环境：

```bash
source install/setup.bash
```

建议将此命令添加到`~/.bashrc`中，以便每次打开终端自动加载：

```bash
echo "source ~/your_workspace_path/install/setup.bash" >> ~/.bashrc
```

## 功能包说明

### 核心功能包

#### 1. ros2_rm_robot-humble
睿尔曼机器人ROS2驱动功能包集，包含以下子功能包：
- **rm_driver**：机械臂底层驱动
- **rm_bringup**：快速启动功能包，手眼标定结果
- **rm_description**：机械臂URDF模型描述
- **rm_ros_interfaces**：ROS2消息接口定义
- **rm_control**：MoveIt2与硬件驱动通信桥接
- **rm_gazebo**：Gazebo仿真环境
- **rm_example**：基础使用案例
- **rm_doc**：技术文档
- **rm_eco65_moveit2_config**：ECO65机械臂MoveIt2配置
- **s300_mini_moveit2_config**：S300 Mini机械臂MoveIt2配置
- **s300_pro_moveit2_config**：S300 Pro机械臂MoveIt2配置

详细说明请参考：[ros2_rm_robot-humble/README_CN.md](ros2_rm_robot-humble/README_CN.md)

#### 2. arm_demo
机械臂运动控制演示功能包，提供以下功能：
- 正运动学控制演示
- 逆运动学控制演示
- 圆形轨迹运动
- 笛卡尔空间运动
- MoveIt2基础类封装

详细说明请参考：[arm_demo/README_CN.md](arm_demo/README_CN.md)

#### 3. grab_demo
基于视觉的机械臂抓取演示功能包，提供以下功能：
- 相机标定与配置
- 手眼标定
- ArUco标记识别与抓取
- 颜色识别与抓取
- HSV参数调试工具
- 抓取与放置服务接口

详细说明请参考：[grab_demo/README_CN.md](grab_demo/README_CN.md)

#### 4. vlm_demo
基于VLM多模态大模型，提供以下功能： 自然语言模糊语意控制机械臂/底盘运动，视觉夹取，动作排列组合 


### 第三方功能包

#### 4. trac_ik-ros2_plugin
高性能TracIK逆运动学求解器插件，提供比KDL更快速准确的逆运动学求解能力。

#### 5. vision_opencv-humble
OpenCV与ROS2的桥接功能包，包含：
- **cv_bridge**：OpenCV图像与ROS消息转换
- **image_geometry**：相机几何模型
- **opencv_tests**：测试工具

## 常用命令

### 控制真实机械臂

⚠️ **注意：控制真实机械臂前，请确保已完成机械臂网络连接配置，并确认周围环境安全！**

启动机械臂驱动和MoveIt2控制界面：

```bash
# S300 Pro机械臂
ros2 launch rm_bringup s300_pro_real_robot.launch.py

# S300 Mini机械臂
ros2 launch rm_bringup s300_mini_real_robot.launch.py

# ECO65机械臂
ros2 launch rm_bringup eco65_real_robot.launch.py


启动成功后，需要在客户端启动以下命令，机械臂的实时状态会同步显示。
ros2 launch s300_pro_moveit2_config moveit_rviz.launch.py
(根据实际车型选择，以S300 pro为例)
```
### 运动演示

运行各种运动控制演示程序：

```bash
# 正运动学演示
ros2 launch arm_demo fk_demo.launch.py

# 逆运动学演示
ros2 launch arm_demo ik_demo.launch.py

# 圆形轨迹运动
ros2 launch arm_demo move_circle.launch.py

# 笛卡尔空间运动
ros2 launch arm_demo moveit_cartesian.launch.py
```

### 视觉抓取

#### 1：手眼标定
使用视觉抓取功能，根据需要进行手眼标定(工作空间已有手眼标定结果)：

```bash
ros2 launch grab_demo hand_eye.launch.py
```

使用Charuco标定板完成标定。

#### 2：运行抓取程序

**ArUco标记抓取：**
```bash
ros2 launch grab_demo aruco_grab.launch.py 
(检查rviz2 物体的tf与机械臂的相对位置是否正确)

夹取: 
ros2 run grab_demo start_grab aruco_x_frame

放置: 
ros2 service call /obj_place_service grab_demo/srv/PlaceObject

```

**颜色识别抓取：**
```bash
ros2 launch grab_demo color_grab.launch.py 
(先进行HSV色域调参，检查rviz2 物体的tf与机械臂的相对位置是否正确)

夹取: 
ros2 run grab_demo start_grab color_link

放置: 
ros2 service call /obj_place_service grab_demo/srv/PlaceObject

```


**YOLO识别抓取：**
```bash
启动相机，机械臂，相机内参等节点:
ros2 launch grab_demo yolo_grab.launch.py  

启动YOLOV8识别节点:
ros2 run yolo_tensorrt_pkg yolo_node

夹取: 
ros2 run grab_demo start_grab yolo_object_link
放置: 
ros2 service call /obj_place_service grab_demo/srv/PlaceObject



如果对yolo_tensorrt_pkg 功能包或者对wheeltec_arm 全局colcon build 编译，需要重新执行下面指令重新指定yolo识别节点的python路径 
sed -i '1s|#!/usr/bin/python3|#!/home/wheeltec/ros2_yolo_env/bin/python3|' /home/wheeltec/wheeltec_arm/install/yolo_tensorrt_pkg/lib/yolo_tensorrt_pkg/yolo_node

如果需要把PyTorch 的 .pt 模型文件转换为 TensorRT 的 .engine 文件，需要先在已激活虚拟环境的终端运行，source ~/ros2_yolo_env/bin/activate

```


#### 3：伺服视觉

**伺服视觉(aruco码位置跟随)：**
```bash
ros2 launch grab_demo visual_servo.launch.py 
启动相机，机械臂，相机内参等节点,启动后会先运动到初始点，然后自动关闭moveit

伺服视觉跟随: 
ros2 run grab_demo visual_servo 
```

#### 4：底盘+机械臂配合

**导航移动夹取：**
```bash 
终端1:   ros2 launch grab_demo aruco_grab.launch.py 
终端2:   ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py start_state_publisher:=false
终端3:   ros2 run grab_demo nav_grab 
```

### VLM机械臂多模态控制
```bash 
终端1: 先启动 LLM 控制所需的机器人节点,需要联网 
export DASHSCOPE_API_KEY="sk-0ca5ca7818fc411ea68xxxxxxxxxx"
ros2 launch vlm_demo vlm_control.launch.py

终端2:启动文字输入终端 
ros2 run vlm_demo command_node

*其他*
**夹爪控制：**
```bash
夹爪张开:ros2 topic pub /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "position: 1000"
夹爪闭合:ros2 topic pub /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "position: 0"
```
**指定功能包编译：**
```bash
colcon build --packages-select package_name --symlink-install
```

**编译前修改系统时间：**
```bash
sudo date -s "2025-06-15 09:00:00"
```

**SSH登录：**
```bash
ssh -Y wheeltec@192.168.0.100
```
**在终端显示的路径下生成TF树pdf：**
```bash
ros2 run tf2_tools view_frames
```
**设置车型：**
```bash
echo 'export ROBOT_TYPE=s300_pro'>>~/.bashrc
```

**查看当前底盘相对地图原点的位姿：**
```bash
ros2 run tf2_ros tf2_echo map base_link
```

**查看机械臂末端夹爪相对base_link的位姿：**
```bash
ros2 run tf2_ros tf2_echo base_link gripper
```



## 目录结构

```
s300-realman/humble/src/
├── arm_demo/                          # 机械臂运动演示功能包
│   ├── include/                       # 头文件
│   ├── launch/                        # 启动文件
│   ├── src/                           # 源代码
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── README_CN.md
├── grab_demo/                         # 视觉抓取演示功能包
│   ├── config/                        # 配置文件
│   ├── include/                       # 头文件
│   ├── launch/                        # 启动文件
│   ├── src/                           # 源代码
│   ├── srv/                           # 服务定义
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── README_CN.md
├── ros2_rm_robot-humble/              # 睿尔曼机器人ROS2功能包集
│   ├── rm_driver/                     # 机械臂驱动
│   ├── rm_bringup/                    # 启动功能包吗，手眼标定结果
│   ├── rm_description/                # 模型描述
│   ├── rm_ros_interfaces/             # 消息接口
│   ├── rm_control/                    # 控制桥接
│   ├── rm_gazebo/                     # 仿真环境
│   ├── rm_example/                    # 使用案例
│   ├── rm_doc/                        # 技术文档
│   ├── rm_install/                    # 安装脚本
│   ├── rm_eco65_moveit2_config/       # ECO65 MoveIt2配置
│   ├── s300_mini_moveit2_config/      # S300 Mini MoveIt2配置
│   ├── s300_pro_moveit2_config/       # S300 Pro MoveIt2配置
│   ├── LICENSE
│   ├── README_CN.md
│   └── README.md
├── trac_ik-ros2_plugin/               # TracIK逆运动学插件
│   ├── trac_ik/
│   ├── trac_ik_examples/
│   ├── trac_ik_kinematics/
│   ├── trac_ik_lib/
│   ├── trac_ik_python/
│   └── README.md
├── vision_opencv-humble/              # OpenCV与ROS2桥接
│   ├── cv_bridge/
│   ├── image_geometry/
│   ├── opencv_tests/
│   └── README.md
└── README_CN.md                       # 本文档
```

## 常见问题

### Q1: 编译时报错找不到某些依赖包
**A:** 确保已经安装了所有必要的依赖：
```bash
sudo apt update
sudo apt install ros-humble-moveit ros-humble-gazebo-ros-pkgs
cd ros2_rm_robot-humble/rm_driver/lib
sudo bash lib_install.sh
```

### Q2: 无法连接到真实机械臂
**A:** 请检查：
1. 机械臂控制器是否已开机
2. 网络连接是否正常（ping测试）
3. 机械臂IP地址配置是否正确
4. 防火墙是否阻止了连接

### Q3: MoveIt2规划失败
**A:** 可能的原因：
1. 目标位姿超出机械臂工作空间
2. 目标位姿会导致自碰撞
3. 逆运动学无解


### Q4: 视觉抓取位置不准确
**A:** 建议检查：
1. 手眼标定是否准确，必要时重新标定
2. 相机内参标定是否正确
3. 光照条件是否稳定
4. ArUco标记或目标物体是否清晰可见


### Q5: 编译时OpenCV版本冲突
**A:** grab_demo使用OpenCV 4.10，确保系统中安装了正确版本的OpenCV，并在CMakeLists.txt中正确设置了OpenCV路径。

## 安全提示

⚠️ **在使用机械臂时，为保证使用者安全，请务必遵守以下操作规范：**

1. **使用前检查**
   - 检查机械臂的安装情况，包括固定螺丝是否松动
   - 检查机械臂是否存在震动、晃动的情况
   - 检查电源线、网线等连接是否牢固

2. **运行时注意**
   - 机械臂运行过程中，人员不可处于机械臂落下或工作范围内
   - 不可将其他物体放到机械臂动作的安全范围内
   - 首次运行程序时建议降低速度，确认动作正确后再提高速度
   - 始终保持急停按钮在可触及范围内

3. **停机时注意**
   - 不使用机械臂时，应将机械臂置于安全位置
   - 防止震动时机械臂跌落而损坏或砸伤其他物体
   - 及时断开机械臂电源

4. **调试时注意**
   - 调试新程序时，建议先在仿真环境中验证
   - 使用真实机械臂调试时，确保已设置合理的速度和加速度限制
   - 准备好急停措施

## 开发指南

### 创建自定义功能包
在工作空间的src目录下创建新的ROS2功能包：

```bash
cd src
ros2 pkg create --build-type ament_cmake your_package_name
```

### 二次开发建议
1. 参考`arm_demo`学习基础运动控制
2. 参考`grab_demo`学习视觉识别与抓取
3. 使用`rm_ros_interfaces`中定义的消息和服务接口
4. 阅读`rm_doc`中的技术文档了解详细API


### 常用开发工具
- **RViz2**：可视化工具
- **rqt**：ROS2图形化工具集
- **ros2 topic**：话题监控工具
- **ros2 service**：服务调试工具

## 技术支持

### 联系方式
- 公司：轮趣科技（东莞）有限公司
- 网址：[WHEELTEC](https://www.wheeltec.net/)

### 参考资源
- [ROS2官方文档](https://docs.ros.org/en/humble/)
- [MoveIt2官方文档](https://moveit.ros.org/)
- [睿尔曼机器人官方网站](http://www.realman-robotics.com/)

---

## License
本项目包含多个开源项目，各子项目遵循其各自的开源协议。详情请参考各功能包中的LICENSE文件。

---

<div align="center">

**感谢使用S300机械臂ROS2工作空间！**

如有任何问题或建议，欢迎联系轮趣科技淘宝技术支持团队。
文件修订记录：
2025-10-30:
①拟制 
2025-12-8:
①更新相机驱动包，解决小面积物体无深度距离问题
②手眼标定结果camera_link 修改为camera_arm_link
③导航识别夹取由色块夹取更换为aruco夹取，识别点修改为侧前边夹取
2025-12-23：
①更新rm_driver、rm_ros_interfaces，避免机械臂固件号和驱动版本不对应导致机械臂SDK-API报错
2026.3.27
①新增yolov8机械臂识别夹取功能
②增加aruco 伺服视觉跟随(位置跟随)例程
③增加点云滤波节点，新增moveit 点云实时避障功能
④提高自动手眼标定结果精度
2026.7.6
新增VLM多模态控制例程
26.8.17
车型配置修改为turn_on_wheeltec_robot 参数文件读取,cv_bridge功能包更换为二进制安装,兼容通用镜像


</div>


