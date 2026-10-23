# Road Network Planner

基于 ROS2 的手动路网编辑器和导航功能包。

## 功能特性

- ✅ **手动添加路标点**：在 RViz2 中点击地图添加路标点
- ✅ **手动连接路标点**：切换到连接模式，依次点击两个路标点创建连接
- ✅ **障碍物检测**：自动检测路径上的障碍物，有障碍物的地方不允许连线
- ✅ **基于路网的导航**：使用 Dijkstra 算法在路网上进行路径规划
- ✅ **路网持久化**：支持保存/加载 GeoJSON 格式的路网文件

## 安装与编译

```bash
# 进入工作空间
cd ~/wheeltec_ros2

# 编译功能包
colcon build --packages-select road_network_planner

# 刷新环境变量
source install/setup.bash
```

## 使用方法

### 1. 启动路网编辑器

```bash
ros2 launch road_network_planner manual_editor_launch.py
```

### 2. 在 RViz2 中配置可视化

打开 RViz2：

```bash
ros2 run rviz2 rviz2
```

添加以下组件：

| 组件 | Topic | Frame | 说明 |
|------|-------|-------|------|
| **Map** | `/map` | `map` | 显示地图 |
| **Marker** | `road_network/waypoint_marker` | `map` | 显示路标点 |
| **Marker** | `road_network/edge_marker` | `map` | 显示路网连接 |
| **RobotModel** | - | `map` | 显示机器人模型 |
| **TF** | `/tf` | - | 显示坐标变换 |

设置 **Fixed Frame** 为 `map`。

### 3. 添加路标点

使用 RViz2 工具栏中的 **Publish Point** 工具（图钉图标）点击地图添加路标点。

### 4. 切换到连接模式

```bash
ros2 service call /road_network/set_edit_mode std_srvs/srv/SetBool "{data: true}"
```

### 5. 创建路网连接

在连接模式下：
1. 点击第一个路标点（选中）
2. 点击第二个路标点（创建连接）
3. 如果路径上有障碍物，不会创建连接

### 6. 切换回添加点模式

```bash
ros2 service call /road_network/set_edit_mode std_srvs/srv/SetBool "{data: false}"
```

### 7. 设置导航目标

使用 RViz2 工具栏中的 **2D Nav Goal** 工具点击地图设置导航目标。

小车会沿着你绘制的路网进行路径规划和导航。

## 服务接口

### 设置编辑模式

```bash
ros2 service call /road_network/set_edit_mode std_srvs/srv/SetBool "{data: true}"
```

- `data: false`：添加点模式（默认）
- `data: true`：连接模式

### 保存路网

```bash
ros2 service call /road_network/save_network std_srvs/srv/Trigger
```

### 加载路网

```bash
ros2 service call /road_network/load_network std_srvs/srv/Trigger
```

### 清空路网

```bash
ros2 service call /road_network/clear_network std_srvs/srv/Trigger
```

## 导航日志输出

导航过程中会输出以下日志：

```
[INFO] Starting ID navigation: 0 -> 3
[INFO] Found path: 0 -> 1 -> 2 -> 3
[INFO] Navigating to waypoint 0 at (0.00, 0.00)
[INFO] ✅ Arrived at waypoint 0
[INFO] Navigating to waypoint 1 at (1.00, 0.00)
[INFO] ✅ Arrived at waypoint 1
[INFO] Navigating to waypoint 2 at (2.00, 0.50)
[INFO] ✅ Arrived at waypoint 2
[INFO] ✅ Navigation completed successfully!
```

## 文件结构

```
road_network_planner/
├── scripts/
│   └── manual_network_editor.py   # 手动路网编辑器节点
├── launch/
│   └── manual_editor_launch.py    # 启动文件
├── srv/
│   └── SetEditMode.srv            # 设置编辑模式服务
├── package.xml                     # 包配置
├── CMakeLists.txt                  # CMake 配置
└── README.md                       # 说明文档
```

## 注意事项

1. 路标点只能添加在可通行区域（白色/灰色区域）
2. 连接路标点时，如果路径穿过障碍物，不会创建连接
3. 导航时小车会严格按照绘制的路网路径运动
4. 确保 TF 变换链完整：`map -> odom -> base_footprint`

## 技术原理

### 路径规划算法

使用 Dijkstra 算法在路网上寻找最短路径：

1. 将路网视为无向图
2. 每个路标点作为图的节点
3. 连接线作为图的边
4. 使用优先队列进行最短路径搜索

### 障碍物检测

在创建连接线时，沿路径采样检查代价地图：

1. 每 0.1m 采样一个点
2. 检查采样点的代价值
3. 如果代价值超过阈值（默认 60），认为有障碍物
4. 有障碍物时不创建连接
