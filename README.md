# QD_Vision2026

奇点战队2026赛季视觉项目仓库，该项目基于中南大学FYT战队24赛季开源

## 一、项目结构

```
.
│
├── rm_bringup (启动及参数文件)
│
├── rm_robot_description (机器人urdf文件，坐标系的定义)
│
├── rm_interfaces (自定义msg、srv)
│
├── rm_hardware_driver
│   │
│   ├── hik-camera (海康相机驱动)
│   │
│   └── rm_serial_driver (串口驱动)
│
├── rm_auto_aim (自瞄算法)
│
├── rm_rune (打符算法)
│
├── rm_utils (工具包) 
│   ├── math (包括PnP解算、弹道补偿等)
│   │
│   └── logger (日志库)
│
├── rm_upstart (自启动配置)
|
└── rm_auto_replay (自动播放录包)
```

## 二、环境配置

```bash
# 海康相机 udev 规则
sudo tee /etc/udev/rules.d/rm_hikrobot.rules >/dev/null <<'EOF'
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="2bdf", MODE="0666", GROUP="plugdev"
EOF
# 串口 udev 规则
sudo tee /etc/udev/rules.d/rm_usb.rules >/dev/null <<'EOF'
KERNEL=="ttyUSB*", MODE:="0777", SYMLINK+="rm_usb0"
KERNEL=="ttyACM*", MODE:="0777", SYMLINK+="rm_usb0"
EOF
# 重新加载 udev 规则
sudo udevadm control --reload-rules
sudo udevadm trigger -v --action=add
```

### Docker

```bash
# x86
docker pull slirute/qidian:latest
# arm64
docker pull slirute/qidian:arm64v8 
```

使用 docker 部署环境，根据需要注释`docker-compose.yaml`里的`command`（只能有一条 command）

```bash
# 构建容器
docker compose up -d 
# 进入容器
docker exec -it rv_devel_ bash
# 进入工作空间
cd /ros_ws
```

### Local

#### 1. 基础

- Ubuntu 22.04
- ROS2 Humble

#### 2. APT 

```bash
sudo apt install libfmt-dev libceres-dev
```

### 3. openvino

参考[OpenVINO官方文档](https://docs.openvino.ai/2022.3/openvino_docs_install_guides_installing_openvino_from_archive_linux.html)，建议同时安装GPU相关依赖

### 4. ROS包

使用`rosdep`安装剩下依赖

```bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-humble-asio-cmake-module       #该包不会自动安装
```

如果还有欠缺的请看根目录下的Dockerfile文件

### Pixi

```bash
# 安装 pixi
curl -fsSL https://pixi.sh/install.sh | sh
# 安装依赖
pixi install
# 进入环境
pixi shell
```

## 三、编译与运行

修改[rm_bringup/config/launch_params.yaml](src/rm_bringup/config/launch_params.yaml)，选择需要启动的功能

当前网口相机的正式内参保存在 `src/rm_bringup/config/camera_info.yaml`（1624×1240），
手眼外参保存在 `launch_params.yaml`，其中 `rpy` 的顺序为 roll、pitch、yaw，单位为弧度。
相机序列号、曝光和增益在 `config/node_params/camera_driver_params.yaml` 中设置；
电控串口与波特率在 `config/node_params/serial_driver_params.yaml` 中设置。
实体网口相机启动前，launch 会按 `launch_params.yaml` 中的 `camera_network` 配置，
为 `enp3s0` 添加 `10.99.99.1/24`；容器须使用 host 网络并具备网卡配置权限。
回放或视频输入模式不会配置网卡。

容器内启动正式自瞄与 Foxglove Bridge：

```bash
cd /ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

`enable_foxglove_bridge: true` 会同时启动 Bridge。Foxglove 中选择 **Foxglove WebSocket**，
连接 `ws://视觉主机IP:8765`（本机使用 `ws://localhost:8765`），添加 **Topic Graph** 面板看节点和话题，
添加 **Image** 面板选择 `/image_raw`，或添加 **3D** 面板查看 `/tf` 和 `/tf_static`。
当前使用话题订阅相机模式，默认关闭本地 OpenCV 窗口，适合远程调试。
Docker 使用 host 网络，Bridge 无需额外映射端口。
当前容器内的 Bridge 在 ROS 图发生变化时更新节点图；若首次连接时 Topic Graph 为空，
先在 Image 面板订阅 `/image_raw`，使节点连接图刷新。

仅启动 Bridge（已有自瞄进程时使用，避免同一端口重复启动）：

```bash
ros2 launch rm_bringup foxglove_bridge.launch.py
# 需要另一个端口时：
ros2 launch rm_bringup foxglove_bridge.launch.py port:=8766
```

Bridge 参数保存在 `config/node_params/foxglove_bridge_params.yaml`，
其中 `connectionGraph` 能力用于节点图。完整 MVS SDK 挂载在 `/opt/MVS` 时，
正式 launch 会自动优先加载其网口相机传输库。

```bash
# 使用 pixi 的话用这行编译
pixi run build
# 编译
# 本仓库包含的功能包过多，建议限制同时编译的线程数
# 推荐 8G 2线程 ，16G 4线程, 32G不用限制
colcon build --symlink-install --parallel-workers 4 

# 运行
source install/setup.bash
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

默认日志和内录视频路径为`qd2026-log/`

## 四、自启动

### Docker

注释`docker-compose.yaml`里的`command`

修改前
```bash
    command: bash
    # command: ros2 launch foxglove_bridge foxglove_bridge_launch.xml # foxglove 调试
    # command: bash -c "source /ros_ws/src/rm_upstart/rm_watch_dog.sh" # 自启动
```

修改后
```bash
    # command: bash
    # command: ros2 launch foxglove_bridge foxglove_bridge_launch.xml # foxglove 调试
    command: bash -c "source /ros_ws/src/rm_upstart/rm_watch_dog.sh" # 自启动
```

### Local

- 编译程序后，进入rm_upstart文件夹

```bash
cd rm_upstart
```

- 修改**rm_watch_dog.sh**中的`NAMESPACE`（ros命名空间）、`NODE_NAMES`（需要看门狗监控的节点）和`WORKING_DIR` （代码路径）

- 注册服务
  
```bash
sudo chmod +x ./register_service.sh
sudo ./register_service.sh

# 正常时有如下输出
# Creating systemd service file at /etc/systemd/system/rm.service...
# Reloading systemd daemon...
# Enabling service rm.service...
# Starting service rm.service...
# Service rm.service has been registered and started.
```

- 查看程序状态

```bash
systemctl status rm
```

- 查看终端输出

```
查看screen.output或qd2026-log下的日志
```  

- 关闭程序

```bash
systemctl stop rm
```

- 取消自启动

```bash
systemctl disable rm
```


## 开源许可证

```
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

## 致谢

感谢以下开源项目：

- [FYT2024_vision](https://github.com/CSU-FYT-Vision/FYT2024_vision) fyt2024是本项目的基础，提供了一套可参考的，规范、易用、高效的视觉算法框架
- [北洋机甲2024视觉算法库](https://github.com/HHgzs/OpenRM-2024) 提供了三分法代码结构
- [上海交通大学](https://github.com/julyfun/rm.cv.fans) 提供三分法代价计算的代码和双模型切换思路
- [sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25) 提供 EKF 前哨战适配思路
- [深圳大学](https://github.com/broalantaps/RobotDetectionModel) 提供神经网络模型

## 更新日志

- 移植上交提出的三分法求YAW
- 加入录包功能
- 加入双头功能
- 制作了docker容器
- 加入平移模型
- 改成同济的 EKF 并适配新前哨

## 未来计划

- [ ] fix:平移模型时打中一块 armor 会转到另一块 armor 打（原因是打中后装甲板灯灭了）
- [ ] fix:马氏距离匹配没有设置允许通过阈值，只按最小的跟踪
- [ ] fix:装甲板在边缘旋转时匹配出错，做 nis 发散检测重置滤波器
- [ ] 测试车在坡上、存在 roll 、枪管装歪对自瞄的影响，判断是否需要更全面的坐标系建模（云台、底盘、枪管）
- [ ] fix:整理线程，优化性能调度
- [ ] 测试 龙格-库塔（RK4） 弹道解算
- [ ] feat:加入能量机关自瞄
- [ ] feat:加入同济 sp_vison_2025 开源出的 MPC轨迹规划
- [ ] refactor:重构双相机共头代码，改用 FOV 重叠切换相机逻辑
- [ ] feat:加入 robot 选择功能，操作手能选第一优先级 robot
- [ ] refactor: 重构全向感知逻辑，更具鲁棒性，便于调试、观测
- [ ] 测试 [jlu_vision_26](https://github.com/Fskaaaaaaaa/jlu_vision_26) 提出的 uv 系作为 EKF 观测量坐标系
- [ ] feat:加入网页前端调试，降低部署门槛
- [ ] refactor:重构整套代码，移除 ros 核心依赖，ros 只作为外部调试接口使用

## slirute/qidian

- `2.5`:加入Intel GPU驱动和 `rsync`
- `2.4`:改用`ros-humble-desktop`以使用rviz，加入`plotjuggler`和`tf-transformations`包
- `2.3`：移除`G2O` 和 `Sophus` 库
- `2.2`：完善 ARM 支持与启动脚本自动化
- `2.1`：引入多架构支持（AMD64/ARM64）与多阶段构建
- `2.0`:仅针对`FYT2024_vision`制作的镜像

## 开发实用技巧

### rsync

使用`rsync`同步文件

```bash
rsync -rlptzvP --delete \
--exclude='build/' \
--exclude='install/' \
--exclude='log/' \
--exclude='MvSDKLog/' \
--exclude='qd2026-log/' \
--exclude='record/' \
--exclude='rosbag/' \
--exclude='watchdog/' \
./ qidian@192.168.137.x:~/rmvision2025/
```

### 进入容器

在`~/.bashrc`中加入以下内容，每次进入容器只需要输入`rv`即可进入

```bash
echo "alias rv='docker exec -it rv_devel_ bash'" >> ~/.bashrc 
```

### commit 规范

- 提交信息应包含：类型 (Type)、描述 (Subject) 以及可选的 范围（scope）、正文 (Body) 和 脚注 (Footer)

```
<type>(<scope>): <subject>

<body>

<footer>
```

- 常用类型 (Type)

    | 类型 | 描述 |
    | :--- | :--- | 
    | **feat** | 新增功能 (feature) |
    | **fix** | 修复 Bug |
    | **docs** | 文档修改 (Documentation) | 
    | **style** | 代码格式修改 (不影响逻辑，如空格、分号等) | 
    | **refactor** | 代码重构 (既不是修复 Bug 也不是新增功能) | 
    | **perf** | 性能优化 (Performance) | 
    | **test** | 增加或修改测试用例 | 
    | **chore** | 构建过程、辅助工具或依赖库的变动 | 
