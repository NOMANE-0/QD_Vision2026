import os
import subprocess
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction

sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))

def ensure_camera_network(network_config):
    """确保网口相机使用的网卡具有指定的本机地址。"""
    interface = network_config['interface']
    address = network_config['address']
    try:
        result = subprocess.run(
            ['ip', '-o', '-4', 'addr', 'show', 'dev', interface],
            check=True, capture_output=True, text=True)
        if any(line.split()[3] == address for line in result.stdout.splitlines()):
            return
        subprocess.run(
            ['ip', 'addr', 'add', address, 'dev', interface],
            check=True, capture_output=True, text=True)
        print(f'[rm_bringup] Added {address} to {interface} for the GigE camera.')
    except (OSError, subprocess.CalledProcessError) as exc:
        detail = exc.stderr.strip() if isinstance(exc, subprocess.CalledProcessError) else str(exc)
        raise RuntimeError(
            f'Failed to configure GigE camera network ({interface}, {address}): {detail}'
        ) from exc

def generate_launch_description():

    # 载入参数
    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('rm_bringup'), 'config', 'launch_params.yaml')))

    # replay 模式自身会回放图像和串口，优先级最高，避免和其它输入源冲突
    if launch_params.get('replay', False):
        launch_params['use_state_machine_camera'] = False
        launch_params['video_player'] = False
        launch_params['virtual_serial'] = False

    # 使用实体相机时，在驱动加载前配置网口相机所在的网段。
    use_physical_camera = not launch_params.get('replay', False) and (
        launch_params['use_state_machine_camera'] or not launch_params.get('video_player', False))
    camera_network = launch_params.get('camera_network', {})
    if use_physical_camera and camera_network.get('enabled', False):
        ensure_camera_network(camera_network)

    if launch_params.get('record', False) and launch_params.get('auto_record', False):
        print('[rm_bringup] record and auto_record cannot both be true; disabling record.')
        launch_params['record'] = False

    # 命名空间处理
    node_namespace = launch_params['namespace']

    # 定义获取参数文件的辅助函数
    def get_params(name):
        path = os.path.join(get_package_share_directory('rm_bringup'), 'config', 'node_params', '{}_params.yaml'.format(name))
        # 相机配置同时供状态机读取；以字典传入 ROS，展开为 camera_6mm.* 参数。
        if name == 'camera_driver':
            with open(path, encoding='utf-8') as config_file:
                return yaml.safe_load(config_file)
        return path

    # --- 准备 ComposableNode 列表 ---
    composable_nodes = []

    # 1. 机器人模型发布 (Robot State Publisher)
    # 这一步通常可以使用官方的 robot_state_publisher::RobotStatePublisher 组件
    robot_gimbal_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_robot_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
        ' xyz:=', launch_params['gimbal2camera']['xyz'], ' rpy:=', launch_params['gimbal2camera']['rpy'],
        ' xyz_:=', launch_params['gimbal2camera']['xyz_'], ' rpy_:=', launch_params['gimbal2camera']['rpy_']
    ])
    
    composable_nodes.append(ComposableNode(
        package='robot_state_publisher',
        plugin='robot_state_publisher::RobotStatePublisher', # 官方组件类名
        name='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_gimbal_description, value_type=str),
                    'publish_frequency': 1000.0}]
    ))

    if launch_params['navigation']:
        robot_navigation_description = Command(['xacro ', os.path.join(
            get_package_share_directory('rm_robot_description'), 'urdf', 'sentry.urdf.xacro')])
        composable_nodes.append(ComposableNode(
            package='robot_state_publisher',
            plugin='robot_state_publisher::RobotStatePublisher',
            name='nav_state_publisher', # 避免重名
            parameters=[{'robot_description': ParameterValue(robot_navigation_description, value_type=str)}]
        ))

    # 2. 相机驱动 (Camera)
    if not launch_params['use_state_machine_camera'] and not launch_params['replay']:
        if launch_params.get('video_player', False):
            composable_nodes.append(ComposableNode(
                package='hik_camera',
                plugin='qd::camera_driver::VideoPlayerNode',
                name='video_player',
                namespace=node_namespace,
                parameters=[get_params('video_player')],
                extra_arguments=[{'use_intra_process_comms': True}]
            ))
        else:
            composable_nodes.append(ComposableNode(
                package='hik_camera',
                plugin='hik_camera::HikCameraNode',
                name='camera_driver',
                namespace=node_namespace,
                parameters=[get_params('camera_driver')],
                extra_arguments=[{'use_intra_process_comms': True}]
            ))
    # 3. 串口驱动 (Serial Driver)
    if not launch_params['replay']:
        if launch_params.get('virtual_serial', False):
            composable_nodes.append(ComposableNode(
                package='rm_serial_driver',
                plugin='qd::serial_driver::VirtualSerialNode',
                name='virtual_serial',
                namespace=node_namespace,
                parameters=[get_params('virtual_serial'),
                            {'has_rune': launch_params.get('rune', False)}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ))
        else:
            serial_driver_params = [get_params('serial_driver')]
            if not launch_params.get('rune', True):
                serial_driver_params.append({'protocol': 'hero'})

            composable_nodes.append(ComposableNode(
                package='rm_serial_driver',
                plugin='qd::serial_driver::SerialDriverNode',
                name='serial_driver',
                namespace=node_namespace,
                parameters=serial_driver_params,
                extra_arguments=[{'use_intra_process_comms': True}]
            ))
    # 4. 装甲板识别 (Armor Detector)
    composable_nodes.append(ComposableNode(
        package='armor_detector', 
        plugin='qd::auto_aim::ArmorDetectorNode',
        name='armor_detector',
        namespace=node_namespace,
        parameters=[get_params('armor_detector'), 
                    get_params('camera_driver'), 
                    {"use_state_machine_camera": launch_params['use_state_machine_camera'],
                    "enable_record": launch_params['record']
                    }
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    ))

    # 5. 装甲板解算 (Armor Solver)
    if launch_params['hero_solver']:
        composable_nodes.append(ComposableNode(
            package='hero_armor_solver',
            plugin='qd::auto_aim::ArmorSolverNode',
            name='armor_solver',
            parameters=[get_params('armor_solver')],
            extra_arguments=[{'use_intra_process_comms': True}]
        ))
    else:
        composable_nodes.append(ComposableNode(
            package='armor_solver',
            plugin='qd::auto_aim::ArmorSolverNode',
            name='armor_solver',
            namespace=node_namespace,
            parameters=[get_params('armor_solver')],
            extra_arguments=[{'use_intra_process_comms': True}]
        ))

    if launch_params['rune']:
        # 6. 打符 (Rune)
        composable_nodes.append(ComposableNode(    
            package='rune_detector',
            plugin='qd::rune::RuneDetectorNode',
            name='rune_detector',
            namespace=node_namespace,
            parameters=[get_params('rune_detector')],
            extra_arguments=[{'use_intra_process_comms': True}]
        ))
        composable_nodes.append(ComposableNode(
            package='rune_solver',
            plugin='qd::rune::RuneSolverNode',
            name='rune_solver',
            namespace=node_namespace,
            parameters=[get_params('rune_solver')],
            extra_arguments=[{'use_intra_process_comms': True}]
        ))

    if launch_params['replay']:
        # 7. 自动回放 (Auto Replay)
        composable_nodes.append(ComposableNode(
            package='rm_auto_replay',
            plugin='qd::auto_replay::AutoReplayNode',
            name='auto_replay',
            namespace=node_namespace,
            parameters=[get_params('auto_replay')],
            extra_arguments=[{'use_intra_process_comms': True}]
        ))

    if launch_params.get('auto_record', False):
        auto_record_params = os.path.join(
            get_package_share_directory('rm_auto_record'), 'config', 'node_params.yaml')
        composable_nodes.append(ComposableNode(
            package='rm_auto_record',
            plugin='rm_auto_record::RecordNode',
            name='rm_auto_record',
            namespace=node_namespace,
            parameters=[auto_record_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        ))

    # --- 创建唯一的容器 ---
    # component_container_mt 支持多线程，适合图像和解算并行处理
    container = ComposableNodeContainer(
        name='rm_container',
        package='rclcpp_components',
        executable='component_container_mt', # 使用多线程容器
        namespace='',
        composable_node_descriptions=composable_nodes,
        output='both',
        emulate_tty=True,
        ros_arguments=['--ros-args', '--log-level', 'info'],
    )

    # 环境变量
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '0')

    actions = [
        set_domain_id,
    ]
    # 完整 MVS SDK 提供网口相机传输层；优先于仓库中仅含 USB 的 SDK 库。
    sdk_path = '/opt/MVS/lib/64'
    if os.path.isdir(sdk_path):
        actions.append(SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            ':'.join([sdk_path, os.path.join(sdk_path, 'ThirdParty'),
                      os.environ.get('LD_LIBRARY_PATH', '')])
        ))
    actions.append(container)
    if launch_params.get('enable_foxglove_bridge', False):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rm_bringup'), 'launch',
                'foxglove_bridge.launch.py')),
            launch_arguments={'namespace': node_namespace}.items()
        ))
    if launch_params.get('enable_dynamic_camera_tf', True):
        # 动态 TF 调参节点（用于调试相机安装的 rpy 误差）
        # 使用 ros2 param set /dynamic_camera_tf pitch 0.02 来调整
        actions.append(Node(
            package='rm_bringup',
            executable='dynamic_camera_tf.py',
            name='dynamic_camera_tf',
            parameters=[{
                'x': float(launch_params['gimbal2camera']['xyz'].strip('"').split()[0]),
                'y': float(launch_params['gimbal2camera']['xyz'].strip('"').split()[1]),
                'z': float(launch_params['gimbal2camera']['xyz'].strip('"').split()[2]),
                'roll': float(launch_params['gimbal2camera']['rpy'].strip('"').split()[0]),
                'pitch': float(launch_params['gimbal2camera']['rpy'].strip('"').split()[1]),
                'yaw': float(launch_params['gimbal2camera']['rpy'].strip('"').split()[2]),
            }],
            output='screen'
        ))

    if launch_params.get('enable_pitch_calibration', False):
        # gimbal2camera pitch 自动标定节点
        # 采集数据后自动搜索最优 pitch，结果输出到控制台
        actions.append(Node(
            package='rm_bringup',
            executable='calibrate_gimbal2camera_pitch.py',
            name='gimbal2camera_pitch_calibrator',
            namespace=node_namespace,
            parameters=[{
                'sample_duration': 30.0,
                'gimbal2camera_x': float(launch_params['gimbal2camera']['xyz'].strip('"').split()[0]),
                'gimbal2camera_y': float(launch_params['gimbal2camera']['xyz'].strip('"').split()[1]),
                'gimbal2camera_z': float(launch_params['gimbal2camera']['xyz'].strip('"').split()[2]),
                'gimbal2camera_pitch': float(launch_params['gimbal2camera']['rpy'].strip('"').split()[1]),
                'pitch_min': -0.3,
                'pitch_max': 0.3,
                'pitch_step': 0.001,
            }],
            output='screen'
        ))
    return LaunchDescription(actions)
