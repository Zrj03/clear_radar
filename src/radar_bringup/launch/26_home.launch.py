from launch import LaunchDescription, actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from tf2_geometry_msgs.tf2_geometry_msgs import _get_quat_from_mat, _build_affine, _decompose_affine

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription

import numpy as np
import os

# 调试开关：设置为 True 时将不使用容器组合方式，便于单节点调试
debug = False

# 节点参数文件路径，指向 radar_bringup 包下的配置文件
# 修改此变量可切换整套运行参数（例如 home vs national）
node_params = os.path.join(
        get_package_share_directory('radar_bringup'),
        'config',
        'config.26.home.yaml'
)


def get_xyzw_tf_broadcaster(cali: list, fr: str, child_fr: str):
    """
    根据给定的位姿四元数/平移（x,y,z,qx,qy,qz,qw）生成一个静态 TF 发布节点。
    参数:
    - cali: 包含 [x, y, z, qx, qy, qz, qw] 的列表
    - fr: 父坐标系名称（frame id）
    - child_fr: 子坐标系名称（child frame id）
    返回:
    - 一个启动 `static_transform_publisher` 的 `Node` 对象，用于将静态变换发布到 TF 树中。
    """
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='radar',
        name=fr+'_to_'+child_fr,
        arguments=['--x', str(cali[0]),
                   '--y', str(cali[1]),
                   '--z', str(cali[2]),
                   '--qx', str(cali[3]),
                   '--qy', str(cali[4]),
                   '--qz', str(cali[5]),
                   '--qw', str(cali[6]),
                   '--frame-id', fr,
                   '--child-frame-id', child_fr],)


def get_matrix_tf_broadcaster(cali: np.array, fr: str, child_fr: str):
    """
    接受 4x4 仿射矩阵（numpy array），分解为平移和四元数后发布静态 TF。
    参数:
    - cali: 4x4 仿射变换矩阵，最后一行为 [0,0,0,1]
    - fr: 父坐标系名称
    - child_fr: 子坐标系名称
    用途:
    - 方便从外部计算好的仿射矩阵直接生成 TF 发布，而不必手动拆解为 xyzw/q。
    """
    quat, trans = _decompose_affine(cali)
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='radar',
        name=fr+'_to_'+child_fr,
        arguments=['--x', str(trans[0]),
                   '--y', str(trans[1]),
                   '--z', str(trans[2]),
                   '--qw', str(quat[0]),
                   '--qx', str(quat[1]),
                   '--qy', str(quat[2]),
                   '--qz', str(quat[3]),
                   '--frame-id', fr,
                   '--child-frame-id', child_fr],)


def get_vision_container(cam_name: str, sn: str, camera_info_url: str):
        """
        构建并返回 камера 相关的容器或单独节点。
        - 非调试模式下，使用 `ComposableNodeContainer` 将 `hik_camera` 和 `img_recognizer` 两个可组合节点放入同一进程容器，
            并开启多线程执行器以提高性能。
        - 调试模式下（debug=True），以普通节点形式单独启动，便于在断点或日志中单步调试单节点。
        参数:
        - cam_name: 相机命名空间（例如 hik_6mm）
        - sn: 相机序列号，用于驱动识别特定硬件
        - camera_info_url: 相机内参 YAML 的 URL
        """
        if not debug:
            return (ComposableNodeContainer(
                name=cam_name + '_vision_container',
                namespace='radar',
                package='rclcpp_components',
                executable='component_container_isolated',
                arguments=['--use_multi_threaded_executor'],
                composable_node_descriptions=[
                    ComposableNode(
                        package='hik_camera',
                        plugin='hik_camera::HikCameraNode',
                        name='hik_camera',
                        namespace='radar/' + cam_name,
                        parameters=[{
                            'camera_info_url': camera_info_url,
                            'sn': sn
                        }],
                        extra_arguments=[{'use_intra_process_comms': True}]
                    ),
                    ComposableNode(
                        package='img_recognizer',
                        plugin='img_recognizer::RecognizerNode',
                        name='img_recognizer',
                        namespace='radar/' + cam_name,
                        parameters=[node_params],
                        extra_arguments=[{'use_intra_process_comms': True}]
                    ),
                ],
                output='both',
                emulate_tty=True,
                on_exit=Shutdown(),
            ),)
        else:
            return (Node(
                    package='hik_camera',
                    executable='hik_camera_node',
                    namespace='radar/' + cam_name,
                    parameters=[node_params, {
                        'camera_info_url': camera_info_url,
                        'sn': sn
                    }],
                    ),
                    Node(
                    package='img_recognizer',
                    executable='img_recognizer_node',
                    namespace='radar/' + cam_name,
                    parameters=[node_params],
                    ),
                    )


def get_pc_container():
        """
        构建点云处理链（Lidar 驱动 + pc_detector）以及可选的 `nn_detector` 节点。
        - 非调试模式下，将 `livox_v1_lidar` 和 `pc_detector` 放入可组合容器以减少进程通信开销，
            并在容器外启动 `nn_detector`（神经网络增强检测）节点，且做必要的话题重映射。
        - 调试模式下以单独节点形式启动，方便观察每个节点的日志输出。
        返回值: 对应要传递给 LaunchDescription 的 node/containers 列表
        """
        if not debug:
            return (ComposableNodeContainer(
                name='pc_container',
                namespace='radar',
                package='rclcpp_components',
                executable='component_container_isolated',
                arguments=['--use_multi_threaded_executor'],
                composable_node_descriptions=[
                    ComposableNode(
                        package='livox_v1_lidar',
                        plugin='livox_v1_lidar::LidarPublisher',
                        name='livox_v1_lidar',
                        namespace='radar/' + 'lidar_mid70',
                        parameters=[node_params],
                        extra_arguments=[{'use_intra_process_comms': False}]
                    ),
                    ComposableNode(
                        package='pc_detector',
                        plugin='pc_detector::DetectorNode',
                        name='pc_detector',
                        namespace='radar',
                        parameters=[node_params],
                        extra_arguments=[{'use_intra_process_comms': False}]
                    ),
                ],
                output='both',
                emulate_tty=True,
                on_exit=Shutdown(),
            ),
            Node(
                package='nn_detector',
                executable='nn_detector_node',
                name='nn_detector',
                namespace='radar',
                parameters=[node_params],
                remappings=[
                    ('lidar_mid70/livox/pointcloud', 'pc_detector/pc_filtered'),
                ],
                output='both',
            ),)
        else:
            return (
                Node(
                    package='livox_v1_lidar',
                    executable='livox_v1_lidar_node',
                    name='livox_v1_lidar',
                    namespace='radar/' + 'lidar_mid70',
                    parameters=[node_params],
                ),
                Node(
                    package='pc_detector',
                    executable='pc_detector_node',
                    name='pc_detector',
                    namespace='radar',
                    parameters=[node_params],
                ),
                Node(
                    package='nn_detector',
                    executable='nn_detector_node',
                    name='nn_detector',
                    namespace='radar',
                    parameters=[node_params],
                    remappings=[
                        ('lidar_mid70/livox/pointcloud', 'pc_detector/pc_filtered'),
                    ],
                ),
            )


def generate_launch_description():
    """
    生成并返回 LaunchDescription 对象，描述本次启动所需的所有节点与静态 TF。
    主要包含：
    - 视觉容器 (`hik_camera` + `img_recognizer`)
    - 雷达容器 (`livox_v1_lidar` + `pc_detector`) 以及 `nn_detector`
    - 一些静态 TF 发布器（相机到雷达的外参等）
    - `pc_aligner`：点云对齐（手动对齐模式）
    - 可视化与结果输出：`marker_pub`, `target_visualizer`, `result_visualizer`
    注意：在 home 模式下默认启用手动配准（由 `pc_aligner` 提供），不会自动加载比赛场地的 6 点标定。
    如果回放赛事 rosbag，请使用 `use_sim_time:=true` 参数并播放 bag 时带 `--clock`。
    """
    return LaunchDescription([
        *get_vision_container(
            'hik_6mm', 'DB0108949', 'package://hik_camera/config/6mm.yaml'),
        get_xyzw_tf_broadcaster(
            [
                -0.15269233258535683,
                1.2123996239366319,
                -0.36915186605897177,
                0.44803863873193683,
                -0.5183558155623317,
                0.5523947175419749,
                -0.47479332630910565
            ], 'lidar_mid70_frame', 'hik_6mm_frame'
        ),
        *get_pc_container(),
        Node(
            package='radar_utils',
            executable='marker_pub',
            namespace='radar',
            parameters=[{"mesh": "home_demo.stl"}],
            output='both',
        ),
        Node(
            package='pc_aligner',
            executable='pc_aligner',
            namespace='radar',
            parameters=[node_params],
            output='both',
        ),
        Node(
            package='target_matcher',
            executable='target_matcher',
            namespace='radar',
            parameters=[node_params],
            output='both',
        ),
        Node(
            package='target_multiplexer',
            executable='target_multiplexer',
            namespace='radar',
            parameters=[node_params],
            output='both',
        ),
        Node(
            package='dv_trigger',
            executable='dv_trigger',
            namespace='radar',
            parameters=[node_params],
            output='both',
        ),
        Node(
            package='radar_supervisor',
            executable='radar_supervisor',
            namespace='radar',
            parameters=[node_params],
            output='both',
        ),
        # # judge_bridge 有串口依赖，若无裁判系统请注释此节点
        # Node(
        #     package='judge_bridge',
        #     executable='judge_bridge',
        #     namespace='radar',
        #     parameters=[node_params],
        #     output='both',
        # ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
        ),
        Node(
            package='target_visualizer',
            executable='target_visualizer',
            namespace='radar',
            output='both',
        ),
        Node(
            package='result_visualizer',
            executable='result_visualizer',
            namespace='radar',
            output='both',
        ),
        Node(
            package='radar_utils',
            executable='camera_param_tuner',
            name='camera_param_tuner',
            output='both',
        ),
    ])
