import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory

sys.path.append(
    os.path.join(get_package_share_directory("vision_bringup"), "launch")
)

def generate_launch_description():
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from ament_index_python.packages import get_package_share_directory
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown

    launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('vision_bringup'), 'config', 'launch_params.yaml')))

    node_params = os.path.join(
    get_package_share_directory('vision_bringup'), 'config', 'node_params.yaml')

    def get_camera_node(package, plugin):
            return ComposableNode(
                package=package,
                plugin=plugin,
                name="camera_node",
                parameters=[node_params],
                extra_arguments=[{"use_intra_process_comms": True}],
            )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name="camera_detector_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                camera_node,
                ComposableNode(
                    package="detector",
                    plugin="DetectorNode",
                    name="detector",
                    parameters=[node_params],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
            output="both",
            emulate_tty=True,
            ros_arguments=[
                "--ros-args",
                "--log-level",
                "detector:=" + launch_params["detector_log_level"],
            ],
            on_exit=Shutdown(),
        )
    mv_camera_node = get_camera_node(
        "mindvision_camera", "mindvision_camera::MVCameraNode"
    )
    cam_detector = get_camera_detector_container(mv_camera_node)
    # hik_camera_node = get_camera_node(
    #     "hik_camera", "hik_camera::HikCameraNode"
    # )

    # cam_detector = get_camera_detector_container(hik_camera_node)

    serial_driver_node = Node(
        package="rm_serial_driver",
        executable="rm_serial_driver_node",
        name="serial_driver",
        output="both",
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
        ros_arguments=[
            "--ros-args",
            "--log-level",
            "serial_driver:=" + launch_params["serial_log_level"],
        ],
    )

    delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )
        
    return LaunchDescription([
        cam_detector,
        # image_view_node,
        delay_serial_node  
    ])