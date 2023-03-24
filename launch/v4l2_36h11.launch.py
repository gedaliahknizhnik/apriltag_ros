import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    device = 4

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name="apriltag_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="v4l2_camera",
                plugin="v4l2_camera::V4L2Camera",
                name="camera",
                namespace="v4l2",
                parameters=[{"video_device": f"/dev/video{device}"}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify",
                namespace="v4l2",
                remappings=[("image", "image_raw")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="apriltag_ros",
                plugin="AprilTagNode",
                name="apriltag",
                namespace="apriltag",
                remappings=[
                    ("image_rect", "/v4l2/image_rect"),
                    ("camera_info", "/v4l2/camera_info"),
                ],
                parameters=[{"history": "keep_last", "size": 0.108, "tf_child": True}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="image_tools",
                plugin="image_tools::ShowImage",
                name="showimage",
                remappings=[("/image", "/v4l2/image_rect")],
                # parameters=[{"$(find-pkg-share apriltag_ros)/cfg/tags_36h11.yaml"}],
                # extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="both",
        emulate_tty=True,
    )

    fixed_apriltags = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.606",
                "1.417",
                "-0.051",
                "0",
                "0",
                "0",
                "world",
                "tag36h11:1",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.086", "1.184", "0", "0", "0", "0", "world", "tag36h11:2"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["1.349", "1.222", "0", "0", "0", "0", "world", "tag36h11:3"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.648",
                "0.803",
                "-0.089",
                "0",
                "0",
                "0",
                "world",
                "tag36h11:4",
            ],
        ),
    ]

    return launch.LaunchDescription(
        [
            fixed_apriltags[0],
            fixed_apriltags[1],
            fixed_apriltags[2],
            fixed_apriltags[3],
            container,
        ]
    )
