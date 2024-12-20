import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    IfElseSubstitution,
)
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.conditions import IfCondition


def generate_launch_description():
    # Set environment variables
    package_root = get_package_share_directory('arena_bringup')
    ss_root = get_package_share_directory('arena_simulation_setup')

    # Set paths for Gazebo, Physics Engine, and Resource

    # GZ_CONFIG_PATHS = [
    #     # os.path.join(get_package_share_directory('gz-sim8'), "gz"),
    #     # os.path.join(workspace_root, 'install', 'gz-tools2', 'share', 'gz'),
    # ]

    # GZ_SIM_PHYSICS_ENGINE_PATH = os.path.join(
    #     workspace_root, "build", "gz-physics7"
    # )

    GZ_SIM_RESOURCE_PATHS = [
        os.path.join(ss_root, "configs", "gazebo"),
        os.path.join(ss_root, "entities"),
        os.path.join(ss_root, "worlds"),
        os.path.join(ss_root, "gazebo_models"),
        os.path.join(ss_root, "entities", "obstacles", "static"),
        os.path.join(ss_root, "entities", "obstacles", "robots"),
        os.path.join(
            ss_root,
            "gazebo_models",
            "Cafe table",
            "materials",
            "textures",
        ),
        os.path.join(get_package_share_directory("jackal_description"), '..'),
    ]
    # GZ_CONFIG_PATH = ":".join(GZ_CONFIG_PATHS)
    GZ_CONFIG_PATH = "/usr/share/gz"

    GZ_SIM_RESOURCE_PATHS_COMBINED = ":".join(GZ_SIM_RESOURCE_PATHS)

    # Update environment variables
    # os.environ['GZ_CONFIG_PATH'] = GZ_CONFIG_PATH
    # os.environ["GZ_CONFIG_PATH"] = GZ_CONFIG_PATH
    # os.environ["GZ_SIM_PHYSICS_ENGINE_PATH"] = GZ_SIM_PHYSICS_ENGINE_PATH
    os.environ["GZ_SIM_RESOURCE_PATH"] = GZ_SIM_RESOURCE_PATHS_COMBINED

    world = LaunchConfiguration("world")

    world_path = IfElseSubstitution(
        condition=PythonExpression(['"', world, '" == ""']),
        if_value=PathJoinSubstitution(
            [
                package_root,
                'configs',
                'gazebo',
                'empty.sdf',
            ]
        ),
        else_value=PathJoinSubstitution(
            [
                ss_root,
                "worlds",
                world,
                "worlds",
                PythonExpression(['"', world, '.world"']),
            ]
        )
    )

    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_model = LaunchConfiguration("model")
    random_spawn_test = LaunchConfiguration("random_spawn_test")

    # Gazebo launch
    gz_sim_launch_file = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_file),
        launch_arguments={
            "gz_args": [
                world_path,
                " -v 4",
                " -r",
                " --render-engine ogre",
            ],
            "physics-engine": "gz-physics-dartsim",
        }.items(),
    )

    # # Robot URDF (Xacro) description
    # robot_desc_path = os.path.join(
    #     workspace_root,
    #     "src",
    #     "arena",
    #     "arena_simulation_setup",
    #     "entities",
    #     "robots",
    #     "jackal",
    #     "urdf",
    #     "jackal.urdf.xacro",
    # )

    # # Process the robot description file using xacro
    # # doc = xacro.process_file(robot_desc_path, mappings={'use_sim': 'true'})
    # # robot_description = doc.toprettyxml(indent='  ')
    # # Process the robot description file using xacro
    # robot_description = xacro.process_file(
    #     robot_desc_path, mappings={"use_sim": "true"}
    # ).toxml()

    # # Robot State Publisher
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="both",
    #     parameters=[
    #         {"use_sim_time": use_sim_time},
    #         {"robot_description": robot_description},
    #     ],
    # )

    # # Joint State Publisher
    # joint_state_publisher = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     name="joint_state_publisher",
    #     parameters=[{"use_sim_time": use_sim_time}],
    # )

    # # Spawn the robot into the Gazebo simulation
    # spawn_robot = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     output="screen",
    #     # arguments=[
    #     #     '-world', 'default',
    #     #     '-string', robot_description,
    #     #     '-name', robot_model,
    #     #     '-allow_renaming', 'false',
    #     #     '-x', '0',
    #     #     '-y', '0',
    #     #     '-z', '0',
    #     # ],
    #     parameters=[
    #         {
    #             "world": "default",
    #             "string": robot_description,
    #             "name": robot_model,
    #             "allow_renaming": False,
    #             "topic": 'robot_description',
    #         }
    #     ],
    # )

    # Bridge configuration
    bridge_config = os.path.join(
        package_root,
        "launch",
        "testing",
        "simulators",
        "gazebo_bridge.yaml",
    )

    world_state_converter = Node(
        package="ros_gz_bridge",
        executable="world_state_to_odom",
        name="world_state_to_odom",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_name": robot_model,
                "world_frame": world,
                "robot_frame": "base_link",
                "odom_frame": "odom",
            }
        ],
    )

    # random_spawn_launch_file = PathJoinSubstitution(
    #     [
    #         workspace_root,
    #         "src",
    #         "arena",
    #         "arena-rosnav",
    #         "arena_bringup",
    #         "launch",
    #         "testing",
    #         "simulators",
    #         "gazebo_entity_spawn.py",
    #     ]
    # )

    # random_spawn_spawn = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(random_spawn_launch_file)
    # )

    # Return the LaunchDescription with all the nodes/actions

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value='True',
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "world",
                default_value='',
                description="World name",
            ),
            DeclareLaunchArgument(
                "model", default_value="jackal", description="Robot model name"
            ),
            SetEnvironmentVariable("GZ_CONFIG_PATH", GZ_CONFIG_PATH),
            # SetEnvironmentVariable(
            #     "GZ_SIM_PHYSICS_ENGINE_PATH", GZ_SIM_PHYSICS_ENGINE_PATH
            # ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", GZ_SIM_RESOURCE_PATHS_COMBINED
            ),
            gazebo,
            # robot_state_publisher,
            # joint_state_publisher,
            # spawn_robot,
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(random_spawn_launch_file),
            #     condition=IfCondition(random_spawn_test),
            # ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
