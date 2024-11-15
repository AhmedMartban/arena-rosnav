import os
import random
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_random_position():
    """Generate random positions within a specific range."""
    x = random.uniform(-5.0, 5.0)
    y = random.uniform(-5.0, 5.0)
    z = 0.0  
    return x, y, z

def generate_spawn_node(index, position):
    """Generate a spawn node for pedestrian"""
    agent_config = {
        'name': f'pedestrian_{index}',
        'skin': random.randint(0, 4),
    }
    
    # Create SDF for pedestrian
    sdf = f"""<?xml version="1.0" ?>
    <sdf version="1.6">
        <model name="{agent_config['name']}">
            <static>false</static>
            <pose>0 0 1.0 0 0 0</pose>
            <link name="link">
                <visual name="visual">
                    <geometry>
                        <mesh>
                            <uri>package://hunav_rviz2_panel/meshes/casual_man.dae</uri>
                        </mesh>
                    </geometry>
                </visual>
                <collision name="collision">
                    <geometry>
                        <cylinder>
                            <radius>0.3</radius>
                            <length>1.7</length>
                        </cylinder>
                    </geometry>
                </collision>
            </link>
        </model>
    </sdf>"""
    
    return Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', sdf,
            '-name', f'pedestrian_{index}',
            '-x', str(position[0]),
            '-y', str(position[1]), 
            '-z', str(position[2])
        ]
    )

def generate_launch_description():
    # Launch Arguments
    num_pedestrians = LaunchConfiguration('num_pedestrians')
    
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'num_pedestrians',
            default_value='5',
            description='Number of pedestrians to spawn'
        )
    ]
    
    spawn_nodes = []
    # Generate spawn nodes für pedestrians - fixed number for now
    for i in range(5):  # Fixed number instead of using LaunchConfiguration
        x, y, z = generate_random_position()
        spawn_nodes.extend([
            LogInfo(msg=f'Spawning pedestrian {i} at position ({x}, {y}, {z})'),
            generate_spawn_node(i, (x, y, z))
        ])

    return LaunchDescription(launch_args + spawn_nodes)