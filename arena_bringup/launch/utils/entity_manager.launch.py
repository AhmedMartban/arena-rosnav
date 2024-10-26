import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='entity_manager',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='sfm',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='world_file',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='scene_file',
            default_value=launch.substitutions.LaunchConfiguration(
                'world_file')
        ),
        launch.actions.DeclareLaunchArgument(
            name='pedsim',
            default_value="$(eval entity_manager in ('pedsim', 'crowdsim'))"
        ),
        # Add Hunavsim argument
        launch.actions.DeclareLaunchArgument(
            name='hunavsim',
            default_value="$(eval entity_manager == 'hunavsim')"
        ),
        
        # Include Hunavsim launch file if hunavsim is selected
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('arena_bringup'),
                    'launch', 'utils', 'hunavsim.launch.py')
            ),
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('hunavsim')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'world_file': launch.substitutions.LaunchConfiguration('world_file')
            }.items()
        )
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()