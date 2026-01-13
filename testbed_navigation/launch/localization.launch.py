import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    amcl_config = os.path.join(
        get_package_share_directory('testbed_navigation'),
        'config',
        'amcl_params.yaml'
    )
    
    rviz_config_path = os.path.join(
      get_package_share_directory('testbed_navigation'),
      'rviz',
      'nav2_default_view.rviz'
    )

    return LaunchDescription([
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[amcl_config],
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{
                'autostart': True,
                'node_names': ['amcl']
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='Rviz',
            arguments=['-d', rviz_config_path],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
