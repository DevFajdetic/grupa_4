import os

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node



def generate_launch_description():
    pkg_share = get_package_share_directory('turtlebot3_bringup')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot.launch.py')
        ),
    )
    
    cam2image = Node(
        package='image_tools', 
        executable='cam2image', 
        parameters=[{'frequency' : 5.0,
        'history'   : "keep_last"
        }]
    )

    controller = Node(
        package='grupa_4',
        executable='controller'
    )

    ld = LaunchDescription()
    ld.add_action(robot_launch)
    ld.add_action(cam2image)
    ld.add_action(controller)

    return ld