import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
     
      matcher_odom= Node(
        package='laser_scan_matcher',
        executable='laser_scan_matcher_node',
        name="laser_scan_matcher",
        namespace='falkor_odom',  
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        parameters=[LaunchConfiguration('matcher_config')],
        
      )

      matcher_parameters_file = os.path.join(
          get_package_share_directory('laser_scan_matcher'),
          'params', 'laser_scan_matcher.yaml'
      )
             
      ld = LaunchDescription([
            DeclareLaunchArgument('matcher_config', default_value=matcher_parameters_file),
           ])

      ld.add_action(matcher_odom)
      return ld