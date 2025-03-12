#!/usr/bin/env python3

"""
This program is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.

created by Thanacha Choopojcharoen at CoXsys Robotics (2022)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro    
    
# for open robot_state_publisher
def generate_launch_description():
    
    pkg = get_package_share_directory('cart_description')

    path_description = os.path.join(pkg,'urdf','cart_description.urdf')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    # robot_desc_xml = xacro.process_file(path_description,mappings={'robot_name': 'cart_description'}).toxml()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='cart_robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc_xml}],
        remappings=[
            ("/robot_description", "/cart/robot_description"),
            ("/joint_states", "/cart/joint_states"),
        ],
    )

    launch_description = LaunchDescription()
    
    launch_description.add_action(robot_state_publisher)
    
    return launch_description