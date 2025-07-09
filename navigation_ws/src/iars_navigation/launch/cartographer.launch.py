# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
#################################### 节点参数配置 ########################################################
    # 导航功能包的路径
    navigation2_dir = get_package_share_directory('originbot_navigation')
    # 是否使用仿真时间，这里使用Gazebo，所以配置为true
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 构建地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 发布地图数据的周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # 参数配置文件在功能包中的文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(navigation2_dir, 'config') )
    # 参数配置文件的名称
    configuration_basename = LaunchConfiguration('configuration_basename', default='lds_2d.lua')
    # rviz可视化显示的配置文件路径
    rviz_config_dir = os.path.join(navigation2_dir, 'rviz')+"/slam.rviz"
 
 ################ 启动节点：cartographer_node、cartographer_occupancy_grid_node、rviz2 ###################
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
        
    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(rviz_node)

    return ld
