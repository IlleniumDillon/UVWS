import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    mapfile = os.path.join(
        get_package_share_directory('uvplanner'),'map/mapdilated.yaml'
    )
    loadmap = Node(
        package='nav2_map_server',
        executable='map_server',
        arguments=[
            '--ros-args',
            '--param',
            'yaml_filename:='+mapfile,
        ]
    )
    configmap = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[[
                    'ros2 lifecycle set /map_server configure'
                ]],
                shell=True
            )
        ]
    )
    activemap = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[[
                    'ros2 lifecycle set /map_server activate'
                ]],
                shell=True
            )
        ]
    )
    return LaunchDescription([
        loadmap,
        configmap,
        activemap
    ])