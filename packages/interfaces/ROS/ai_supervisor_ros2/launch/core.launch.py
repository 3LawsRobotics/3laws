import pathlib
import typing

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

CFG_PATH = pathlib.Path(get_package_share_directory('ai_supervisor_ros2')) / 'config'


def load_cfg(filename: str) -> typing.Optional[typing.Dict[str, typing.Any]]:
    ret = {}
    # General config
    if (CFG_PATH / filename).exists():
        print(CFG_PATH / filename)
        ret = yaml.load((CFG_PATH / filename).open().read(), Loader=yaml.FullLoader)

    return ret if len(ret) > 0 else None


def generate_launch_description():

    launchdesc = LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('which', default_value='unicycle'),
        ]
    )

    cfg = load_cfg('unicycle.yaml')

    # ---------------------------------------------------------------------------- #
    #                                  MAIN NODES                                  #
    # ---------------------------------------------------------------------------- #

    launchdesc.add_action(
        Node(
            package='ai_supervisor_ros2',
            executable='kernel_generator_unicycle',
            output='screen',
            emulate_tty=True,
            parameters=[
                cfg['kernel_generator'],
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        )
    )

    launchdesc.add_action(
        Node(
            package='ai_supervisor_ros2',
            executable='input_filter',
            output='screen',
            emulate_tty=True,
            parameters=[cfg['input_filter'], {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )
    )

    return launchdesc
