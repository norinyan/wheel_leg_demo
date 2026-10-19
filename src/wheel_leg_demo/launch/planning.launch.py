"""Global planner with optional wheel-local TEB execution; run alongside demo.launch.py."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    config = yaml.safe_load((Path(get_package_share_directory('wheel_leg_demo')) / 'config/demo.yaml').read_text())
    scene, robot = config['scene'], config['robot']
    points, stair = scene['waypoints'], scene['stair']
    params = dict(config['global_planning'])
    params.update(lower_bounds=scene['lower_bounds'], upper_bounds=scene['upper_bounds'],
                  start=points['A'], goal=points['B'], entry=points['E'], exit=points['X'],
                  stair_x=stair['start_x'], rise=stair['rise'], tread=stair['tread'],
                  stair_width=stair['width'], footprint_length=robot['wheel_mode_length'],
                  footprint_width=robot['wheel_mode_width'],
                  obstacles=[float(v) for o in scene['obstacles'] for v in o['center'] + o['size']],
                  use_sim_time=True)
    params.update({"wheel."+key: value for key, value in config["wheel_local"].items()})
    params.update({"foot."+key: value for key, value in config["foot_local"].items()})
    params.update({"task."+key: value for key, value in config["task"].items()})
    params["task_enabled"] = ParameterValue(LaunchConfiguration("task"), value_type=bool)
    params["foot_local_enabled"] = ParameterValue(LaunchConfiguration("foot_local"), value_type=bool)
    # Resolve recording paths at launch time, independently of cwd and checkout.
    for prefix, section in (("wheel", "wheel_local"), ("foot", "foot_local"), ("task", "task")):
        params[prefix + ".output_dir"] = ParameterValue(
            PathJoinSubstitution([LaunchConfiguration("results_dir"), config[section]["output_dir"]]),
            value_type=str)
    params["output_dir"] = ParameterValue(LaunchConfiguration("global_output_dir"), value_type=str)
    params["wheel_local_enabled"] = ParameterValue(LaunchConfiguration("wheel_local"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument(
            "results_dir", default_value=str(Path.home() / ".local/state/wheel_leg_demo"),
            description="Runtime results root; independent of source/install location"),
        DeclareLaunchArgument("task", default_value="false"),
        DeclareLaunchArgument("foot_local", default_value="false"),
        DeclareLaunchArgument("wheel_local", default_value="false"),
        DeclareLaunchArgument(
            "global_output_dir", default_value=PathJoinSubstitution([
                LaunchConfiguration("results_dir"), config["global_planning"]["output_dir"]])),
        Node(package='wheel_leg_demo', executable='global_planner',
             parameters=[params], output='screen'),
    ])
