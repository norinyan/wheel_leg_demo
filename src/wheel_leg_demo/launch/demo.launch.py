"""Load the stage 3.0 scene and robot; retain the stage 2 animation plugin."""
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('wheel_leg_demo'))
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    config = yaml.safe_load((share / 'config/demo.yaml').read_text())
    x, y, z, yaw = config['scene']['start']
    robot = xacro.process_file(str(share / 'urdf/robot.urdf.xacro')).toxml()
    # Assemble a temporary world using the editable Xacro and scene sources.
    # Loading the robot in the world avoids losing a factory-spawned model on reset.
    work = tempfile.TemporaryDirectory(prefix='wheel_leg_demo_')
    try:
        urdf_path = Path(work.name) / 'robot.urdf'
        urdf_path.write_text(robot)
        sdf = subprocess.run(['gz', 'sdf', '-p', str(urdf_path)],
                             check=True, capture_output=True, text=True)
        model = ET.fromstring(sdf.stdout).find('model')
        if model is None:
            raise RuntimeError('URDF conversion did not produce a Gazebo model')
        model.set('name', 'wheel_leg_robot')
        pose = model.find('pose')
        if pose is None:
            pose = ET.SubElement(model, 'pose')
        pose.text = f'{x} {y} {z} 0 0 {yaw}'
        plugin = ET.SubElement(model, 'plugin', name='wheel_leg_demo',
            filename=str(Path(get_package_prefix('wheel_leg_demo')) / 'lib/libwheel_leg_animation.so'))
        animation = config['animation']
        scene = config['scene']
        params = dict(animation)
        params.pop('platform_footholds')
        params.update({"foot_"+k: v for k, v in config["foot_local"].items() if k.startswith("max_") or k == "timeout"})
        params.update({key: config["wheel_local"][key] for key in ("max_v", "max_w", "max_a", "max_alpha")})
        params.update(exit_x=scene["waypoints"]["X"][0], exit_y=scene["waypoints"]["X"][1], start_x=x, start_y=y, start_z=z,
            stair_x=scene['stair']['start_x'], rise=scene['stair']['rise'],
            tread=scene['stair']['tread'], entry_x=scene['stair']['entry'][0],
            lower_end=scene['lower_bounds'][1], upper_start=scene['upper_bounds'][0],
            upper_end=scene['upper_bounds'][1], half_width=scene['stair']['width']/2,
            platform_foot_1=animation['platform_footholds'][0],
            platform_foot_2=animation['platform_footholds'][1])
        for key, value in params.items():
            ET.SubElement(plugin, key).text = str(value)
        world = ET.parse(share / 'worlds/two_steps.world')
        world.getroot().find('world').append(model)
        world_path = Path(work.name) / 'demo.world'
        world.write(world_path, encoding='unicode', xml_declaration=True)
    except Exception:
        work.cleanup()
        raise

    def cleanup(context):
        work.cleanup()
        return []

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo / 'launch/gzserver.launch.py')),
            launch_arguments={'world': str(world_path), 'verbose': 'true',
                              'server_required': 'true'}.items()),
        TimerAction(period=2.0, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo / 'launch/gzclient.launch.py')),
                condition=IfCondition(LaunchConfiguration('gui')))]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot, 'use_sim_time': True}], output='screen'),
    ])
