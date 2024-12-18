import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, SetParameter
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, EqualsSubstitution, NotEqualsSubstitution


def generate_launch_description():
    package_name = 'frtn70'

    # load crazyflies
    crazyflies_yaml = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'crazyflies.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    # server params
    server_yaml = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'server.yaml')

    with open(server_yaml, 'r') as ymlfile:
        server_yaml_content = yaml.safe_load(ymlfile)

    server_yaml_content["/crazyflie_server"]["ros__parameters"]['robots'] = crazyflies['robots']
    server_yaml_content["/crazyflie_server"]["ros__parameters"]['robot_types'] = crazyflies['robot_types']
    server_yaml_content["/crazyflie_server"]["ros__parameters"]['all'] = crazyflies['all']

    # robot description
    urdf = os.path.join(
        get_package_share_directory('crazyflie'),
        'urdf',
        'crazyflie_description.urdf')
    with open(urdf, 'r') as f:

        robot_desc = f.read()
    server_yaml_content["/crazyflie_server"]["ros__parameters"]["robot_description"] = robot_desc

    # construct motion_capture_configuration
    motion_capture_yaml = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'motion_capture.yaml')

    with open(motion_capture_yaml, 'r') as ymlfile:
        motion_capture_content = yaml.safe_load(ymlfile)

    motion_capture_content["/motion_capture_tracking"]["ros__parameters"]["rigid_bodies"] = dict()
    for key, value in crazyflies["robots"].items():
        type = crazyflies["robot_types"][value["type"]]
        if value["enabled"] and type["motion_capture"]["enabled"]:
            motion_capture_content["/motion_capture_tracking"]["ros__parameters"]["rigid_bodies"][key] =  {
                    "initial_position": value["initial_position"],
                    "marker": type["motion_capture"]["marker"],
                    "dynamics": type["motion_capture"]["dynamics"],
                }

    # copy relevent settings to server params
    server_yaml_content["/crazyflie_server"]["ros__parameters"]["poses_qos_deadline"] = motion_capture_content[
        "/motion_capture_tracking"]["ros__parameters"]["topics"]["poses"]["qos"]["deadline"]

    # Save server and mocap in temp file such that nodes can read it out later
    with open('tmp_server.yaml', 'w') as outfile:
        yaml.dump(server_yaml_content, outfile, default_flow_style=False, sort_keys=False)

    with open('tmp_motion_capture.yaml', 'w') as outfile:
        yaml.dump(motion_capture_content, outfile, default_flow_style=False, sort_keys=False)

    ld = [
        DeclareLaunchArgument('debug', default_value='False'),
        DeclareLaunchArgument('rviz', default_value='True'),
        DeclareLaunchArgument('gui', default_value='False'),
        DeclareLaunchArgument('graph', default_value='False'),
        DeclareLaunchArgument('server_yaml_file', default_value=''),
        DeclareLaunchArgument('teleop_yaml_file', default_value=''),
        DeclareLaunchArgument('mocap_yaml_file', default_value=''),
        Node(
            package='motion_capture_tracking',
            executable='motion_capture_tracking_node',
            condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration('backend'),'sim')),
            name='motion_capture_tracking',
            output='screen',
            parameters= [PythonExpression(["'tmp_motion_capture.yaml' if '", LaunchConfiguration('mocap_yaml_file'), "' == '' else '", LaunchConfiguration('mocap_yaml_file'), "'"])],
        ),
        Node(
            package='crazyflie',
            executable='crazyflie_server.py',
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('backend'),'cflib')),
            name='crazyflie_server',
            output='screen',
            parameters= [PythonExpression(["'tmp_server.yaml' if '", LaunchConfiguration('server_yaml_file'), "' == '' else '", LaunchConfiguration('server_yaml_file'), "'"])],
        ),
        Node(
            package='crazyflie',
            executable='crazyflie_server',
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('backend'),'cpp')),
            name='crazyflie_server',
            output='screen',
            parameters= [PythonExpression(["'tmp_server.yaml' if '", LaunchConfiguration('server_yaml_file'), "' == '' else '", LaunchConfiguration('server_yaml_file'), "'"])],
            prefix=PythonExpression(['"xterm -e gdb -ex run --args" if ', LaunchConfiguration('debug'), ' else ""']),
        ),
        Node(
            package='crazyflie_sim',
            executable='crazyflie_server',
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('backend'),'sim')),
            name='crazyflie_server',
            output='screen',
            emulate_tty=True,
            parameters= [PythonExpression(["'tmp_server.yaml' if '", LaunchConfiguration('server_yaml_file'), "' == '' else '", LaunchConfiguration('server_yaml_file'), "'"])],
        ),
        Node(
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('rviz'), 'True')),
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory(package_name), 'config', 'config.rviz')],
            parameters=[{
                "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
            }]
        ),
        Node(
            name="PlotJuggler",
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('graph'), 'True')),
            package="plotjuggler",
            executable="plotjuggler",
            arguments=['-n',  'buffer_size 9999'],
            output="screen"
        ),
    ]

    use_simtime = IfCondition(EqualsSubstitution(LaunchConfiguration('backend'), 'sim'))
    backend = DeclareLaunchArgument('backend', default_value='cpp')
    ld.insert(0, backend)

    if use_simtime:
        print("Using simtime if true: ", EqualsSubstitution(LaunchConfiguration('backend'), 'sim'))
        ld.insert(1, SetParameter(name='use_sim_time', value=EqualsSubstitution(LaunchConfiguration('backend'), 'sim')))


    return LaunchDescription(ld)

