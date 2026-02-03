from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Mode
    use_sim_arg = DeclareLaunchArgument(
        'use_sim', default_value='False',
        description='Use simulation (Gazebo) message types (TwistStamped)')
        
    speed_arg = DeclareLaunchArgument(
        'speed', default_value='0.25',
        description='Standard forward speed')

    # Topics
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic', default_value='/scan',
        description='Lidar scan topic name')

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/cmd_vel',
        description='Velocity command topic name')

    # Sonar Configuration
    enable_sonar_arg = DeclareLaunchArgument(
        'enable_sonar', default_value='True',
        description='Enable sonar logic for rear safety')
        
    sonar_topic_arg = DeclareLaunchArgument(
        'sonar_topic', default_value='/sonar_pointcloud2',
        description='Sonar PointCloud2 topic name')

    # Bumper Configuration
    enable_bumper_arg = DeclareLaunchArgument(
        'enable_bumper', default_value='True',
        description='Enable physical bumper failsafe (requires rosaria)')
        
    bumper_topic_arg = DeclareLaunchArgument(
        'bumper_topic', default_value='/bumper_state',
        description='Bumper state topic name')

    return LaunchDescription([
        use_sim_arg,
        speed_arg,
        scan_topic_arg,
        cmd_vel_topic_arg,
        enable_sonar_arg,
        sonar_topic_arg,
        enable_bumper_arg,
        bumper_topic_arg,
        
        Node(
            package='patrolbot_nav',
            executable='navigator',
            name='patrolbot_nav',
            output='screen',
            parameters=[{
                'use_sim': LaunchConfiguration('use_sim'),
                'speed': LaunchConfiguration('speed'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'enable_sonar': LaunchConfiguration('enable_sonar'),
                'sonar_topic': LaunchConfiguration('sonar_topic'),
                'enable_bumper': LaunchConfiguration('enable_bumper'),
                'bumper_topic': LaunchConfiguration('bumper_topic'),
            }]
        )
    ])