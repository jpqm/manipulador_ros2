from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Se true, abre o RViz2 automaticamente.',
    )
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Índice da câmera V4L2 (/dev/video<N>).',
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Porta serial do controlador GRBL.',
    )

    xacro_file = PathJoinSubstitution([
        FindPackageShare('arctos_urdf_description'),
        'urdf',
        'arctos_urdf.xacro',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('manipulador_control'),
        'config',
        'interface.rviz',
    ])

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_gui')),
    )

    return LaunchDescription([
        use_gui_arg,
        camera_index_arg,
        serial_port_arg,

        # 1. Robô real/atual (/joint_states)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_real',
            parameters=[{'robot_description': robot_description}],
        ),

        # 2. Robô de prévia (/preview/joint_states com prefixo TF 'preview/')
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='preview',
            name='robot_state_publisher_preview',
            parameters=[{
                'robot_description': robot_description,
                'frame_prefix': 'preview/',
            }],
        ),

        # 3. Ancora preview/base_link na mesma origem de base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='preview_base_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'preview/base_link',
            ],
        ),

        # 4. Nó principal de controle (GRBL/Mock + bezier.py + ik_craig.py + Gizmo 6-DOF + Câmera/YOLO)
        Node(
            package='manipulador_control',
            executable='ros2_node',
            name='manipulador_node',
            output='screen',
            parameters=[{
                'camera_index': ParameterValue(LaunchConfiguration('camera_index'), value_type=int),
                'serial_port': LaunchConfiguration('serial_port'),
            }],
        ),

        # 5. RViz2 único (com Painel Lateral embutido + Gizmo 6-DOF + Robô Atual/Prévia + Câmera)
        rviz_node,

        # 6. Encerra todos os nós automaticamente quando a janela do RViz2 for fechada no X
        RegisterEventHandler(
            OnProcessExit(
                target_action=rviz_node,
                on_exit=[Shutdown(reason='Janela do RViz2 fechada pelo usuário')],
            )
        ),
    ])
