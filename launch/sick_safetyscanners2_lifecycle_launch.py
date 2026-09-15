# Parametreler config/sick_safetyscanners2.param.yaml dosyasindan okunur.
# Baska bir dosya kullanmak icin:
#   ros2 launch sick_safetyscanners2 sick_safetyscanners2_lifecycle_launch.py \
#       params_file:=/yol/benim_scanner.param.yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("sick_safetyscanners2"),
            "config",
            "sick_safetyscanners2.param.yaml",
        ]
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Surucu parametrelerini iceren YAML dosyasi.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Dugumun calisacagi namespace.",
        ),
    ]

    driver_node = Node(
        package="sick_safetyscanners2",
        executable="sick_safetyscanners2_lifecycle_node",
        name="sick_safetyscanners2_lifecycle_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription(declared_arguments + [driver_node])
