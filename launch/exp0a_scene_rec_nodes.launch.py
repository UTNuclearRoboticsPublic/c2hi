import os

from ament_index_python import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    clip_params = os.path.join(
        get_package_share_directory('mm_scene_rec'),
        'config',
        'philbart_clip_params.yaml'
    )
    audio_scene_rec_params = os.path.join(
        get_package_share_directory('mm_scene_rec'),
        'config',
        'philbart_audio_scene_rec_params.yaml'
    )

    # Scene recognition nodes
    clip_rec_node = Node(package = "mm_scene_rec", 
                    executable = "clip_scene_rec_server.py",
                    name = "clip_scene_rec_server",
                    remappings=[('/clip_scene_image','/oak/rgb/image_raw')],
                    parameters=[clip_params]
    )
    ld.add_action(clip_rec_node)
    
    audio_rec_node = Node(
        package='mm_scene_rec',
        executable='audio_scene_rec_server.py',
        name='audio_scene_rec',
        output='screen',
        parameters=[audio_scene_rec_params]
    )
    ld.add_action(audio_rec_node)

    # Foxglove bridge for optional visualization
    viz_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('c2hi'),
                'launch/foxglove_bridge_launch.xml'))
    )
    ld.add_action(viz_node)

    return ld