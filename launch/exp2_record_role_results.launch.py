import os

from ament_index_python import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    ld = LaunchDescription()

    # Configuration
    params = os.path.join(
        get_package_share_directory('c2hi'),
        'config',
        'exp2_role_cfgs',
        'default_role_exp_params.yaml'
    )

    tracker_params = os.path.join(
        get_package_share_directory('c2hi'),
        'config',
        'experiment_tracker.yaml'
    )

    # Static TF nodes
    tf_node = Node(package = "tf2_ros", 
                    executable = "static_transform_publisher",
                    arguments = [".15", "0", ".55", "0", "-0.261799", "0", "philbart/base_link", "oak-d-base-frame"]
    )
    ld.add_action(tf_node)

    # Detector preprocessing nodes
    oakd_preproc_node = Node(
        package='marmot',
        executable='depthai_img_preproc',
        name='depthai_img_preproc_node',
        remappings=[('/converted_detections','/converted_vision_detections'),
                    ('/depthai_detections','/oak/nn/spatial_detections'),
                    ('/depthai_img','/oak/rgb/image_raw')],
        parameters=[params])    
    ld.add_action(oakd_preproc_node)

    # Object recognition
    clip_vis_rec_server = Node(package = "c2hi", 
                    executable = "clip_vis_rec_server.py",
                    name = "clip_vis_rec_server",
                    parameters=[params]
    )
    ld.add_action(clip_vis_rec_server)

    ### MULTIMODAL PROCESSING
    
    # Tracker node
    trk_node = Node(
        package='marmot',
        executable='tbd_node.py',
        name='tbd_tracker_node',
        output='screen',
        parameters=[tracker_params]
    )
    ld.add_action(trk_node)

    # Semantic fusion node
    semantic_fusion_node = Node(
        package='c2hi',
        executable='semantic_fusion_node.py',
        name='semantic_fusion_node',
        output='screen',
        remappings=[('tracks','tbd_tracker_node/tracks')],
        parameters=[params]
    )
    ld.add_action(semantic_fusion_node)

    ### EXPERIMENT NODES
    experiment_mgr_node = Node(
        package='c2hi',
        executable='role_exp_mgr.py',
        name='role_exp_mgr',
        output='screen',
        parameters=[params]
    )
    ld.add_action(experiment_mgr_node)

    rec_cmd_results_node = Node(package = "c2hi", 
                    executable = "record_hierarchical_cmd_results_node.py",
                    name = "record_hierarchical_cmd_results_node",
                    output = 'screen',
                    parameters=[params]
    )
    ld.add_action(rec_cmd_results_node)

    ### MISCELLANY
    # Foxglove bridge for visualization
    viz_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('marmot'),
                'launch/foxglove_bridge_launch.xml'))
    )
    ld.add_action(viz_node)

    return ld