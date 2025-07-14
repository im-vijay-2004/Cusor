#!/usr/bin/env python3
"""
Launch file for the complete warehouse exploration system.
Starts both the object recognition and warehouse exploration nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate launch description for warehouse exploration system."""
    
    # Declare launch arguments
    shelf_count_arg = DeclareLaunchArgument(
        'shelf_count',
        default_value='5',
        description='Number of shelves expected in the warehouse'
    )
    
    initial_angle_arg = DeclareLaunchArgument(
        'initial_angle',
        default_value='0.0',
        description='Initial orientation angle in radians'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.3',
        description='Confidence threshold for object detection'
    )
    
    debug_enabled_arg = DeclareLaunchArgument(
        'debug_enabled',
        default_value='true',
        description='Enable debug image publishing'
    )
    
    filter_shelf_objects_arg = DeclareLaunchArgument(
        'filter_shelf_objects',
        default_value='true',
        description='Filter objects to only shelf-relevant items'
    )
    
    performance_monitoring_arg = DeclareLaunchArgument(
        'performance_monitoring',
        default_value='true',
        description='Enable performance monitoring and reporting'
    )

    # Object Recognition Node
    object_recognizer_node = Node(
        package='b3rb_ros_aim_india',  # Replace with your actual package name
        executable='object_recognizer.py',
        name='object_recognizer',
        output='screen',
        parameters=[
            {'confidence_threshold': LaunchConfiguration('confidence_threshold')},
            {'debug_enabled': LaunchConfiguration('debug_enabled')},
            {'filter_shelf_objects': LaunchConfiguration('filter_shelf_objects')},
            {'performance_monitoring': LaunchConfiguration('performance_monitoring')},
            {'iou_threshold': 0.45},
            {'max_detections': 50},
            {'min_object_size': 0.01}
        ],
        remappings=[
            # Add any topic remappings if needed
        ]
    )

    # Warehouse Exploration Node
    warehouse_explorer_node = Node(
        package='b3rb_ros_aim_india',  # Replace with your actual package name
        executable='warehouse_explore.py',
        name='warehouse_explore',
        output='screen',
        parameters=[
            {'shelf_count': LaunchConfiguration('shelf_count')},
            {'initial_angle': LaunchConfiguration('initial_angle')}
        ],
        remappings=[
            # Add any topic remappings if needed
        ]
    )

    # Log startup message
    startup_message = LogInfo(
        msg=[
            '🚀 Starting Warehouse Exploration System\n',
            '   📦 Object Recognition: Detecting shelf objects\n',
            '   🗺️  Warehouse Explorer: Autonomous navigation and exploration\n',
            '   🔗 Integration: QR code + object detection workflow\n',
            '   📊 Shelves Expected: ', LaunchConfiguration('shelf_count'), '\n',
            '   🎯 Confidence Threshold: ', LaunchConfiguration('confidence_threshold')
        ]
    )

    return LaunchDescription([
        # Arguments
        shelf_count_arg,
        initial_angle_arg,
        confidence_threshold_arg,
        debug_enabled_arg,
        filter_shelf_objects_arg,
        performance_monitoring_arg,
        
        # Startup message
        startup_message,
        
        # Nodes
        object_recognizer_node,
        warehouse_explorer_node
    ])


if __name__ == '__main__':
    generate_launch_description()