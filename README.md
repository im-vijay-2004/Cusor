# Warehouse Exploration ROS2 Node

A comprehensive ROS2 node for autonomous warehouse exploration with object detection and QR code recognition capabilities.

## Features

### Core Functionality
- **Autonomous Navigation**: Uses Nav2 for path planning and obstacle avoidance
- **Frontier-Based Exploration**: Automatically identifies and navigates to unexplored areas
- **QR Code Detection**: Real-time QR code detection using OpenCV (no external dependencies)
- **Object Detection Integration**: Processes shelf object data and publishes results
- **GUI Progress Tracking**: Real-time visualization of detected objects and QR codes

### Advanced Capabilities
- **Multi-Mode Exploration**: Switches between frontier-based and random exploration
- **Intelligent Goal Management**: Handles goal cancellation and recovery behaviors
- **State Monitoring**: Tracks robot pose, map data, and detection timestamps
- **Robust Error Handling**: Graceful failure recovery and status reporting

## Architecture

### Subscriptions
- `/pose` - Robot pose updates
- `/global_costmap/costmap` - Global map for exploration planning
- `/map` - Simple map data
- `/cerebri/out/status` - Robot status and arming state
- `/behavior_tree_log` - Navigation behavior monitoring
- `/shelf_objects` - Detected objects from shelf analysis
- `/camera/image_raw/compressed` - Camera feed for QR detection

### Publishers
- `/cerebri/in/joy` - Manual control commands
- `/debug_images/qr_code` - Annotated QR detection images
- `/shelf_data` - Processed shelf data with QR codes

### Action Clients
- `/navigate_to_pose` - Nav2 navigation goals

## Usage

### Basic Launch
```bash
ros2 run <package_name> warehouse_explore.py
```

### With Parameters
```bash
ros2 run <package_name> warehouse_explore.py --ros-args -p shelf_count:=5 -p initial_angle:=1.57
```

### Parameters
- `shelf_count` (int, default=1): Number of shelves expected in warehouse
- `initial_angle` (float, default=0.0): Initial orientation angle in radians

## Exploration Modes

### 1. Frontier Mode (Default)
- Identifies unexplored map boundaries
- Selects closest accessible frontiers
- Automatically expands search radius when needed

### 2. Random Mode
- Activated when frontiers are exhausted
- Selects random free space locations
- Avoids recently visited positions

### 3. Complete Mode
- Reached after maximum exploration attempts
- Stops autonomous exploration

## QR Code & Object Detection

### Shelf Detection Workflow
The node follows a strict detection sequence to ensure data integrity:

1. **Shelf Detection First**: QR code detection indicates shelf presence
2. **Object Processing**: Objects are only processed after shelf confirmation
3. **Data Association**: Objects are linked with the shelf's QR code
4. **Publication**: Complete shelf data is published only when both are confirmed

### QR Code Processing
- Uses OpenCV's built-in QR detector (no libzbar dependency)
- QR detection triggers shelf detection state
- Updates detection timestamps
- Resets QR data after extended non-detection periods

### Object Data Publishing
- **Conditional Processing**: Objects are only processed if a shelf is detected first
- **Temporal Buffering**: Objects detected before shelf confirmation are temporarily stored
- **Timeout Handling**: Orphaned objects or shelves are discarded after timeout periods
- **Complete Data**: Publishes shelf data only when both QR code and objects are confirmed
- Updates GUI table in real-time

### Detection States
- **No Shelf**: Waiting for QR code detection
- **Shelf Detected**: QR code found, waiting for or processing objects
- **Processing Complete**: Shelf data published, ready for next shelf

## GUI Features

The optional GUI provides:
- **Object Display**: Shows detected objects per shelf
- **QR Code Display**: Current QR code for each shelf
- **Color Coding**: Visual status indicators
- **Real-time Updates**: Live data from exploration

## Error Handling & Recovery

### Navigation Recovery
- Monitors recovery attempts during navigation
- Cancels stuck goals automatically
- Switches exploration modes when needed

### Detection Timeout
- Resets QR codes after 30 seconds of non-detection
- Tracks last detection timestamps
- Provides status logging for debugging

## Dependencies

Install dependencies via pip:
```bash
pip install -r requirements.txt
```

For ROS2 dependencies:
```bash
rosdep install --from-paths . --ignore-src -r -y
```

## Code Structure

### Main Class: `WarehouseExplore`
- **Initialization**: Sets up publishers, subscribers, and parameters
- **Exploration Logic**: Handles frontier detection and goal planning
- **Detection Processing**: Manages QR codes and object data
- **State Management**: Tracks exploration progress and robot status

### Helper Classes
- **WindowProgressTable**: GUI for progress visualization
- **Various Utilities**: Map coordinate conversion, goal creation, etc.

## Status Monitoring

The node provides periodic status updates including:
- Current exploration mode
- Robot arming status
- Active goal status
- Time since last QR/object detections
- Exploration attempt counters

## Customization

### Exploration Parameters
- `max_step_dist_world_meters`: Maximum goal distance
- `min_step_dist_world_meters`: Minimum goal distance
- `recovery_threshold`: Max recovery attempts before goal cancellation
- `max_exploration_attempts`: Total exploration limit

### Detection Settings
- `shelf_detection_timeout`: Time to wait for objects after shelf detection (default: 5.0s)
- `object_detection_timeout`: Time to wait for shelf confirmation after objects (default: 10.0s) 
- QR timeout duration (30s for automatic reset)
- Object detection processing logic
- GUI update behavior

## License

Licensed under the Apache License, Version 2.0. See the file header for full license text.

## Contributing

This code serves as a complete warehouse exploration solution with room for customization based on specific warehouse layouts and requirements.