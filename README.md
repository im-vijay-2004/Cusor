# Warehouse Exploration & Object Recognition System

A comprehensive ROS2 system for autonomous warehouse exploration with AI-powered object detection and QR code recognition capabilities.

## System Overview

This system consists of two integrated ROS2 nodes:

1. **Object Recognizer Node** (`object_recognizer.py`): YOLO-based object detection with shelf-aware filtering
2. **Warehouse Explorer Node** (`warehouse_explore.py`): Autonomous navigation with shelf detection workflow

Together, they provide a complete solution for warehouse inventory and exploration tasks.

## Features

### Core Functionality
- **🤖 AI Object Detection**: YOLO-based real-time object recognition optimized for warehouse items
- **🚗 Autonomous Navigation**: Uses Nav2 for path planning and obstacle avoidance
- **🗺️ Frontier-Based Exploration**: Automatically identifies and navigates to unexplored areas
- **📱 QR Code Detection**: Real-time QR code detection using OpenCV (no external dependencies)
- **🔗 Shelf Detection Workflow**: Strict sequence ensuring QR → Objects association
- **📊 GUI Progress Tracking**: Real-time visualization of detected objects and QR codes
- **⚡ Performance Optimization**: TensorFlow Lite inference with NPU support

### Advanced Capabilities
- **Multi-Mode Exploration**: Switches between frontier-based and random exploration
- **Intelligent Goal Management**: Handles goal cancellation and recovery behaviors
- **State Monitoring**: Tracks robot pose, map data, and detection timestamps
- **Robust Error Handling**: Graceful failure recovery and status reporting

## Architecture

### Object Recognizer Node
**Subscriptions:**
- `/camera/image_raw/compressed` - Camera feed for object detection

**Publishers:**
- `/shelf_objects` - Detected objects with counts
- `/debug_images/object_recog` - Annotated object detection images

### Warehouse Explorer Node
**Subscriptions:**
- `/pose` - Robot pose updates
- `/global_costmap/costmap` - Global map for exploration planning
- `/map` - Simple map data
- `/cerebri/out/status` - Robot status and arming state
- `/behavior_tree_log` - Navigation behavior monitoring
- `/shelf_objects` - Detected objects from Object Recognizer
- `/camera/image_raw/compressed` - Camera feed for QR detection

**Publishers:**
- `/cerebri/in/joy` - Manual control commands
- `/debug_images/qr_code` - Annotated QR detection images
- `/shelf_data` - Processed shelf data with QR codes

**Action Clients:**
- `/navigate_to_pose` - Nav2 navigation goals

### Data Flow
```
Camera → Object Recognizer → /shelf_objects → Warehouse Explorer
Camera → Warehouse Explorer (QR Detection)
Warehouse Explorer → /shelf_data (Final Output)
```

## Usage

### Complete System Launch (Recommended)
```bash
ros2 launch <package_name> warehouse_system_launch.py shelf_count:=5 confidence_threshold:=0.3
```

### Individual Node Launch

#### Object Recognizer Only
```bash
ros2 run <package_name> object_recognizer.py --ros-args -p confidence_threshold:=0.3 -p filter_shelf_objects:=true
```

#### Warehouse Explorer Only
```bash
ros2 run <package_name> warehouse_explore.py --ros-args -p shelf_count:=5 -p initial_angle:=1.57
```

### Launch Parameters

#### System Parameters
- `shelf_count` (int, default=5): Number of shelves expected in warehouse
- `confidence_threshold` (float, default=0.3): Object detection confidence threshold
- `debug_enabled` (bool, default=true): Enable debug image publishing
- `performance_monitoring` (bool, default=true): Enable performance reporting

#### Object Recognition Parameters
- `iou_threshold` (float, default=0.45): Intersection over Union threshold for NMS
- `max_detections` (int, default=50): Maximum number of objects to detect
- `filter_shelf_objects` (bool, default=true): Filter to shelf-relevant objects only
- `min_object_size` (float, default=0.01): Minimum object size as fraction of image

#### Warehouse Explorer Parameters  
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

## Object Detection Capabilities

### YOLO Model Integration
- **Model**: YOLOv5n-int8 TensorFlow Lite for efficient inference
- **Classes**: 80 COCO object classes with warehouse-relevant filtering
- **Performance**: Optimized for real-time processing on edge devices
- **Hardware Support**: CPU, GPU, and NPU (NavQPlus) acceleration

### Shelf-Relevant Object Categories
The system filters detected objects to focus on items typically found on warehouse shelves:
- **Food Items**: banana, apple, orange, pizza, donut, cake, etc.
- **Kitchen Items**: bottle, cup, bowl, fork, knife, spoon, etc.
- **Electronics**: laptop, mouse, remote, keyboard, cell phone, etc.
- **Household**: book, clock, vase, scissors, teddy bear, etc.
- **Vehicles**: car, motorcycle, airplane, bus, train, truck, etc.

### Detection Features
- **Confidence-Based Filtering**: Adjustable threshold for detection reliability
- **Size Filtering**: Removes objects too small to be reliably identified
- **Non-Maximum Suppression**: Eliminates duplicate detections
- **Visual Debugging**: Color-coded bounding boxes based on confidence levels
- **Performance Monitoring**: Real-time FPS and inference time reporting

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

### Python Dependencies
Install dependencies via pip:
```bash
pip install -r requirements.txt
```

### ROS2 Dependencies
```bash
rosdep install --from-paths . --ignore-src -r -y
```

### Required Model Files
The system requires these model files to be available:

1. **YOLO Model**: `yolov5n-int8.tflite`
   - Place in: `share/ament_index/resource_index/`
   - Alternative locations: `~/yolov5n-int8.tflite` or working directory

2. **COCO Labels**: `coco.yaml`
   - Place in: `share/ament_index/resource_index/`
   - Alternative locations: `~/coco.yaml` or working directory
   - Contains COCO class names for object detection

### Hardware Requirements
- **Minimum**: CPU-only inference (slower but functional)
- **Recommended**: GPU-accelerated inference for real-time performance
- **Optimal**: NavQPlus with NPU support for maximum efficiency

## Code Structure

### Object Recognizer Node (`object_recognizer.py`)
- **ObjectRecognizer**: Main class handling YOLO inference and object detection
- **Model Management**: TensorFlow Lite model loading and initialization
- **Image Processing**: Camera image preprocessing and postprocessing
- **Object Filtering**: Shelf-relevant object filtering and size validation
- **Performance Monitoring**: FPS tracking and inference time reporting

### Warehouse Explorer Node (`warehouse_explore.py`)
- **WarehouseExplore**: Main class for navigation and exploration
- **Exploration Logic**: Handles frontier detection and goal planning
- **Shelf Detection**: Manages QR code detection and shelf confirmation
- **Object Integration**: Processes detected objects with strict workflow
- **State Management**: Tracks exploration progress and robot status

### Helper Components
- **WindowProgressTable**: GUI for progress visualization
- **Utility Functions**: Map coordinate conversion, goal creation, etc.
- **Detection Workflow**: Shelf-object association logic

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