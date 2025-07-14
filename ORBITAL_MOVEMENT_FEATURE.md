# Orbital Movement for Shelf Detection

## Problem Statement
The robot was getting stuck too close to shelves and could only detect objects in the bottom row due to limited field of view. This prevented complete shelf detection and proper inventory scanning.

## Solution: Orbital Movement
Implemented orbital movement behavior that makes the robot revolve around detected shelves to ensure all objects are properly detected.

## Key Features

### 1. Orbital Movement Parameters
- **Radius**: 1.5 meters from shelf center
- **Speed**: 0.15 m/s linear speed, 0.3 rad/s angular speed
- **Timeout**: 45 seconds maximum orbital time
- **Target**: 6 objects expected per shelf

### 2. Detection Workflow
1. **Object Detection**: Robot starts detecting objects through camera
2. **Shelf Focus**: When objects stabilize, robot enters shelf focus mode
3. **Orbital Movement**: Robot starts revolving around the shelf position
4. **Multi-angle Detection**: Camera captures objects from different viewing angles
5. **Completion Check**: Continues until all 6 objects are detected or timeout
6. **QR Scanning**: After object detection complete, enables QR code scanning
7. **Association**: Links detected objects with QR code for shelf completion

### 3. Movement States
- **Normal Exploration**: Full speed frontier-based exploration
- **Orbital Detection**: Controlled circular movement around shelf
- **QR Focus**: Stationary or minimal movement for QR code reading

### 4. Intelligent Control
- **Proportional Speed**: Movement speed adjusts based on distance to target
- **Face the Shelf**: Robot always faces toward the shelf center during orbit
- **Automatic Completion**: Stops orbital movement when target object count reached
- **Timeout Safety**: Prevents infinite orbital movement with configurable timeout

## Technical Implementation

### New Class Variables
```python
self.orbital_mode = False
self.orbital_center = None  # (x, y) coordinates
self.orbital_radius = 1.5   # meters
self.orbital_angle = 0.0    # current position on orbit
self.orbital_speed = 0.15   # linear speed
self.orbital_angular_speed = 0.3  # angular speed
self.expected_objects_count = 6   # target objects per shelf
```

### Key Methods
- `_start_orbital_movement()`: Initializes orbital movement around current position
- `_execute_orbital_movement()`: Calculates and executes orbital motion commands
- `_orbital_movement_step()`: Timer callback for continuous orbital movement
- `_complete_orbital_detection()`: Transitions from orbital to QR scanning phase
- `_stop_orbital_movement()`: Cleanly stops orbital movement and timers

### Movement Calculation
The robot calculates its desired position on a circle around the shelf:
```python
desired_x = center_x + radius * cos(angle)
desired_y = center_y + radius * sin(angle)
```

Then faces toward the shelf center for optimal object detection:
```python
face_angle = atan2(center_y - robot_y, center_x - robot_x)
```

## Benefits

1. **Complete Detection**: Ensures all shelf objects are visible from multiple angles
2. **Robust Scanning**: Prevents missed objects due to viewing angle limitations
3. **Automated Process**: No manual intervention required for complete shelf scanning
4. **Timeout Protection**: Prevents infinite loops with configurable time limits
5. **Smooth Integration**: Works seamlessly with existing exploration and QR detection
6. **Visual Feedback**: Status messages show orbital progress and object count

## Usage
The orbital movement is automatically triggered when:
1. Objects are detected and stabilized (3 consecutive detections)
2. Robot enters shelf focus mode
3. Less than 6 objects have been detected

The orbital movement automatically stops when:
1. Target object count (6) is reached
2. Maximum orbital time (45s) is exceeded
3. Manual reset or navigation to next area

## Status Monitoring
The system provides real-time status updates:
- `ORBITAL DETECTION` mode indicator
- Object count progress: `3/6 objects`
- Orbital angle and target position logging
- Timeout warnings and completion notifications

This feature significantly improves the warehouse inventory system's accuracy and reliability by ensuring comprehensive shelf scanning from optimal viewing angles.