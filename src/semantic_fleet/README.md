# Semantic Fleet Package

Multi-Robot Semantic SLAM system using YOLOv8 object detection.

## Package Contents

### Nodes

#### 1. `yolo_detector`
- **Subscribes to:** `/camera/image_raw` (sensor_msgs/Image)
- **Publishes:**
  - `~/detected_objects` (semantic_fleet/DetectedObjects)
  - `~/detection_visualization` (sensor_msgs/Image)
- **Parameters:**
  - `model_path`: YOLOv8 model file (default: "yolov8n.pt")
  - `confidence_threshold`: Detection confidence threshold (default: 0.6)
  - `detection_rate`: Detection frequency in Hz (default: 5.0)
  - `camera_topic`: Camera image topic (default: "/camera/image_raw")
  - `publish_visualization`: Enable detection visualization (default: true)

#### 2. `object_localizer`
- **Subscribes to:** `/yolo_detector/detected_objects`
- **Publishes:** `~/localized_objects` (semantic_fleet/DetectedObjects)
- **Parameters:**
  - `camera_frame`: Camera frame ID (default: "camera_rgb_optical_frame")
  - `world_frame`: World/map frame ID (default: "map")
  - `assumed_object_distance`: Default object distance in meters (default: 1.5)
  - `camera_fov_h`: Horizontal field of view in radians (default: 1.085)
  - `camera_fov_v`: Vertical field of view in radians (default: 0.785)

#### 3. `semantic_mapper`
- **Subscribes to:** `/object_localizer/localized_objects`
- **Publishes:** `~/semantic_map` (semantic_fleet/SemanticMap)
- **Parameters:**
  - `matching_distance_threshold`: Distance for object matching in meters (default: 0.5)
  - `min_observations`: Minimum observations before adding to map (default: 2)
  - `update_rate`: Map publish frequency in Hz (default: 1.0)
  - `confidence_decay`: Confidence decay rate for unseen objects (default: 0.95)
  - `min_confidence`: Minimum confidence to keep object in map (default: 0.3)

### Custom Messages

- `DetectedObject.msg`: Single detected object with 2D bbox and 3D position
- `DetectedObjects.msg`: Array of detected objects
- `SemanticMap.msg`: Complete semantic map with object positions and classes

## Installation

### Prerequisites

```bash
# ROS 2 Humble packages
sudo apt install ros-humble-navigation2 \
                 ros-humble-slam-toolbox \
                 ros-humble-cv-bridge \
                 ros-humble-vision-opencv \
                 ros-humble-tf2-ros

# Python dependencies (in virtual environment)
pip install "numpy<2.0"  # Important: cv_bridge requires numpy <2.0
pip install ultralytics opencv-python lxml
```

### Build

```bash
cd ~/Desktop/fleet_ws
colcon build --packages-select semantic_fleet --symlink-install
source install/setup.bash

## Usage

### Complete Test Setup of Single Robot Demo with TurtleBot3

```bash
# Terminal 1: Gazebo + TurtleBot3
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: SLAM
source ~/Desktop/fleet_ws/setup_workspace.sh
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

# Terminal 3: Semantic Fleet
source ~/Desktop/fleet_ws/setup_workspace.sh
ros2 launch semantic_fleet single_robot_semantic.launch.py

# Terminal 4: Teleop Control
source ~/Desktop/fleet_ws/setup_workspace.sh
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard

### Running Individual Nodes

```bash
# YOLO Detector
ros2 run semantic_fleet yolo_detector

# Object Localizer
ros2 run semantic_fleet object_localizer

# Semantic Mapper
ros2 run semantic_fleet semantic_mapper
```

### Viewing Topics

```bash
# List all topics
ros2 topic list

# View detected objects
ros2 topic echo /yolo_detector/detected_objects

# View semantic map
ros2 topic echo /semantic_mapper/semantic_map

# Visualize detections with bounding boxes
ros2 run rqt_image_view rqt_image_view
# Select: /yolo_detector/detection_visualization
```

## Dependencies

- ROS 2 Humble
- Python 3.10+
- ultralytics (YOLOv8)
- opencv-python
- **numpy <2.0** (critical for cv_bridge compatibility)
- cv_bridge
- tf2_ros
- tf2_geometry_msgs


## Known Limitations

### Simulation Detection Accuracy
YOLO is trained on real-world images (COCO dataset) and may produce false positives or misclassifications in Gazebo due to:
- Domain gap between synthetic and real images
- Simplified textures and lighting in simulation
- Geometric shapes matching trained patterns

**Solution:** Works significantly better with real robot hardware or by adding photorealistic objects to Gazebo.

### Performance
- Detection rate set to 5 Hz for optimal performance
- Higher rates may cause CPU load in simulation
- Real hardware typically allows 10-15 Hz


## Week 1 Status  - ✅ COMPLETE

- [x] YOLO object detection node
- [x] Object localization with TF2
- [x] Semantic map builder
- [x] Custom message definitions
- [x] Launch files
- [x] Configuration files
- [ ] RViz visualization (coming soon)
- [ ] Gazebo world (coming soon)

## Next Steps (Week 2)

- Multi-robot launch system
- Map sharing between robots
- Map merging algorithm
- RViz configuration with semantic markers

