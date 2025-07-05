# Multi-AUV Formation Simulation

This setup extends the Orca4 AUV simulation to support 3 AUVs flying in formation. AUV1 acts as the leader, while AUV2 and AUV3 automatically follow in a triangular formation.

## Features

- **3 AUV Simulation**: Complete simulation of 3 independent AUVs with separate ArduSub instances
- **Formation Flying**: Automatic formation control with AUV1 as leader
- **Individual Camera Feeds**: Each AUV has its own stereo camera system
- **Trajectory Visualization**: Real-time path visualization for all AUVs in RViz
- **Proper Port Management**: Each AUV uses different ArduPilot ports (I0, I1, I2)
- **Namespaced Topics**: All ROS topics are properly namespaced (auv1/, auv2/, auv3/)

## AUV Configuration

### Port Assignments
- **AUV1 (Leader, I0)**: 
  - FDM ports: 9002/9003
  - GCS connection: UDP:14550
  - MAVROS TCP: localhost:5760
- **AUV2 (Follower, I1)**:
  - FDM ports: 9012/9013  
  - GCS connection: UDP:14560
  - MAVROS TCP: localhost:5761
- **AUV3 (Follower, I2)**:
  - FDM ports: 9022/9023
  - GCS connection: UDP:14570
  - MAVROS TCP: localhost:5762

### Formation Layout
```
    AUV2 (Left)
        *
         \
          \
           * AUV1 (Leader)
          /
         /
        *
    AUV3 (Right)
```

- AUV1: Leader at formation center
- AUV2: Left follower, 3m behind and 2m to port
- AUV3: Right follower, 3m behind and 2m to starboard

## Usage

### 1. Build the Docker Image
```bash
cd /home/udy/orca4/docker
./build.sh
```

### 2. Launch the Multi-AUV Simulation
```bash
./run.sh
ros2 launch orca_bringup multi_auv_sim_launch.py
```

This will start:
- Gazebo with 3 AUVs in formation starting positions
- ArduSub instances for each AUV (-I0, -I1, -I2)
- MAVROS nodes for each AUV with proper port configuration
- Base controllers and SLAM for each AUV
- Camera image bridges for all stereo cameras
- Formation controller (followers automatically follow leader)
- Path visualization publisher
- RViz with multi-AUV configuration

### 3. Control the Formation

The formation controller automatically manages AUV2 and AUV3 to follow AUV1. You can control the entire formation by sending commands to AUV1.

#### Option A: Manual Control
Send velocity commands to AUV1:
```bash
# Move forward
ros2 topic pub /auv1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Turn left
ros2 topic pub /auv1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.3}}" --once

# Stop
ros2 topic pub /auv1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

#### Option B: Automated Mission
Run a pre-programmed formation mission:
```bash
ros2 run orca_base formation_mission_runner.py
```

### 4. Monitor the AUVs

#### RViz Visualization
The custom RViz configuration shows:
- All AUV poses and orientations (color-coded)
- Camera feeds from each AUV (left cameras displayed by default)
- Real-time trajectory paths for all AUVs
- TF frames for each AUV namespace

#### Camera Feeds
Individual camera topics:
- AUV1: `/auv1/stereo_left`, `/auv1/stereo_right`
- AUV2: `/auv2/stereo_left`, `/auv2/stereo_right`  
- AUV3: `/auv3/stereo_left`, `/auv3/stereo_right`

#### Trajectory Paths
Real-time path visualization:
- AUV1: `/auv1/path` (Magenta)
- AUV2: `/auv2/path` (Green)
- AUV3: `/auv3/path` (Blue)

### 5. Advanced Usage

#### Individual AUV Control
You can also control followers individually if needed:
```bash
# Control AUV2 directly
ros2 topic pub /auv2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# Control AUV3 directly  
ros2 topic pub /auv3/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

#### Formation Parameters
The formation controller can be tuned by modifying:
- `formation_distance`: Distance followers stay behind leader (default: 3.0m)
- `formation_offset`: Lateral separation of followers (default: 2.0m)
- `kp_linear`, `kp_angular`: Control gains
- `max_linear_speed`, `max_angular_speed`: Speed limits

## File Structure

### New Model Files
- `orca_description/models/auv1/` - AUV1 model with port 9002
- `orca_description/models/auv2/` - AUV2 model with port 9012  
- `orca_description/models/auv3/` - AUV3 model with port 9022
- `orca_description/worlds/multi_auv_sand.world` - World with 3 AUVs

### New Launch Files
- `orca_bringup/launch/multi_auv_sim_launch.py` - Main multi-AUV launcher
- `orca_bringup/launch/multi_auv_bringup.py` - Coordinates all AUV nodes

### New Parameter Files
- `orca_bringup/params/auv1_mavros_params.yaml` - MAVROS config for AUV1
- `orca_bringup/params/auv2_mavros_params.yaml` - MAVROS config for AUV2
- `orca_bringup/params/auv3_mavros_params.yaml` - MAVROS config for AUV3
- `orca_bringup/params/auv1_orca_params.yaml` - Orca config for AUV1
- `orca_bringup/params/auv2_orca_params.yaml` - Orca config for AUV2
- `orca_bringup/params/auv3_orca_params.yaml` - Orca config for AUV3

### New Scripts
- `orca_base/scripts/formation_controller.py` - Formation control logic
- `orca_base/scripts/multi_auv_path_publisher.py` - Trajectory visualization
- `orca_base/scripts/formation_mission_runner.py` - Example mission

### New Configuration
- `orca_bringup/cfg/multi_auv_sim_launch.rviz` - RViz config for multi-AUV

## Troubleshooting

### AUVs Not Following Formation
- Check that the formation controller is running: `ros2 node list | grep formation`
- Verify odometry topics are publishing: `ros2 topic list | grep odometry`
- Check for error messages in the formation controller logs

### Missing Camera Feeds
- Verify image bridge is running: `ros2 node list | grep image_bridge`
- Check camera topics: `ros2 topic list | grep stereo`
- Ensure Gazebo is publishing camera data

### ArduSub Connection Issues
- Verify all ArduSub instances are running with correct -I arguments
- Check MAVROS connections: `ros2 topic list | grep mavros`
- Ensure ports 5760, 5761, 5762 are available

### TF Transform Issues

If you see warnings like "No transform from [auvX/base_link] to [map]" in RViz:

1. **Wait for initialization**: These warnings are normal during the first 10-15 seconds as the system initializes.

2. **Check TF status**: Use the quick TF checker:
   ```bash
   ./quick_tf_check.sh
   ```

3. **Verify static transforms**: Ensure all static transform publishers are running:
   ```bash
   ros2 node list | grep static_transform_publisher
   ```

4. **Check base controller status**: The base controllers need some time to initialize:
   ```bash
   ros2 topic echo /auv1/rosout | grep base_controller
   ```

The TF tree structure for each AUV should be:
```
map -> auvX/map -> auvX/slam -> auvX/down
               \-> auvX/odom -> auvX/base_link -> auvX/left_camera_link
```

#### Recent Fixes Applied

- **Static transforms**: Added initial static transforms that are immediately available when the system starts
- **Base controller compatibility**: Ensured the base controllers get the transforms they need during startup
- **RViz compatibility**: All necessary transforms are now published early enough for RViz to work correctly
- **Dual transform strategy**: Static transforms provide initial connectivity, then dynamic transforms take over

1. **Check the TF tree**: Run the TF tree checker:
   ```bash
   ./check_tf_tree.sh
   ```

2. **Verify static transforms**: Ensure all static transform publishers are running:
   ```bash
   ros2 node list | grep static_transform_publisher
   ```

3. **Check base controller status**: The base controllers need static transforms to initialize:
   ```bash
   ros2 topic echo /auv1/base_controller/get_logger   # Check for state messages
   ```

The TF tree structure for each AUV should be:
```
map -> auvX/map -> auvX/slam -> auvX/down
               \-> auvX/odom -> auvX/base_link -> auvX/left_camera_link
```

Most TF warnings resolve within 10-15 seconds as the system initializes and the base controllers start publishing dynamic transforms.

### Performance Issues
- Reduce camera update rates in model.sdf files
- Disable unnecessary displays in RViz
- Limit path history length in path publisher

## Extending the System

### Adding More AUVs
1. Create new model directory (e.g., `auv4/`)
2. Update model.sdf with new port (I3 uses 9032/9033)
3. Add parameter files for new AUV
4. Update launch files to include new AUV
5. Modify formation controller for new formation pattern

### Custom Formation Patterns
Modify `formation_controller.py` to implement:
- Line formations
- Diamond formations  
- Dynamic formations
- Obstacle avoidance formations

### Mission Planning
Extend `formation_mission_runner.py` with:
- Waypoint navigation
- GPS coordinate missions
- Depth control missions
- Search patterns

This multi-AUV setup provides a comprehensive platform for testing formation control, cooperative behaviors, and multi-vehicle coordination in underwater environments.
