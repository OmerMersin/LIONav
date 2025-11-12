#!/bin/bash
# ==========================================================
#  Full Drone Stack Launcher (MAVROS + Ouster + LIO-SAM)
# ==========================================================

# --- CONFIGURATION ---
ROS_DISTRO=humble
WORKSPACE=~/ros2_ws
URDF_PATH=$WORKSPACE/src/my_drone_description/urdf/drone_body.urdf
LIDAR_HOSTNAME=os-122511001246.local

# --- ENVIRONMENT SETUP ---
source /opt/ros/$ROS_DISTRO/setup.bash
source $WORKSPACE/install/setup.bash

# --- TERMINAL SETTINGS ---
TERM_CMD="gnome-terminal"  # or "x-terminal-emulator -e" if using another DE

# --- MAVROS ---
$TERM_CMD --tab --title="MAVROS" -- bash -c "
echo 'Starting MAVROS...';
source /opt/ros/$ROS_DISTRO/setup.bash;
source $WORKSPACE/install/setup.bash;
ros2 run mavros mavros_node \
  --ros-args \
  -p fcu_url:=serial:///dev/ttyTHS1:921600 \
  -p fcu_protocol:=v2.0 \
  -p use_companion_time:=true \
  -p timesync_mode:=MAVLINK \
  -p timesync_rate:=10.0 \
  -p system_time_rate:=1.0 \
  -p plugin_allowlist:=\"['sys_time','sys_status','imu','odometry','setpoint_position','setpoint_velocity','setpoint_attitude','actuator_control']\" \
  -p plugin_denylist:=\"['distance_sensor']\";
exec bash" &

# --- OUSTER DRIVER ---
$TERM_CMD --tab --title="Ouster ROS" -- bash -c "
echo 'Starting Ouster LiDAR...';
source /opt/ros/$ROS_DISTRO/setup.bash;
source $WORKSPACE/install/setup.bash;
ros2 launch ouster_ros sensor.launch.xml \
  sensor_hostname:=$LIDAR_HOSTNAME \
  timestamp_mode:=TIME_FROM_ROS_TIME;
exec bash" &

# --- LIO-SAM ---
$TERM_CMD --tab --title="LIO-SAM" -- bash -c "
echo 'Starting LIO-SAM...';
source /opt/ros/$ROS_DISTRO/setup.bash;
source $WORKSPACE/install/setup.bash;
ros2 launch lio_sam run.launch.py;
exec bash" &

# --- STATIC TF PUBLISHER ---
$TERM_CMD --tab --title="TF Publisher" -- bash -c "
echo 'Publishing static transform base_link -> os_sensor...';
source /opt/ros/$ROS_DISTRO/setup.bash;
source $WORKSPACE/install/setup.bash;
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link os_sensor;
exec bash" &

# --- ROBOT STATE PUBLISHER ---
$TERM_CMD --tab --title="Robot Model" -- bash -c "
echo 'Starting robot_state_publisher...';
source /opt/ros/$ROS_DISTRO/setup.bash;
source $WORKSPACE/install/setup.bash;
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:=\"\$(xacro $URDF_PATH)\";
exec bash" &

echo "✅ All drone nodes launched in separate tabs."
