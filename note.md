ros2 run rplidar_ros rplidar_node --ros-args -p      
  channel_type:=serial -p serial_port:=/dev/rplidar -p serial_baudrate:=460800 -p      
  frame_id:=laser_frame -p angle_compensate:=true -p scan_mode:=Standard

ros2 run rplidar_ros rplidar_node --ros-args -p channel_type:=serial -p serial_port:=/dev/rplidar -p serial_baudrate:=460800 -p frame_id:=laser_frame -p angle_compensate:=true   -p scan_mode:=Standard"




docker exec docker-ros2_runtime-1 bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run rplidar_ros rplidar_node --ros-args -p channel_type:=serial -p serial_port:=/dev/rplidar -p serial_baudrate:=460800 -p frame_id:=laser_frame -p angle_compensate:=true -p scan_mode:=Standard"

```code
source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run rplidar_ros rplidar_node --ros-args -p channel_type:=serial -p serial_port:=/dev/rplidar -p serial_baudrate:=460800 -p frame_id:=laser_frame -p angle_compensate:=true -p scan_mode:=Standard
```


ros2 run rplidar_ros rplidar_node --ros-args -p channel_type:=serial -p serial_port:=/dev/rplidar -p serial_baudrate:=460800 -p frame_id:=laser_frame -p angle_compensate:=true -p scan_mode:=Standard