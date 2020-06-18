rosnode info mavros (0.23.1)
--------------------------------------------------------------------------------
Node [/mavros]
Publications: 
 * /diagnostics [diagnostic_msgs/DiagnosticArray]
 * /mavlink/from [mavros_msgs/Mavlink]
 * /mavros/adsb/vehicle [mavros_msgs/ADSBVehicle]
 * /mavros/altitude [mavros_msgs/Altitude]
 * /mavros/battery [sensor_msgs/BatteryState]
 * /mavros/cam_imu_sync/cam_imu_stamp [mavros_msgs/CamIMUStamp]
 * /mavros/debug_value/debug [mavros_msgs/DebugValue]
 * /mavros/debug_value/debug_vector [mavros_msgs/DebugValue]
 * /mavros/debug_value/named_value_float [mavros_msgs/DebugValue]
 * /mavros/debug_value/named_value_int [mavros_msgs/DebugValue]
 * /mavros/distance_sensor/hrlv_ez4_pub [sensor_msgs/Range]
 * /mavros/distance_sensor/lidarlite_pub [sensor_msgs/Range]
 * /mavros/extended_state [mavros_msgs/ExtendedState]
 * /mavros/global_position/compass_hdg [std_msgs/Float64]
 * /mavros/global_position/global [sensor_msgs/NavSatFix]
 * /mavros/global_position/gp_lp_offset [geometry_msgs/PoseStamped]
 * /mavros/global_position/gp_origin [geographic_msgs/GeoPointStamped]
 * /mavros/global_position/local [nav_msgs/Odometry]
 * /mavros/global_position/raw/fix [sensor_msgs/NavSatFix]
 * /mavros/global_position/raw/gps_vel [geometry_msgs/TwistStamped]
 * /mavros/global_position/rel_alt [std_msgs/Float64]
 * /mavros/home_position/home [mavros_msgs/HomePosition]
 * /mavros/imu/atm_pressure [sensor_msgs/FluidPressure]
 * /mavros/imu/data [sensor_msgs/Imu]
 * /mavros/imu/data_raw [sensor_msgs/Imu]
 * /mavros/imu/mag [sensor_msgs/MagneticField]
 * /mavros/imu/temperature [sensor_msgs/Temperature]
 * /mavros/local_position/odom [nav_msgs/Odometry]
 * /mavros/local_position/pose [geometry_msgs/PoseStamped]
 * /mavros/local_position/velocity [geometry_msgs/TwistStamped]
 * /mavros/manual_control/control [mavros_msgs/ManualControl]
 * /mavros/px4flow/ground_distance [sensor_msgs/Range]
 * /mavros/px4flow/raw/optical_flow_rad [mavros_msgs/OpticalFlowRad]
 * /mavros/px4flow/temperature [sensor_msgs/Temperature]
 * /mavros/radio_status [mavros_msgs/RadioStatus]
 * /mavros/rc/in [mavros_msgs/RCIn]
 * /mavros/rc/out [mavros_msgs/RCOut]
 * /mavros/safety_area/get [geometry_msgs/PolygonStamped]
 * /mavros/setpoint_raw/target_attitude [mavros_msgs/AttitudeTarget]
 * /mavros/setpoint_raw/target_global [mavros_msgs/GlobalPositionTarget]
 * /mavros/setpoint_raw/target_local [mavros_msgs/PositionTarget]
 * /mavros/state [mavros_msgs/State]
 * /mavros/target_actuator_control [mavros_msgs/ActuatorControl]
 * /mavros/time_reference [sensor_msgs/TimeReference]
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /mavlink/to [unknown type]
 * /mavros/actuator_control [unknown type]
 * /mavros/adsb/send [unknown type]
 * /mavros/debug_value/send [unknown type]
 * /mavros/distance_sensor/laser_1_sub [unknown type]
 * /mavros/distance_sensor/sonar_1_sub [unknown type]
 * /mavros/fake_gps/mocap/tf [unknown type]
 * /mavros/global_position/global [sensor_msgs/NavSatFix]
 * /mavros/global_position/home [unknown type]
 * /mavros/global_position/set_gp_origin [unknown type]
 * /mavros/home_position/set [unknown type]
 * /mavros/local_position/pose [geometry_msgs/PoseStamped]
 * /mavros/manual_control/send [unknown type]
 * /mavros/mocap/pose [unknown type]
 * /mavros/odometry/odom [unknown type]
 * /mavros/rc/override [unknown type]
 * /mavros/safety_area/set [unknown type]
 * /mavros/setpoint_accel/accel [unknown type]
 * /mavros/setpoint_attitude/cmd_vel [unknown type]
 * /mavros/setpoint_attitude/thrust [unknown type]
 * /mavros/setpoint_position/global [unknown type]
 * /mavros/setpoint_position/local [geometry_msgs/PoseStamped]
 * /mavros/setpoint_raw/attitude [unknown type]
 * /mavros/setpoint_raw/global [unknown type]
 * /mavros/setpoint_raw/local [unknown type]
 * /mavros/setpoint_velocity/cmd_vel [unknown type]
 * /mavros/setpoint_velocity/cmd_vel_unstamped [unknown type]
 * /mavros/vision_pose/pose [geometry_msgs/PoseStamped]
 * /mavros/vision_pose/pose_cov [unknown type]
 * /mavros/vision_speed/speed_vector [unknown type]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]

Services: 
 * /mavros/cmd/arming
 * /mavros/cmd/command
 * /mavros/cmd/command_int
 * /mavros/cmd/land
 * /mavros/cmd/set_home
 * /mavros/cmd/takeoff
 * /mavros/cmd/trigger_control
 * /mavros/get_loggers
 * /mavros/home_position/req_update
 * /mavros/param/get
 * /mavros/param/pull
 * /mavros/param/push
 * /mavros/param/set
 * /mavros/set_logger_level
 * /mavros/set_mode
 * /mavros/set_stream_rate
 * /mavros/setpoint_position/mav_frame
 * /mavros/setpoint_velocity/mav_frame