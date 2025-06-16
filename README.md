# Overview
This package provides a ROS2 driver node for HOKUYO 3D LiDAR(SOKUIKI Sensor).

- Feature
  - Scanning data output
  - Scanning auxiliary output (use `imu` `mag` `temperature` topic)
  - Output hardware diagnostic information (use `diagnostic_updater` topic)
  - Component Mounting
  - [Life cycle control](http://design.ros2.org/articles/node_lifecycle.html)support

The interfaces and some of the processes are based on [hokuyo3d package](https://github.com/at-wat/hokuyo3d), which is often used in ROS1.

[urg3d_library](https://github.com/UrgNetwork/urg3d_library) is used for communication with LiDAR.

## Life cycle control
The operation in each state of the life cycle is as follows
- Unconfigured  
  startup state
- Inactive  
  LiDAR connection state (not delivered scanning data and diagnostic information)
- Active  
  LiDAR data delivery state (delivered scanning data and diagnostic information)
- Finalized  
  end state

# Supported models
Hokuyo's VSSP 2.1-compliant LiDAR
Tested models: `YVT-35LX-F0`, `YVT-35LX-FK`

Tested Environment: `foxy`, `galactic`, `humble`

# License
`Apache License 2.0`
The urg3d_library is licensed under the `Simplified BSD License`.

# Publish
- /hokuyo_cloud2 (sensor_msgs::msg::PointCloud2)  
  LiDAR scanning data
- /imu (sensor_msgs::msg::Imu)  
  Internal 6-D imu data (output when parameter `publish_auxiliary`=true)
- /mag (sensor_msgs::msg::MagneticField)  
  Internal 3-D MagneticField data (output when parameter `publish_auxiliary`=true)
- /temperature (sensor_msgs::msg::Temperature)  
  Internal Temperature data (output when parameter `publish_auxiliary`=true)
- /diagnostics (diagnostics_msgs::msg::DiagnosticArray)  
  Diagnostic information

# Parameters
- ip_address (string, default: "192.168.0.10")  
  IP address for Ethernet connection (Specify in the format "XX.XX.XX.XX.XX") 
- ip_port (int, default: 10940)  
  Port number for Ethernet connection
- frame_id (string, default: "hokuyo3d")  
  Frame_id of scanning data
  The parameter `frame_id` is set in the message header "frame_id" of the scan data.
- range_min (double, default: 0.1)
  Points within range_min are dropped. This parameter is useful to remove ghost points nearby.
- interlace_h (int, default: 1)
  Interlace(horizontal) setting of laser scanning. One means no interlace.
- interlace_v (int, default: 1)
  Interlace(vertical) setting of laser scanning. One means no interlace.
- output_cycle (string, default: "frame")
  Specifies timing of point cloud output. "frame": outputs for each interlace cycle (several fields), "field": one horizontal scan (tens of lines), "line": one vertical scan
- calibrate_time (bool, default: false)  
  Adjustment mode flags
  If this flag is true, the discrepancy between the LiDAR time and the system time is measured at the start of the scan and added to the timestamp of the scan data as latency.
- synchronize_time (bool, default: false)  
  Synchronous mode flags
  If this flag is true, the system time, which is the reference for the timestamp of the scan data, is dynamically corrected using the discrepancy from the LiDAR time.
- publish_intensity (bool, default: true)  
  Intensity output mode flag
  If this flag is true, the intensity data of the scan data is output; if false, the intensity data is output empty.  
  If LiDAR does not support intensity output, it works the same as the false setting.
- publish_auxiliary (bool, default: false)  
  auxiliary publish flag
  If this flag is true, auxiliary scan data (/imu, /mag, /temperature) is output 
- error_limit (int, default: 4 [count])  
  Number of errors to perform reconnection
  Reconnects the connection with LiDAR when the number of errors that occurred during data acquisition becomes larger than error_limit.    
- error_reset_period (double, default: 10.0 [sec])  
  Error reset cycle 
  Periodically resets the number of errors that occurred during data acquisition.  
  * To prevent reconnection in case of sporadic errors
- diagnostics_tolerance (double, default: 0.05)  
  Allowable percentage of diagnostic information on frequency of scan data delivery relative to target delivery frequency  
  * For example, if diagnostics_tolerance is 0.05, the normal range is 105% to 95% of the target delivery frequency.
- diagnostics_window_time (double, default: 5.0 [sec])  
  Period to measure the frequency of output delivery of diagnostic information on the frequency of scan data delivery.
- time_offset (double, default: 0.0 [sec])  
  User latency to be added to timestamp of scan data

# How to build

1. Obtaining Source Code

```
$ cd <ROS2_workspace>/src
$ git clone --recursive https://github.com/Hokuyo-aut/urg3d_node2
```

2. Installing Related Packages

```
$ rosdep update
$ rosdep install -i --from-paths urg3d_node2
```

3. Build

```
$ cd <ROS2_workspace>
$ colcon build --symlink-install
```

4. Testing (optional)

Since the test is executed for `YVT-35LX-FK`, please connect `YVT-35LX-FK` and execute it with the power ON.

```
$ cd <ROS2_workspace>
$ colcon test
```

# Examples of Use

## launch

1. Connect LiDAR  
   Connect via Ethernet. 
1. Set the connection destination (parameters)   
   Edit `config/params.yaml`    
1. Node startup
   Execute the following command to automatically transition to the Active state and begin distribution of scan data.

   ```
   $ ros2 launch urg3d_node2 urg3d_node2.launch.py
   ```

   If you do not want to automatically transition to the Active state, execute the following command. (After startup, it will enter the Unconfigured state.)

   ```
   $ ros2 launch urg_node2 urg_node2.launch.py auto_start:=false
   ```

   In `urg3d_node2.launch.py`, urg3d_node2 is launched as a standalone lifecycle node (not as a component). This is because there is no lifecycle control interface in launch when launched as a component and lifecycle node (not implemented in ROS2 at this time).

## Parameter Change

To change the parameters, perform the following steps.

1. Node Termination
   \<Ctrl-C\> to terminate the node.
1. Reset parameters  
   Edit the parameter file you are using (e.g. config/params.yaml).
1. Restarting a node  
   Start the node with the following command

   ```
   $ ros2 launch urg3d_node2 urg3d_node2.launch.py
   ```

## Reset in case of abnormality

If you encounter abnormalities, such as no scan data being delivered, please follow the steps below.

1. Node Termination
   \<Ctrl-C\> to terminate the node.
1. Connection Confirmation  
   Check if the LiDAR power supply and connections are normal.
1. Restarting a node  
   Start the node with the following command

   ```
   $ ros2 launch urg3d_node2 urg3d_node2.launch.py
   ```

# Restrictions

- in Galactic, Inactive->Active state transitions fail for the second and subsequent times.

When diagnostic_updater is generated, declare_parameter is always executed. The state transition cannot take place because of the failure of diagnostic_updater generation.  
This limitation will be resolved when the following pull request is reflected. 
  https://github.com/ros/diagnostics/pull/227

