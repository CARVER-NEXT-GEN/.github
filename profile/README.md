# Autonomous ackerman steering mobile robot

**This is Autonomous ackerman steering mobile robot class project in FRA501(Robotics Development) at *FIBO*.**

<p align="center"><img src="images/carver iso.jpg" alt="Insert this image by CARVER picture." /></p>

<p align="center"><img src="images/carver side.jpg" alt="Insert this image by CARVER picture." /></p>

<p align="center"><img src="images/carver back.jpg" alt="Insert this image by CARVER picture." /></p>

<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
        <ul>
            <li><a href="#system-architecture">System architecture</a></li>
        </ul>
    </li>
    <li>
      <a href="#components-of-the-robot">Components of the Robot</a>
    </li>
    <li><a href="#prerequisites">Prerequisites</a></li>
        <ul>
            <li><a href="#ros2-setup-environment">ROS2 setup environment</a></li>
            <li><a href="#intallation">Intallation</a></li>
        </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <ul>
        <li><a href="#launch-the-project">Launch the project</a></li>
        <li><a href="#service-call-in-this-project">Service call in this project</a></li>
        <li><a href="#teleop_twist_keyboard">Teleop twist keyboard</a></li>
    </ul>
    <li><a href="#features">Features</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

## About The Project

This project involves the development of an autonomous Ackermann steering mobile robot. The robot can navigate to a target location independently. It is equipped with a LiDAR sensor, along with an IMU and GPS, enabling it to determine its position on the previously mapped environment.

### System architecture

<p align="center"><img src="images/Carver System Architecture - Carver System Architecture.jpg" alt="Image Description" /></p>

You can read full system architecture via this link -> [Full system architecture](https://miro.com/app/board/uXjVLuVrW2E=/?moveToWidget=3458764613100769122&cot=14})


### Demonstrate

You can click this [link](https://www.youtube.com/watch?v=LT-cnW9JqOo) or picture below to go to youtube video.

[![Watch the video](https://img.youtube.com/vi/LT-cnW9JqOo/maxresdefault.jpg)](https://www.youtube.com/watch?v=LT-cnW9JqOo)

This video will demonstrate how this robot do mapping using SLAM.

## Components of the Robot

In this section show what sensors, actuators, microcontrollers or etc. we use in this project. And you can click the link that embeded on components name to see what they use for or how it's work.

### 1. Laser and Sensor

- [2D RPLIDAR C1 Laser](https://github.com/CARVER-NEXT-GEN/merge_2D_Lidar) (GIT)
- [3D LSLIDAR C16](https://github.com/CARVER-NEXT-GEN/LSLiDAR-C16-3D-LiDAR) (GIT)
- [LD-MRS400001](https://github.com/CARVER-NEXT-GEN/SICK-LD-MRS400001-3D-LiDAR) (GIT)
- [Realsense L515](https://github.com/CARVER-NEXT-GEN/Intel-RealSense) (GIT)

### 2. IMU

- [BNO055](https://github.com/CARVER-NEXT-GEN/Adafruit-9-DOF-Absolute-Orientation-IMU-Fusion-Breakout---BNO055/tree/G4_UROS_UART) (GIT)
- [BNO086](https://github.com/CARVER-NEXT-GEN/SparkFun-VR-IMU-Breakout-BNO086-Qwiic-SPI/tree/G4_UROS_UART) (GIT)

### 3. GPS

- [SparkFun GPS-RTK Dead Reckoning Kit](https://github.com/CARVER-NEXT-GEN/carver_gps) (GIT)
- [GNSS Multi-Band L1/L2 Surveying Antenna](https://github.com/CARVER-NEXT-GEN/carver_gps) (GIT)

### 4. Microcontroller

- [STM32G474RE](https://www.st.com/en/microcontrollers-microprocessors/stm32g474re.html)
- [ESP32 S3 Pico](https://www.waveshare.com/wiki/ESP32-S3-Pico)

### 5. Motor

- [1000 W 48 V Brushless Hub Motor](https://evgracias.com/product-view/tsuyo-bldc-brushless-hub-motor1000-w-48-v) 
- [MY1016Z-250W24V](https://www.amazon.com/MY1016Z-Electric-Brushed-Wheelchair-Joystick/dp/B08PVFSSJK)

### 6. Motor Driver

- [MD20A-cytron 20Amp 6V-30V](https://th.aliexpress.com/item/1005006861348837.html)

### 7. Encoder

- [AMT21EV](https://github.com/CARVER-NEXT-GEN/AMT212E-V/tree/G474RE) (GIT)

### 8. Electrical
- [README](https://github.com/CARVER-NEXT-GEN/Carver_Electrical)

## Prerequisites

In this section show what you need to install before using this project.

⚠️**Warning:** Make sure your OS is `Ubuntu 22.04` and you already install `ROS2 Humble` on your computer. If you're not install Ubuntu 22.04 and ROS2 Humble you can follow the [Ubuntu 22.04.5 LTS (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/) and [Official ROS2 documentation](https://docs.ros.org/en/humble/index.html) to install. Additionally, this project require basic Ubuntu knowledge and intermediate ROS2 knowledge.

### ROS2 setup environment

Follow the [Official ROS 2 tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) to create your Colcon workspace.

```bash
mkdir ~/your_workspace
cd ~/your_workspace
colcon build
source install/setup.bash
```
To automate workspace setup every time you open a new terminal, add the following line to your `~/.bashrc` or `~/.bash_profile` file:

```bash
echo "source ~/your_workspace/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Intallation

⚠️**Warning:** Before using this project you need to follow all the components section that have `(GIT)` behide the name of component to setup the components. (<a href="#components-of-the-robot">back to Components</a>)

To installation this project

```bash
cd ~/your_workspace
git clone https://github.com/CARVER-NEXT-GEN/CARVER_WS.git
```

## Usage

In this section show how to use this project.

- To use any feature in this project you can following command below.

  ```bash
  ros2 launch feature_package feature_launch_file
  ```

  or

  ```bash
  ros2 run feature_package feature_launch_file
  ```

  example: If you want to creating the map, run this command below.

  ```bash
  ros2 launch carver_controller carver_mapping.launch.py
  ```  

## Features

### Carver Interface

<p align="center"><img src="images/carver interfaces.jpg" alt="Insert this image with carver interface image" /></p>

The Carver system features a robust Human-Machine Interface (HMI) to provide intuitive control and operational flexibility. Users can select various modes and interact with the system effectively:

1. **Mode Selection**
   - Modes available:
     - **Steering Mode:**
       - Allows users to control the steering manually using their hands.
       - Users can twist the hand accelerator to move Carver faster and apply the brake to slow it down.
       - Provides two buttons: *Forward* and *Backward* for switching between forward gear and reverse gear.
     - **Teleoperation (Teleop) Mode:**
       - Enables users to control the steering via Carver's website.
       - Users can control the steering motor to turn left or right and adjust the speed using the BL motor.
       - Offers the same functionality as manual control but remotely through the web interface.
     - **Joystick Mode:**
       - Allows control using a remote control device.
       - Users can control the steering motor for turning and adjust the speed via the BL motor.
       - Provides functionality identical to manual control but operated through the joystick.
     - **Auto Mode:**
       - Enables autonomous driving.
       - Users can specify a destination on a map, and Carver will automatically navigate to the specified point.

2. **Accelerator and Brake Control**
   - Hand Accelerator:
     - Enables Carver to move faster when twisted by hand.
   - Steering Brake:
     - Allows users to slow down Carver as needed.

3. **State Monitoring**
   - Reads and monitors the state of:
     - Emergency
     - Brake
     - Accelerator

4. **Lighting Control**
   - Supports controlling the following lights on Carver:
     - **Front Light**
     - **Back Light**
     - **Turn Signal Lights**
     - **Parking Lights**

To use interfaces you can follow [Carver_Interface
](https://github.com/CARVER-NEXT-GEN/Carver_Interface/tree/main). After follow this repository you will got micro_ros agent to connect main computer with microcontroller.

*rqt graph of carver interfaces*

<p align="center"><img src="images/rqt_uros.png" alt="" /></p>

- **uros_carver_interface_node**
  This node will publish every data from microcontroller.

And to control robot with interface, data from carver interface will send to `carver odrive manual steering` to control wheel's motor.

*rqt graph of carver interfaces with carver odrive manual steering*

<p align="center"><img src="images/rqt_uros_odrive.png" alt="" /></p>

- **carver_odrive_manual_steering**
  This node will subscribe the data from carver interface and publish cmd_vel to control wheel's motor.

### Visualization

#### 1. Carver description

You can use the [Carver_Description_package](https://github.com/CARVER-NEXT-GEN/carver_description) to visualize the robot in rviz2. It requires the MESH and URDF files of the robot, with the correct coordinate frame of the IMU sensor and base link.

1. rviz2 window after use Carver_Description_package

  ![image](https://scontent.fbkk12-4.fna.fbcdn.net/v/t1.15752-9/464679391_988248309799997_8734980141970335225_n.png?_nc_cat=109&ccb=1-7&_nc_sid=9f807c&_nc_ohc=7LctUSIbGLQQ7kNvgHRmSLX&_nc_zt=23&_nc_ht=scontent.fbkk12-4.fna&oh=03_Q7cD1gG_EyTmnSigVNgUkigzCWyMdj7vb_HWhkh8Kf_DcAMOuw&oe=67AC9302)

2. Transformation of robot with robot description

<p align="center"><img src="images/tf_with_robot.png" alt="" /></p>

3. Only transformation of robot

<p align="center"><img src="images/only_tf.png" alt="" /></p>

#### 2. View Transformations Execute the following command to watch the relationships between the robot's transformations
  
```bash
ros2 run tf2_tools view_frames 
```

This command opens a graphical interface displaying the relationships between different frames. The provided link shows an example image capturing these transformations:

<p align="center"><img src="images/tf2_view.png" alt="" /></p>

Use this visualization to understand how different parts of the robot are related in terms of coordinate frames. Feel free to adjust the paths and commands based on your actual file locations and robot system. Happy exploring!

*rqt graph of carver description*

<p align="center"><img src="images/description_rqt.png" alt="" /></p>

- **rviz**
  This node will subscribe any topic to visualize.
- **robot_state_publisher**
  This node will publish transformation by "/tf" and "/tf_static" of robot and publish "/robot_description" for visualize model robot and relationship between frame by URDF file.

### Sensors preparation

#### RPLiDAR

In Carver description you can see the laser scan around robot that come from 2 rplidar and we need to merge them into one laser scan data.

1. We use [rplidar_ros](https://github.com/CARVER-NEXT-GEN/rplidar_ros.git) to started 2 rplidar. After that we will have 2 node call `rplidar_node_1` and `rplidar_node_2`. They publish 2 laser scan data topic call `lidar_1/scan` and `lidar_2/scan`.

2. After we have laser scan data from those sensor, we merge 2 lidars together. To merge lidar data we use [ros2_laser_scan_merger](https://github.com/CARVER-NEXT-GEN/ros2_laser_scan_merger.git) to merge data and then it will transform to point cloud in topic `cloud_in`

<p align="center"><img src="images/Before merge lidar.png" alt="" /></p>

3. After we have point cloud from those lidars, we must transform point cloud back to laser scan. we use [pointcloud_to_laserscan](https://github.com/CARVER-NEXT-GEN/pointcloud_to_laserscan.git) to transfrom point cloud data to laser scan for use with slamtoolbox, at last it will publish laser scan data topic call`scan`.

<p align="center"><img src="images/After merge lidar.png" alt="" /></p>

*rqt graph of lidar data*
<p align="center"><img src="images/rqt_lidar.png" alt="" /></p>

- **rplidar_node_1 and rplidar_node_2**
  This node will start two lidar and publish laser scan with "/lidar_1/scan" and "/lidar_2/scan".
- **ros2_laser_scan_merger**
  This node will subscribe"/lidar_1/scan" and "/lidar_2/scan" to merge laser scan to point cloud and publish "/cloud_in".
- **pointcloud_to_laserscan**
  This node will convert point cloud from "/cloud_in" to laser scan and publish "/scan".

### Mapping

To perform map creation, we merged point cloud data from multiple sensors on the robot to enhance the accuracy and completeness of the collected environment data. This data is then utilized in LiDAR based SLAM (Simultaneous Localization and Mapping), a process that allows the robot to simultaneously map the environment while estimating its own position within it. By integrating point clouds from various sensors, we reduce blind spots, improve map resolution, and ensure consistency in the representation of the environment. This approach enables the robot to construct a detailed and precise map of the desired location, even in complex or dynamic environments.

![image](https://scontent.fbkk12-2.fna.fbcdn.net/v/t1.15752-9/472821627_1688660595367736_5944732485133289307_n.jpg?_nc_cat=104&ccb=1-7&_nc_sid=9f807c&_nc_ohc=TiVNBQVh1IoQ7kNvgEqHx5T&_nc_zt=23&_nc_ht=scontent.fbkk12-2.fna&oh=03_Q7cD1gHXZnN6kRi87J8V_PxUG7YZ-bHHApfDnkU25smuwzn-TA&oe=67AC956C)

#### 1. Odometry

The robot use odometry to estimate position. We use odometry from yaw_rate that incorporates feedback from wheel velocity and yaw information from IMU, But it still have error or slip.

To fix this we use Extended Kalman Filter (EKF) implemented with the robot_localization package.

EKF will estimations of pose for reduce noisy or gaps in the data during the estimation process. and at last EKF will get **"/odometry/filtered"** topic.


#### 2. Creating map

To create a map, run following command:

```bash
ros2 launch carver_controller carver_mapping.launch.py
```

After you have created a map, you can save it using the following command:

```bash
ros2 run nav2_map_server map_saver_cli -f <map_name>
```

This launch file contains the following nodes:

<p align="center"><img src="images/rqt_full.png" alt="Insert this image with Teleop website" /></p>

Node for visualize robot description
- **rviz**
  This node will subscribe any topic to visualize.
- **robot_state_publisher**
  This node will publish transformation by "/tf" and "/tf_static" of robot and publish "/robot_description" for visualize model robot and relationship between frame by URDF file.

Node from low-level (STM32)
- **uros_AMT_Node**
  This will subscribe "/steering_angle" and "/steering_mode" to control real steering and publish "/amt_publisher" that are angle position.
- **uros_carver_interface_node**
  This node will send state of emergency button("/carver_emergency"), throttle data("/accl_publisher") and forward and reverse direction of robot from button ("/accel_direction").  
- **bno_055_node**
  This node will send imu data of BNO055 in float array "/bno055_cubemx".
- **bno_086_node**
  This node will send imu data of BNO086 in float array "/bno086_cubemx".

Node about lidar (RPLIDAR s3)
- **rplidar_node_1 and rplidar_node_2**
  This node will start two lidar and publish laser scan with "/lidar_1/scan" and "/lidar_2/scan".
- **ros2_laser_scan_merger**
  This node will subscribe"/lidar_1/scan" and "/lidar_2/scan" to merge laser scan to point cloud and publish "/cloud_in".
- **pointcloud_to_laserscan**
  This node will convert point cloud from "/cloud_in" to laser scan and publish "/scan".

Node for calculation odometry
- **carver_odrive_manual_steering**
  This node will subscribe "/accl_publisher" and "/accel_direction" to calculate wheel speed and publish "/feedback_wheelspeed".
- **carver_messenger**
  This node will subscribe "/bno055_cubemx" and "/bno086_cubemx" to put in Imu Message and MagneticField Message and publish it by "/imu_086/data", "/imu_086/mag", "/imu_055/data", "/imu_055/mag".
- **ackerman_odom**
  This node will subscribe "/feedback_wheelspeed" and "/imu_055/data" to create odom that calculated using a yaw_rate and publish "/yaw_rate/odom".

Node about mapping and localization
- **/ekf_filter_node**
  This node will subscribe "/yaw_rate/odom" and "/imu_055/data" to fusion is achieved through an Extended Kalman Filter (EKF) implemented with the robot_localization package, then publish "/odometry/filtered" and "/tf".
- **/slam_toolbox**
  This node will subscribe "/scan" for mapping and localization then it publish "/map".

#### 3. Using map

⚠️**Warning:** This feature is in progress, we will update soon.

To use map that you create, you need to use some tool for editing some area in map that you need to do. After that you must ..........

### Localization

⚠️**Warning:** This feature is in progress, we will update soon.

We can visualize our robot on the mapped environment in real-time, allowing us to monitor its position and movement within the map dynamically.

### Teleoperation

⚠️**Warning:** This feature is in progress, we will update soon.

This feature is about to control robot via website call teleoperation mode. To connect the website you can click this link [Teleoperation website](). After you connect to website you will able to control robot but you need to change mode of robot at carver interfaces first.

<p align="center"><img src="" alt="Insert this image with Teleop website" /></p>

### Navigation

⚠️**Warning:** This feature is in progress, we will update soon.

To run navigation mode of this robot you need to select mode at carver interfaces to auto mode. After that you can use this mode in rviz2 window.

<p align="center"><img src="" alt="Insert this image with rviz2 window" /></p>

#### 1. Set initial pose

To localize initial pose of robot, you can easily click `2D Pose Estimate`. After that click on position in map that is closed to robot's current position and drag toward the robot's heading.

<p align="center"><img src="" alt="Insert this image with How do initial pose" /></p>

#### 2. Set goal point

After localization, click `2D NAV Goal`. After that click position you want in map and drag toward the direction you want your robot to be heading at once it has reached its goal.

<p align="center"><img src="" alt="Insert this image with Example gif" /></p>

<p align="right">(<a href="#autonomous-ackerman-steering-mobile-robot">back to top</a>)</p>
