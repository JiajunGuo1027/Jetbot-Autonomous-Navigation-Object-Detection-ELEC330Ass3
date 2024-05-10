# Jetbot-Autonomous-Navigation-Object-Detection-ELEC330Ass3
ELEC330 Robotic Systems II Assignment 3. 
• Use sensors attached to the Jetbot to autonomously navigate within a set arena. 
• Get the robot to identify one object placed in the arena and park next to it. 
• Demonstrate interfacing the sensors and using ROS to broadcast sen- sors data.

## Requirements
- NVIDIA-designed open-source JetBot hardware
- Python 2.7 and Python 3.6

## Starting the NVIDIA UI
To open the NVIDIA UI interface, run the following command in the terminal: 
```bash
$ sudo systemctl isolate graphical
```

## Running the Jetbot
To run the Jetbot for obstacle avoidance and to stop when detecting a yellow toy duck, execute the following commands in the terminal:
```bash
$ cd ~/catkin_ws/src/jetbot_ros/scripts
$ roscore
$ python3 allpubtest.py
```
The files to run include `imu_F.py`, `publishTOFData.py`, `encoder.py`, `encoder_publish.py`, `duckDetect.py` and `navTue.py`. The test results are displayed in the `images` folder.

## Viewing ROS Topics
To view all subscribed and published topics, run: 
```bash
$ rostopic list
```
![rostopic list](images/rostopicList.jpg)

## Third-Party Repositories
Below you can find links to third-party libraries and repositories used in this project.

### Jetson Inference
A library for deep learning with NVIDIA Jetson and inference utilities - [Jetson Inference GitHub](https://github.com/dusty-nv/jetson-inference)

### CSI Camera
A repository containing the CSI Camera interface for NVIDIA Jetson Nano - [CSI Camera GitHub](https://github.com/JetsonHacksNano/CSI-Camera)

### Jetbot ROS
A ROS implementation for NVIDIA JetBot with deep learning and robotics capabilities integrated - [Jetbot ROS GitHub](https://github.com/dusty-nv/jetbot_ros)

### LSM303D Python
Python library for interfacing with the LSM303D accelerometer and magnetometer sensor - [LSM303D Python GitHub](https://github.com/pimoroni/lsm303d-python)

### Raspicam Node
A ROS node for the Raspberry Pi camera - [Raspicam Node GitHub](https://github.com/UbiquityRobotics/raspicam_node)

### VL53L5CX Python
Python library for the VL53L5CX time-of-flight sensor - [VL53L5CX Python GitHub](https://github.com/pimoroni/vl53l5cx-python)

### Usage
To use these repositories, refer to their respective README files for detailed installation and usage instructions.


