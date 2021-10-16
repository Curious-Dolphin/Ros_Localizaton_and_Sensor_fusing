# Localization and Sensor Fusing for a Mobile Robot
This project is divide into two stages: in the first one the aim was to compute the odometry of a mobile robot using two different methods, differential drive and Ackermann steering, in the second part the focus was on fusing sensor data from three different sources, the computed odometry, the IMU data and GPS data.
All the data were contained in a bag.

## Structure of the code

The code is divided in two folders, one for the first and one for the second part of the project.
 
## Details

The first part was coded usign the following nodes:
- A node that calculates the two different odometry, diff drive and Ackermann Steering
- Custom definition for the odometry msg
- A parameter file used for the dynamic reconfigure in order to reset and switch between the published odometries.

The second part has the following features:
- Ekf localization node that fuses IMU and Odometry data
- A complementary filter that filters the raw IMU data
- A final navsat node that fuses the result of above nodes with the GPS data from the bag



## Installation

ROS 1 is required to run the code.
Clone this repository to your workspace as follows

```sh
cd ~/catkin_ws/src
git clone <this repo>
cd ~/catkin_ws
catkin_make
```
