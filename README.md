# arm_robot
STM32 project for control board of double 7-DOF mobile robot from HUST AC

- MCU: STM32F103RCT6
- Board: Customized (as described in `arm_robot.ioc`)
- Develop environment: STM32CubeMX + SW4STM32 (or MDK5) + J-Link
- Language: C99, C++11

# Peripherals
- PWM
  - [x] Servo
    - Camera pitch (LD-1501MG)
    - Camera yaw (LD-3015MG)
    - [x] smoothed movement
  - [ ] Lifter (customized)
- UART (non-block DMA r/w)
  - [x] PC Debug
  - [x] rosserial
    - [x] high resolution time sync (~microsecond resolution), based on Kalman filter and RTT (Round Trip Time) @ 2Hz
    - [x] publish imu bmx055 data
      - accel and gyro (`arm_robot_msgs::Imu`): imu_camera_raw @ 1kHz
      - mag (`arm_robot_msgs::MagneticField`): mag_camera_raw @ 20Hz
      - temp (`sensor_msgs::Temperature`): temp_camera @ 1Hz
    - [x] publish servo joint states (`sensor_msgs::JointState`): servo_joint_states @ 100Hz
    - [x] subscribe servo control
      -  camera servo pitch (`control_msgs::SingleJointPositionGoal`): servo/pitch
      -  camera servo yaw (`control_msgs::SingleJointPositionGoal`): servo/yaw
- RS-485
  - [ ] left arm (7-DOF customized from zeda-tech.com)
  - [ ] right arm (7-DOF customized from zeda-tech.com)
- I2C (non-block DMA r/w)
  - [x] IMU beside camera (BMX055)
  - [ ] IMU top (BMX055)
  - [ ] IMU bottom (BMX055)
- CAN
  - [ ] Left front wheel (C620 Brushless DC Motor Speed Controller)
  - [ ] Left back wheel (C620 Brushless DC Motor Speed Controller)
  - [ ] Right back wheel (C620 Brushless DC Motor Speed Controller)
  - [ ] Right front wheel (C620 Brushless DC Motor Speed Controller)
- GPIO
  - [x] Button
    - [x] press mode
    - [x] press + holding mode
    - [x] press + auto repeat mode
  - [ ] LED

# ROS side code

https://github.com/HUSTWZH/hustarm