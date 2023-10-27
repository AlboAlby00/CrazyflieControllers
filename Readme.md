



## Crazyflie 2.1

### motor numbers and spinning orientation

#### m1 - clockwise
![m1](https://github.com/AlboAlby00/CrazyflieControllers/assets/23526716/6d818960-ba34-4e41-8b84-6226d07ec23c)


#### m2 - counterclockwise
![m2](https://github.com/AlboAlby00/CrazyflieControllers/assets/23526716/22949faf-5311-4327-bd66-b3dcd1d5c061)

#### m3 - clockwise
![m3](https://github.com/AlboAlby00/CrazyflieControllers/assets/23526716/5395d51b-9080-424e-8d01-99bd3c10c1c2)

#### m4 - counterclockwise
![m4](https://github.com/AlboAlby00/CrazyflieControllers/assets/23526716/0b32600a-bad9-4c78-91b8-0753a321e449)



#### yaw generation

https://github.com/AlboAlby00/CrazyflieControllers/assets/23526716/95a2af0e-7645-4e86-b425-01dafb7f4590

### Overview
![msg734367861-16883](https://github.com/AlboAlby00/CrazyflieControllers/assets/23526716/53508fcd-6208-40b6-823a-055a93c55f19)



### start commands:
- launch the pid attitude controller:  `ros2 launch crazyflie_controllers attitude_pid_controller.launch.py`
- launch the pid position controller:  `ros2 launch crazyflie_controllers position_pid_controller.launch.py` (it launches both attitude and position controller)


### plotjuggler visualization of continuous signals
- `ros2 run plotjuggler plotjuggler`
- In Plotjuggler:
- Start streaming the ROS 2 topics
- drag the actual value the vertically splitted graphs

### rqt to receive info and send commands
- Command `rqt`
- Rqt interface and topic list:
![photo1698328232](https://github.com/AlboAlby00/CrazyflieControllers/assets/23526716/7bb07329-8aa6-478b-b2d9-063774010c98)

thrust: 5.3681225 is the threshold thrust value which let's the drone just levitate from ground very slowly
