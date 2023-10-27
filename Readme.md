



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

### coordinate system and motors and orientation
![crazyflie_coordinate_system](https://github.com/AlboAlby00/CrazyflieControllers/assets/23526716/a7b6904f-05e3-4853-8cea-51ab7b48541a)

### Overview
![msg734367861-16883](https://github.com/AlboAlby00/CrazyflieControllers/assets/23526716/53508fcd-6208-40b6-823a-055a93c55f19)



### start commands:
- launch the pid attitude controller:  `ros2 launch crazyflie_controllers attitude_pid_controller.launch.py`
- launch the pid position controller:  `ros2 launch crazyflie_controllers position_pid_controller.launch.py` (it launches both attitude and position controller)
- launch just drone with still-standing proppellers: `ros2 launch crazyflie_ros2_driver crazyflie_ros2_driver.launch.py`

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


### Bug hunting

#### Bug with the Pitch direction:

```
Crazyflie {
  translation -4.416961417915289e-15 -3.268234596351256e-15 0.46319061419783847
  rotation 0.008470942120165671 0.009841051341120451 -0.9999156948703717 1.4447209483285885e-14
  controller "<extern>"
  name "crazyflie"
}
```
![Crazyflie_Bug](https://github.com/AlboAlby00/CrazyflieControllers/assets/23526716/45f4fe91-cbef-43f8-8ff5-9cba6c02bbb4)
