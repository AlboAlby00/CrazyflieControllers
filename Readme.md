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

thrust: 5.36815 is the threshold thrust value which let's the drone just levitate from ground very slowly
