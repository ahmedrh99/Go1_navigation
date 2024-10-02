## To start the robot navigation run the following 

```bash
roslaunch start navigate.launch 
```
## in another terminal run the node for sending the goal positions
```
rosrun a2_ros2udp goal_sender.py
```
## the node sends command based on the locations.yaml file located in /a2_ros2udp/params/ 
