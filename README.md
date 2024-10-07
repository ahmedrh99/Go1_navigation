## all code is supposed to run from the robot itself
## To start the robot navigation run the following 

ssh into into the jetson nano 15, 
```bash
ssh -X unitree@192.168.123.15
```

navigate towards 
```bash
cd /UnitreeSlam/catkin_make_3d
```
Run 
```bash
source setup.bash
```
export the ROSMASTER
```bash
export ROS_MASTER_URI=http://192.168.123.15:11311
export ROS_HOSTNAME=192.168.123.15
```
launch the navigate launch file 
```bash
roslaunch start navigate.launch 
```
## in another terminal ssh into jetson 15 also, source the bash file and export the ROSMASTER as mentioned above and run the node for sending the goal positions
```
rosrun a2_ros2udp goal_sender.py
```
## the node sends command based on the locations.yaml file located in in the src folder under /a2_ros2udp/params/ 
