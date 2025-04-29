# Navigation
## all code is supposed to run from the robot itself
please make sure the robot is in the designated initial position
(note: the positions and orientations of office locations were manually registered)
## To start the robot navigation, run the following 

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
## in another terminal ssh into jetson 15, source the bash file and export the ROSMASTER as mentioned above. 
finally run the node for sending the goal positions
```
rosrun a2_ros2udp goal_sender.py
```
## the node sends command based on the locations.yaml file located in in the src folder under /a2_ros2udp/params/ 
# Mapping 
## To run mapping navigate to lio_sam/launch/
```bash
roslaunch run.launch
```
# or navigate to start/launch/
```bash 
roslaunch build_map.launch
```
