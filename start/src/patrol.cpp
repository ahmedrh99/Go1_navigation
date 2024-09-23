#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <std_msgs/UInt8.h>
#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>




typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

	ros::init(argc, argv, "simple_navigation_goals");
	ros::NodeHandle n;



	MoveBaseClient ac("move_base", true);


	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("MLB: Waiting for the move_base action server to come up");
	}


	move_base_msgs::MoveBaseGoal goal;
	std::string patrol_points_file_path;
	float point_x = 0;
	float point_y = 0;
	float yaw = 0;
	tf2::Quaternion dog_pose;
	float stay_time = 0;
	std_msgs::UInt8 stand;
	std_msgs::UInt8 walk;
	stand.data = 1;
	walk.data = 2;


	ros::Publisher dog_mode_pub = n.advertise<std_msgs::UInt8>("/dog_mode", 10);


	ros::param::get("send_patrol_points/patrol_points_file", patrol_points_file_path);
	

	std::ifstream patrol_points_file(patrol_points_file_path.c_str());
	while (true) {

		patrol_points_file >> point_x;
		patrol_points_file >> point_y;
		patrol_points_file >> yaw;
		patrol_points_file >> stay_time;


		if (patrol_points_file.eof()) {
			break;
		}


		dog_pose.setRPY(0, 0, yaw);
		std::cout << "[MLB Goal Pose]" << point_x 
					<< " "
					<< point_y
					<< " "
					<< yaw
					<< " "
					<< stay_time << std::endl;


		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = point_x;
		goal.target_pose.pose.position.y = point_y;
		goal.target_pose.pose.orientation.x = dog_pose.x();
		goal.target_pose.pose.orientation.y = dog_pose.y();
		goal.target_pose.pose.orientation.z = dog_pose.z();
		goal.target_pose.pose.orientation.w = dog_pose.w();


		ROS_INFO("Sending goal");

		ac.sendGoal(goal);

		ac.waitForResult(ros::Duration(0));


		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("Hooray, the base moved to the POINT!");

			dog_mode_pub.publish(stand);
 
			ros::Duration(stay_time).sleep();

			dog_mode_pub.publish(walk);
		} else {
			ROS_INFO("The base failed to move to the POINT for some reason");
		}
	}

	dog_mode_pub.publish(stand);
	ros::spin();

	return 0;
}
