
#pragma once

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <std_msgs/Time.h>
#include <mutex>
#include <signal.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>

using namespace UNITREE_LEGGED_SDK;

/**
 * @brief Custom
 */
class UnitreeLeggedSDKToROS
{
public:
  Safety safe_;
  UDP udp_;
  HighCmd cmd_ = {0};
  HighState state_ = {0};
  float dt_ = 0.005;
  uint8_t vel_cmd_mode_ = 2;

  ros::NodeHandle nh_;
  ros::Subscriber sub_cmd_vel_;
  tf::TransformListener listener_;
  tf::TransformBroadcaster bc_odom_;

  ros::Publisher pub_robot_imu_;
  ros::Publisher pub_robot_odometry_3d_;
  ros::Publisher pub_robot_odometry_2d_;

  tf::Transform pose_sdk_init_3d_;
  tf::Transform pose_sdk_init_2d_;
  tf::Transform pose_sdk_cur_3d_;
  tf::Transform pose_sdk_cur_2d_;

  tf::Transform pose_wld_cur_3d_;
  tf::Transform pose_wld_cur_2d_;

  ros::Time stamp_new_state_;
  ros::Time stamp_construct_;

  bool has_pose_sdk_init_ = false;

  geometry_msgs::Twist cmd_vel_msg_;
  double stamp_vel_cmd_last_ = -1;

public:
  UnitreeLeggedSDKToROS(ros::NodeHandle nh,
                        float period, int control_level, LeggedType leg_type,
                        uint16_t local_port, const char *target_ip, uint16_t target_port,
                        int vel_cmd_mode)
      : safe_(leg_type),
        udp_(local_port, target_ip, target_port, sizeof(HighCmd), sizeof(HighState)),
        vel_cmd_mode_(vel_cmd_mode), dt_(period)
  {

    udp_.InitCmdData(cmd_);

    nh_ = nh;

    sub_cmd_vel_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 2, &UnitreeLeggedSDKToROS::cmdVelHandler, this);

    pub_robot_imu_ = nh_.advertise<sensor_msgs::Imu>("/imu_raw", 1000);

    pub_robot_odometry_3d_ = nh.advertise<nav_msgs::Odometry>("/sdk2ros/odom/3d", 50);
    pub_robot_odometry_2d_ = nh.advertise<nav_msgs::Odometry>("/sdk2ros/odom/2d", 50);

    stamp_construct_ = ros::Time::now();
  }

  void cmdVelHandler(const geometry_msgs::Twist::ConstPtr &msg)
  {
    cmd_vel_msg_ = *msg;
    stamp_vel_cmd_last_ = ros::Time::now().toSec();

    resetVelCmdToZero();
    cmd_.mode = vel_cmd_mode_;
    cmd_.velocity[0] = msg->linear.x;
    cmd_.velocity[1] = msg->linear.y;
    cmd_.yawSpeed = msg->angular.z;
  }

  /**
   * @brief update UDPRecv() and UDPSend() once
   */
  void updateOnce()
  {
    UDPRecv();
    UDPSend();
  }

  void UDPRecv()
  {

    udp_.Recv();
    udp_.GetRecv(state_);

    stamp_new_state_ = ros::Time::now();
    if ((stamp_new_state_ - stamp_construct_).toSec() < 0.5)
    {
      return;
    }

    publishRobotIMU();

    updateOdometry();
    publishRobotOdometry();
  }

  void UDPSend()
  {
    udp_.SetSend(cmd_);
    udp_.Send();

    if (ros::Time::now().toSec() - stamp_vel_cmd_last_ > 0.5)
    {
      resetVelCmdToZero();
    }
  }

  void UDPSend_Go1()
  {
    char buf[13] = "[encryptCRC]";
    udp_.SetSend(cmd_);
    udp_.SetSend(buf);
    udp_.Send();

    if (ros::Time::now().toSec() - stamp_vel_cmd_last_ > 0.5)
    {
      resetVelCmdToZero();
    }
  }

  void publishRobotIMU()
  {

    sensor_msgs::Imu dogImu;
    dogImu.header.frame_id = "imu_link";
    dogImu.header.stamp = stamp_new_state_;

    // dogImu.orientation.w = state_.imu.quaternion[3];
    // dogImu.orientation.x = state_.imu.quaternion[2];
    // dogImu.orientation.y = state_.imu.quaternion[1];
    // dogImu.orientation.z = state_.imu.quaternion[0];

    tf::Quaternion q3d;
    q3d.setRPY(state_.imu.rpy[0], state_.imu.rpy[1], state_.imu.rpy[2]);
    dogImu.orientation.w = q3d.w();
    dogImu.orientation.x = q3d.x();
    dogImu.orientation.y = q3d.y();
    dogImu.orientation.z = q3d.z();


    dogImu.angular_velocity.x = state_.imu.gyroscope[0];
    dogImu.angular_velocity.y = state_.imu.gyroscope[1];
    dogImu.angular_velocity.z = state_.imu.gyroscope[2];

    dogImu.linear_acceleration.x = state_.imu.accelerometer[0];
    dogImu.linear_acceleration.y = state_.imu.accelerometer[1];
    dogImu.linear_acceleration.z = state_.imu.accelerometer[2];

    pub_robot_imu_.publish(dogImu);
  }

  void publishRobotOdometry()
  {

    if (!has_pose_sdk_init_)
    {
      return;
    }

    pose_wld_cur_3d_ = pose_sdk_init_3d_.inverse() * pose_sdk_cur_3d_;
    pose_wld_cur_3d_.setRotation(pose_wld_cur_3d_.getRotation().normalize());

    pose_wld_cur_2d_ = pose_sdk_init_2d_.inverse() * pose_sdk_cur_2d_;
    pose_wld_cur_2d_.setRotation(pose_wld_cur_2d_.getRotation().normalize());

    geometry_msgs::TransformStamped pose_2d;
    pose_2d.header.stamp = stamp_new_state_;
    pose_2d.header.frame_id = "sdk_odom";
    pose_2d.child_frame_id = "sdk_base_link_2d";
    tf::transformTFToMsg(pose_wld_cur_2d_, pose_2d.transform);

    geometry_msgs::TransformStamped pose_3d;
    pose_3d.header.stamp = stamp_new_state_;
    pose_3d.header.frame_id = "sdk_odom";
    pose_3d.child_frame_id = "sdk_base_link_3d";
    tf::transformTFToMsg(pose_wld_cur_3d_, pose_3d.transform);

    nav_msgs::Odometry odom3d;
    odom3d.header.stamp = stamp_new_state_;
    odom3d.header.frame_id = "sdk_odom";
    odom3d.child_frame_id = "sdk_base_link_3d";
    odom3d.pose.pose.position.x = pose_3d.transform.translation.x;
    odom3d.pose.pose.position.y = pose_3d.transform.translation.y;
    odom3d.pose.pose.position.z = pose_3d.transform.translation.z;
    odom3d.pose.pose.orientation = pose_3d.transform.rotation;
    odom3d.twist.twist.linear.x = state_.velocity[0];
    odom3d.twist.twist.linear.y = state_.velocity[1];
    odom3d.twist.twist.linear.z = state_.velocity[2];
    odom3d.twist.twist.angular.x = state_.imu.gyroscope[0];
    odom3d.twist.twist.angular.y = state_.imu.gyroscope[1];
    odom3d.twist.twist.angular.z = state_.yawSpeed;

    nav_msgs::Odometry odom2d;
    odom2d.header.stamp = stamp_new_state_;
    odom2d.header.frame_id = "sdk_odom";
    odom2d.child_frame_id = "sdk_base_link_2d";
    odom2d.pose.pose.position.x = pose_2d.transform.translation.x;
    odom2d.pose.pose.position.y = pose_2d.transform.translation.y;
    odom2d.pose.pose.position.z = 0;
    odom2d.pose.pose.orientation = pose_2d.transform.rotation;
    odom2d.twist.twist.linear.x = state_.velocity[0];
    odom2d.twist.twist.linear.y = state_.velocity[1];
    odom2d.twist.twist.linear.z = 0;
    odom2d.twist.twist.angular.x = 0;
    odom2d.twist.twist.angular.y = 0;
    odom2d.twist.twist.angular.z = state_.yawSpeed;

    pub_robot_odometry_3d_.publish(odom3d);

    pub_robot_odometry_2d_.publish(odom2d);
  }

  void updateOdometry()
  {

    tf::Quaternion q2d;
    q2d.setRPY(0., 0., state_.imu.rpy[2]);
    tf::Vector3 p2d(state_.position[0], state_.position[1], 0);
    pose_sdk_cur_2d_ = tf::Transform(q2d, p2d);

    tf::Quaternion q3d;
    q3d.setRPY(state_.imu.rpy[0], state_.imu.rpy[1], state_.imu.rpy[2]);
    tf::Vector3 p3d(state_.position[0], state_.position[1], state_.position[2]);
    pose_sdk_cur_3d_ = tf::Transform(q3d, p3d);

    if (!has_pose_sdk_init_)
    {
      pose_sdk_init_3d_ = tf::Transform(q3d, p3d);
      pose_sdk_init_2d_ = tf::Transform(q2d, p2d);
      has_pose_sdk_init_ = true;
      return;
    }
  }

  /**
   * @brief Reset cmd_ to zero
   * @note Only for a1, go1
   */
  void resetVelCmdToZero()
  {
    cmd_.mode = 0;
    cmd_.velocity[0] = 0.0f;
    cmd_.velocity[1] = 0.0f;
    cmd_.yawSpeed = 0.0f;

    cmd_.bodyHeight = 0.0f;

    cmd_.euler[0] = 0;
    cmd_.euler[1] = 0;
    cmd_.euler[2] = 0;

    cmd_.gaitType = 0;
    cmd_.levelFlag = 0;
  }
};
