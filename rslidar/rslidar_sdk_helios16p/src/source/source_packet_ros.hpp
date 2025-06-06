/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include "source/source_driver.hpp"

#ifdef ROS_FOUND
#include "msg/ros_msg/rslidar_packet.hpp"
#include <ros/ros.h>

namespace robosense
{
namespace lidar
{

inline Packet toRsMsg(const rslidar_msg::RslidarPacket& ros_msg)
{
  Packet rs_msg;
  rs_msg.timestamp = ros_msg.header.stamp.toSec();
  rs_msg.seq = ros_msg.header.seq;
  rs_msg.is_difop = ros_msg.is_difop;
  rs_msg.is_frame_begin = ros_msg.is_frame_begin; 

  for (size_t i = 0; i < ros_msg.data.size(); i++)
  {
    rs_msg.buf_.emplace_back(ros_msg.data[i]);
  }

  return rs_msg;
}

class SourcePacketRos : public SourceDriver
{ 
public: 

  virtual void init(const YAML::Node& config);

  SourcePacketRos();

private:

  void putPacket(const rslidar_msg::RslidarPacket& msg);

  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Subscriber pkt_sub_;
};

SourcePacketRos::SourcePacketRos()
  : SourceDriver(SourceType::MSG_FROM_ROS_PACKET)
{
}

void SourcePacketRos::init(const YAML::Node& config)
{
  SourceDriver::init(config);

  std::string ros_recv_topic;
  yamlRead<std::string>(config["ros"], "ros_recv_packet_topic", 
      ros_recv_topic, "rslidar_packets");

  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  pkt_sub_ = nh_->subscribe(ros_recv_topic, 100, &SourcePacketRos::putPacket, this);
} 

void SourcePacketRos::putPacket(const rslidar_msg::RslidarPacket& msg)
{
  driver_ptr_->decodePacket(toRsMsg(msg));
}

inline rslidar_msg::RslidarPacket toRosMsg(const Packet& rs_msg, const std::string& frame_id)
{
  rslidar_msg::RslidarPacket ros_msg;
  ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
  ros_msg.header.seq = rs_msg.seq;
  ros_msg.header.frame_id = frame_id;
  ros_msg.is_difop = rs_msg.is_difop;
  ros_msg.is_frame_begin = rs_msg.is_frame_begin;

  for (size_t i = 0; i < rs_msg.buf_.size(); i++)
  {
    ros_msg.data.emplace_back(rs_msg.buf_[i]);
  }

  return ros_msg;
}

class DestinationPacketRos : public DestinationPacket
{
public:

  virtual void init(const YAML::Node& config);
  virtual void sendPacket(const Packet& msg);
  virtual ~DestinationPacketRos() = default;

private:

  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Publisher pkt_pub_;
  std::string frame_id_;
};

inline void DestinationPacketRos::init(const YAML::Node& config)
{
  yamlRead<std::string>(config["ros"], 
      "ros_frame_id", frame_id_, "rslidar");

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_packet_topic", 
      ros_send_topic, "rslidar_packets");

  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  pkt_pub_ = nh_->advertise<rslidar_msg::RslidarPacket>(ros_send_topic, 10);
}

inline void DestinationPacketRos::sendPacket(const Packet& msg)
{
  pkt_pub_.publish(toRosMsg(msg, frame_id_));
}

}  // namespace lidar
}  // namespace robosense

#endif  // ROS_FOUND

#ifdef ROS2_FOUND
#include "rslidar_msg/msg/rslidar_packet.hpp"
#include <rclcpp/rclcpp.hpp>

namespace robosense
{
namespace lidar
{

inline Packet toRsMsg(const rslidar_msg::msg::RslidarPacket& ros_msg)
{
  Packet rs_msg;
  rs_msg.timestamp = ros_msg.header.stamp.sec + double(ros_msg.header.stamp.nanosec) / 1e9;
  //rs_msg.seq = ros_msg.header.seq;
  rs_msg.is_difop = ros_msg.is_difop;
  rs_msg.is_frame_begin = ros_msg.is_frame_begin; 

  for (size_t i = 0; i < ros_msg.data.size(); i++)
  {
    rs_msg.buf_.emplace_back(ros_msg.data[i]);
  }

  return rs_msg;
}

class SourcePacketRos : public SourceDriver
{ 
public: 

  virtual void init(const YAML::Node& config);

  SourcePacketRos();

private:

  void putPacket(const rslidar_msg::msg::RslidarPacket& msg);

  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Subscription<rslidar_msg::msg::RslidarPacket>::SharedPtr pkt_sub_;
};

SourcePacketRos::SourcePacketRos()
  : SourceDriver(SourceType::MSG_FROM_ROS_PACKET)
{
}

void SourcePacketRos::init(const YAML::Node& config)
{
  SourceDriver::init(config);

  std::string ros_recv_topic;
  yamlRead<std::string>(config["ros"], "ros_recv_packet_topic", 
      ros_recv_topic, "rslidar_packets");

  node_ptr_.reset(new rclcpp::Node("rslidar_packets_adapter"));
  pkt_sub_ = node_ptr_->create_subscription<rslidar_msg::msg::RslidarPacket>(ros_recv_topic, 10, 
      std::bind(&SourcePacketRos::putPacket, this, std::placeholders::_1));
} 
void SourcePacketRos::putPacket(const rslidar_msg::msg::RslidarPacket& msg)
{
  driver_ptr_->decodePacket(toRsMsg(msg));
}

inline rslidar_msg::msg::RslidarPacket toRosMsg(const Packet& rs_msg, const std::string& frame_id)
{
  rslidar_msg::msg::RslidarPacket ros_msg;
  ros_msg.header.stamp.sec = (uint32_t)floor(rs_msg.timestamp);
  ros_msg.header.stamp.nanosec = (uint32_t)round((rs_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
  //ros_msg.header.seq = rs_msg.seq;
  ros_msg.header.frame_id = frame_id;
  ros_msg.is_difop = rs_msg.is_difop;
  ros_msg.is_frame_begin = rs_msg.is_frame_begin;

  for (size_t i = 0; i < rs_msg.buf_.size(); i++)
  {
    ros_msg.data.emplace_back(rs_msg.buf_[i]);
  }

  return ros_msg;
}

class DestinationPacketRos : public DestinationPacket
{
public:

  virtual void init(const YAML::Node& config);
  virtual void sendPacket(const Packet& msg);
  virtual ~DestinationPacketRos() = default;

private:

  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Publisher<rslidar_msg::msg::RslidarPacket>::SharedPtr pkt_pub_;
  std::string frame_id_;
};

inline void DestinationPacketRos::init(const YAML::Node& config)
{
  yamlRead<std::string>(config["ros"], 
      "ros_frame_id", frame_id_, "rslidar");

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_packet_topic", 
      ros_send_topic, "rslidar_packets");

  node_ptr_.reset(new rclcpp::Node("rslidar_packets_adapter"));
  pkt_pub_ = node_ptr_->create_publisher<rslidar_msg::msg::RslidarPacket>(ros_send_topic, 10);
}

inline void DestinationPacketRos::sendPacket(const Packet& msg)
{
  pkt_pub_->publish(toRosMsg(msg, frame_id_));
}

}  // namespace lidar
}  // namespace robosense

#endif  // ROS2_FOUND


