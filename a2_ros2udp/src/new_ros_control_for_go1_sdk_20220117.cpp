
#include "dog_controller.h"

const float AX_CONTROL_PERIOD = 0.002;
const int AX_CONTROL_LEVEL = HIGHLEVEL;
const LeggedType AX_LEG_TYPE = LeggedType::A1;

const uint16_t AX_LOCAL_PORT = 8095;
const char *AX_TARGET_IP = "192.168.123.161";
const uint16_t AX_TARGET_PORT = 8082;
const uint8_t AX_VEL_CMD_MODE = 2;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ros_control_node");
  ros::NodeHandle nh("~");

  signal(SIGINT, MySigintHandler);

  ROS_INFO("[ROS_CONTROL] Node ros2udp has started.");
  std::cout << "[ROS_CONTROL] Control level is set to HIGH-level." << std::endl;
  std::cout << "[ROS_CONTROL] Make sure the robot is standing on the ground." << std::endl;

  DogController dog_controller(nh, AX_CONTROL_PERIOD, AX_CONTROL_LEVEL, AX_LEG_TYPE,
                AX_LOCAL_PORT, AX_TARGET_IP, AX_TARGET_PORT,
                AX_VEL_CMD_MODE);

  // Run function Custom::UDPSend() and Custom::UDPRecv in distinct threads
  // LoopFunc loop_udpSend("udp_send", custom.dt_, 3, boost::bind(&Custom::UDPSend, &custom));
  // LoopFunc loop_udpRecv("udp_recv", custom.dt_, 3, boost::bind(&Custom::UDPRecv, &custom));
   //loop_udpSend.start();
  // loop_udpRecv.start();

  // ROS Spin
  ros::Rate loop_rate(1.0 / AX_CONTROL_PERIOD);
  while (nh.ok())
  {
    ros::spinOnce();
    dog_controller.UDPRecv();
    dog_controller.UDPSend();
    dog_controller.savePatrolPoints();
    loop_rate.sleep();
  }

  return 0;
}
