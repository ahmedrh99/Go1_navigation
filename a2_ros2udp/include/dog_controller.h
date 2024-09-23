
#include "unitree_legged_sdk_to_ros.h"

/**
 * Custom
 */
class DogController : public UnitreeLeggedSDKToROS
{
public:
  std::string patrol_points_file_path;
  int last_set_cmd = 0;
  float last_point[3] = {0};

  ros::Publisher dog_can_move_pub;

  bool is_build_map = false;
  float yaw = 0;

  std_msgs::Bool dog_can_move;

public:
  DogController(ros::NodeHandle nh,
                float period, int control_level, LeggedType leg_type,
                uint16_t local_port, const char *target_ip, uint16_t target_port,
                int vel_cmd_mode)
      : UnitreeLeggedSDKToROS(nh, period, control_level, leg_type, local_port, target_ip, target_port, vel_cmd_mode)
  {

    is_build_map = false;
    ros::param::get("dog_control_node/is_build_map", is_build_map);

    patrol_points_file_path = "";
    if (is_build_map)
    {
      ros::param::get("dog_control_node/patrol_points_file", patrol_points_file_path);
    }

    last_set_cmd = 0;
    last_point[3] = {0};

    dog_can_move_pub = nh_.advertise<std_msgs::Bool>("/dog_can_move", 1000);

    std::ofstream patrol_points_file(patrol_points_file_path, std::ios::trunc);
    patrol_points_file.close();
  }

  /**
   * @return true if there is valid command that need to send.
   */
  bool savePatrolPoints()
  {

    ros::Time time_cur = ros::Time::now();
    double stamp_cur = time_cur.toSec();

    dog_can_move.data = true;
    dog_can_move_pub.publish(dog_can_move);

    if (is_build_map)
    {
      tf::StampedTransform transformStamped;

      if (state_.wirelessRemote[3] - last_set_cmd == 4)
      {

        try
        {
          listener_.lookupTransform("map", "base_link", ros::Time(0), transformStamped);
        }
        catch (tf::TransformException &ex)
        {
          ROS_WARN("%s", ex.what());
          ros::Duration(1.0).sleep();
          return false;
        }

        yaw = tf::getYaw(transformStamped.getRotation());

        if (std::fabs(transformStamped.getOrigin().x() - last_point[0]) <= 0.05 &&
            std::fabs(transformStamped.getOrigin().y() - last_point[1]) <= 0.05 &&
            std::fabs(yaw - last_point[2]) <= 0.1)
        {
          std::cout << "Same points, not saved!" << std::endl;
        }
        else
        {
          std::ofstream patrol_points_file(patrol_points_file_path, std::ios::app);
          if (patrol_points_file.is_open())
          {
            patrol_points_file << transformStamped.getOrigin().x()
                               << " "
                               << transformStamped.getOrigin().y()
                               << " "
                               << yaw
                               << " "
                               << 3 << std::endl;
            std::cout << "Save a patrol point!" << std::endl;
            std::cout << transformStamped.getOrigin().x()
                      << " "
                      << transformStamped.getOrigin().y()
                      << " "
                      << yaw
                      << " "
                      << 3 << std::endl;

            last_point[0] = transformStamped.getOrigin().x();
            last_point[1] = transformStamped.getOrigin().y();
            last_point[2] = yaw;

            patrol_points_file.close();
          }
        }
      }
    }

    last_set_cmd = state_.wirelessRemote[3];
  }
};

void MySigintHandler(int sig)
{
  ROS_INFO("shutting down!");
  ros::shutdown();
  exit(0);
}


