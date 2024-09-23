#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>



int main(int argc, char *argv[]){


    ros::init(argc, argv, "local_map_tf_pubulisher");
    ros::NodeHandle n;

    ros::Rate loop_rate(50);


    tf2_ros::Buffer tfBuffer;

    tf2_ros::TransformListener tfListener(tfBuffer);

    tf2_ros::TransformBroadcaster tfBroadcaster;

    geometry_msgs::TransformStamped transformStampedRec;

    geometry_msgs::TransformStamped transformStampedPub;


    while (ros::ok()){

        try {
            transformStampedRec = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        transformStampedPub.header.stamp = transformStampedRec.header.stamp;
        transformStampedPub.header.frame_id = "map";
        transformStampedPub.child_frame_id = "local_map";
        transformStampedPub.transform.translation.z = transformStampedRec.transform.translation.z;
        transformStampedPub.transform.rotation.w = 1;

        tfBroadcaster.sendTransform(transformStampedPub);
        // ros::spinOnce();

        loop_rate.sleep(); 
    }
    return 0;
}
