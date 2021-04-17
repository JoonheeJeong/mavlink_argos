#include <ros/ros.h>
#include <mavros_msgs/Mavlink.h>
#include <mavlink/v2.0/mavlink_argos/mavlink.h>
#include <sstream>

void rmsg_callback(const mavros_msgs::Mavlink &rmsg)
{
    ROS_INFO("rmsg sub");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mavlink_argos_controller");
    ros::NodeHandle node_handle;

    ros::Publisher rmsg_pub = node_handle.advertise<mavros_msgs::Mavlink>("rmsg", 1000);
    ros::Subscriber rmsg_sub = node_handle.subscribe("rmsg", 1000, rmsg_callback);
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {
        mavros_msgs::Mavlink rmsg;
        // std::stringstream ss;
        // ss << count << ": ";
        // 
        ROS_INFO("rmsg pub");
        rmsg_pub.publish(rmsg);

        ros::spin();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
