#include <ros/ros.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/mavlink_convert.h>

#ifndef MAVLINK_H
    typedef mavlink::mavlink_message_t mavlink_message_t;
#   include <mavlink/v2.0/mavlink_argos/mavlink.h>
#endif
#include <mavlink/v2.0/mavlink_argos/mavlink.h>

#include <sstream>
#include <string>
#include "argos_bridge/Position.h"

void rmsgCallback(const mavros_msgs::Mavlink::ConstPtr &rmsg);
void positionCallback(const argos_bridge::Position::ConstPtr &position);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mavlink_argos_controller");
    ros::NodeHandle nh;

    ros::Subscriber positionSub = nh.subscribe("position", 1, positionCallback);
    ros::Subscriber rmsgSub = nh.subscribe("/rmsg", 1, rmsgCallback);
    ros::spin();

    return 0;
}

void rmsgCallback(const mavros_msgs::Mavlink::ConstPtr &p_rmsg)
{
    /*
    mavlink::mavlink_message_t mmsg;
    mavros_msgs::mavlink::convert(rmsg, mmsg);
    uint8_t system_id = mmsg.sysid; 
    uint32_t id = mavlink_msg_agent_info_get_id(&mmsg);;
    uint64_t timestamp = mavlink_msg_agent_info_get_timestamp(&mmsg);
    float x = mavlink_msg_agent_info_get_x(&mmsg);
    float y = mavlink_msg_agent_info_get_y(&mmsg);
    float z = mavlink_msg_agent_info_get_z(&mmsg);
    CVector3 src_pos(x, y, z);
    CVector3 dst_pos = m_pcPosSens->GetReading().Position;
    Real dist = Distance(src_pos, dst_pos);

    std::stringstream ss;
    ss << "rmsg callback: bot" << id << ">" << ros::this_node::getNamespace() << ": " << std::to_string(dist);
    ROS_INFO("%s", ss.str().c_str());
    */
    std::string name = ros::this_node::getNamespace();
    ROS_INFO("[%s] eyebot[%d]'s rmsg", name.c_str(), p_rmsg->sysid);
}

void positionCallback(const argos_bridge::Position::ConstPtr &p_pos)
{
    std::string name = ros::this_node::getNamespace();
    ROS_INFO("[%s] position : %f %f %f", name.c_str(), p_pos->x, p_pos->y, p_pos->z);
}
