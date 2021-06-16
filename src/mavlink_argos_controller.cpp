#include <ros/ros.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/mavlink_convert.h>

#ifndef MAVLINK_H
    typedef mavlink::mavlink_message_t mavlink_message_t;
#endif
#include <mavlink/v2.0/mavlink_argos/mavlink.h>

#include <argos3/core/utility/math/vector2.h>

#include <iostream>
#include <sstream>
#include <string>
#include "argos_bridge/Position.h"

// generating random real numbers for target position
#include <random>
#include <limits>
#include <cmath>

// timestamp
#include <string>
#include <time.h>

#define ARENA_SIZE 30
#define BOTS 5
#define COMM_THRESHOLD 3.f

/* for generating random values */
const float lower = std::nextafter(-ARENA_SIZE/2, std::numeric_limits<float>::max());
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis(lower, ARENA_SIZE/2); 
/********************************/ 

argos_bridge::Position *gp_curPos = nullptr;

void curPosCallback(const argos_bridge::Position::ConstPtr &curPos);
void rmsgCallback(const mavros_msgs::Mavlink::ConstPtr &rmsg);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mavlink_argos_controller");
    ros::NodeHandle nh;
    const std::string name = ros::this_node::getNamespace();

    std::stringstream tgtPosTopic;
    tgtPosTopic << name << "/target_pos";
    ros::Publisher tgtPosPub = nh.advertise<argos_bridge::Position>(tgtPosTopic.str(), 1);

    std::stringstream rmsgTopic("/rmsg");
    ros::Publisher rmsgPub = nh.advertise<mavros_msgs::Mavlink>(rmsgTopic.str(), 1);
    ros::Subscriber rmsgSub = nh.subscribe(rmsgTopic.str(), BOTS * 1, rmsgCallback);

    std::stringstream curPosTopic;
    curPosTopic << name << "/current_pos";
    ros::Subscriber curPosSub = nh.subscribe(curPosTopic.str(), 1, curPosCallback);

    /*
       1. publish target position one time per one second
       2. subscribe current position, rmsg ten times per one second
    */
    mavlink::mavlink_message_t mmsg;
    mavros_msgs::Mavlink rmsg;
    uint8_t system_id = (uint8_t) std::stoi(name.substr(7));
    uint8_t component_id = 200;
    uint32_t id = system_id;

    ros::Rate loop_late(10); // 10 Hz
    int count = 0;
    static bool gp_curPos_init = false;
    while (ros::ok()) {
        // At first, ros controllers need to get a signal from argos simulator.
        if (gp_curPos == nullptr) {
            ros::spinOnce();
            loop_late.sleep();
            continue;
        }

        /* gp init check
        if (!gp_curPos_init) {
            std::stringstream ss;
            ss<<""<<name.substr(1)<<"'s current position: {x:"<<gp_curPos->x<<",y:"<<gp_curPos->y<<",z:"<<gp_curPos->z<<"}";
            ROS_INFO("%s", ss.str().c_str());
            gp_curPos_init = true;
        }
        */

        if (++count == 30) {
            argos_bridge::Position tgtPos;
            tgtPos.x = dis(gen);
            tgtPos.y = dis(gen);
            // coordinate of z-axis is fixed
            // tgtPos.z = 3.f;
            tgtPosPub.publish(tgtPos);
            // ROS_INFO("tgtPosPub: %s", name.substr(1).c_str());
            count = 0;
        }

        /* mavlink <> ros */
        
        uint64_t timestamp = (uint64_t) std::time(NULL);
        /*
        float estimate = dis(gen);
        uint8_t belief = (estimate >= 0.5) ? 1 : 0;
        mavlink_msg_heartbeat_pack(system_id, component_id, &mmsg, 
               MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE);
        */

        mavlink_msg_agent_info_pack(system_id, component_id, &mmsg, 
                id, timestamp, gp_curPos->x, gp_curPos->y, 3.f);
        mavros_msgs::mavlink::convert(mmsg, rmsg);
        rmsgPub.publish(rmsg);
        ros::spinOnce();
        loop_late.sleep();
    }

    return 0;
}

void curPosCallback(const argos_bridge::Position::ConstPtr &p_curPos)
{
//    ROS_INFO("curPosCallback: %s", ros::this_node::getNamespace().substr(1).c_str());
    if (gp_curPos == nullptr)
        gp_curPos = new argos_bridge::Position();
    gp_curPos->x = p_curPos->x;
    gp_curPos->y = p_curPos->y;
    gp_curPos->z = p_curPos->z;
    // std::string name = ros::this_node::getNamespace();
    // ROS_INFO("[%s] position : %f %f %f", name.c_str(), gp_curPos->x, gp_curPos->y, gp_curPos->z);
}

void rmsgCallback(const mavros_msgs::Mavlink::ConstPtr &p_rmsg)
{
    if (gp_curPos == nullptr)
        return;
    mavlink_message_t mmsg;
    mavros_msgs::mavlink::convert(*p_rmsg, mmsg);

    const std::string name = ros::this_node::getNamespace().substr(1);
    int sysid = (int) mmsg.sysid; // (uint8_t -> int)
    if (sysid == std::stoi(name.substr(6)))
        return;

    uint64_t timestamp = mavlink_msg_agent_info_get_timestamp(&mmsg);
    float x = mavlink_msg_agent_info_get_x(&mmsg);
    float y = mavlink_msg_agent_info_get_y(&mmsg);
    argos::CVector2 src_pos(x, y);
    argos::CVector2 dst_pos(gp_curPos->x, gp_curPos->y);
    const float dist = argos::Distance(src_pos, dst_pos);
    // std::cout << name << " dist: " << dist << std::endl;
    
    if (dist > COMM_THRESHOLD) {
        // ss << "DISCARD! (distance=" << dist << ">" << COMM_THRESHOLD << ")";
        return;
    } else {
        std::stringstream ss;
        ss << "[" << name << "] got eyebot" << sysid << "'s rmsg (at:" << timestamp << ")\n"
            << "  => ACCEPT! (distance=" << dist << "<=" << COMM_THRESHOLD << ")";
        ROS_INFO("%s", ss.str().c_str());
    }
}

