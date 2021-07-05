#include <ros/ros.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/mavlink_convert.h>

#ifndef MAVLINK_H
    typedef mavlink::mavlink_message_t mavlink_message_t;
#endif
#include <mavlink/v2.0/mavlink_argos/mavlink.h>

#include <argos3/core/utility/math/vector3.h>

#include <iostream>
#include <sstream>
#include <string>
#include "argos_bridge/Position.h"

// generating random real numbers for target position
#include <random>
#include <limits>
#include <cmath>

// timestamp
#include <ctime>

// handling with argos init signal
#include <std_msgs/Empty.h>
#include <fstream>

#define ARENA_SIZE 30
#define BOTS 5
#define MOVING_THRESHOLD 0.1f
#define COMM_THRESHOLD 3.f
#define FIXED_HEIGHT 3.f

/* for generating random values */
// const float lower = std::nextafter(-ARENA_SIZE/2, std::numeric_limits<float>::max());
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis(-ARENA_SIZE/2, ARENA_SIZE/2); 
/********************************/ 

argos::CVector3 g_v3_curPos;
bool g_signal = false;

void setInitPos(int id);
void setLimit(int &limit, float &remRatio, argos::CVector3 &v3_tgtPos, argos::CVector3 &v3_posDiff);

void rmsgCallback(const mavros_msgs::Mavlink::ConstPtr &rmsg);
void argosSignalCallback(const std_msgs::Empty::ConstPtr &signal) { g_signal = true; }

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mavlink_argos_2_controller");
    ros::NodeHandle nh;
    const std::string name = ros::this_node::getNamespace();

    std::stringstream curPosTopic;
    curPosTopic << name << "/current_pos";
    ros::Publisher curPosPub = nh.advertise<argos_bridge::Position>(curPosTopic.str(), 1);

    std::stringstream rmsgTopic("/rmsg");
    ros::Publisher rmsgPub = nh.advertise<mavros_msgs::Mavlink>(rmsgTopic.str(), 1);
    ros::Subscriber rmsgSub = nh.subscribe(rmsgTopic.str(), BOTS * 1, rmsgCallback);

	std::stringstream argosSignalTopic;
	argosSignalTopic << name << "/signal";
    ros::Subscriber argosSignalSub = nh.subscribe(argosSignalTopic.str(), 1, argosSignalCallback);

    /*
		- publish & subscribe current position, rmsg 100 times per a second
			(generate virtual target position 1 time per 3 seconds)
		- publish rmsg 1 time per a second
    */
    mavlink::mavlink_message_t mmsg;
    mavros_msgs::Mavlink rmsg;
    uint8_t system_id = (uint8_t) std::stoi(name.substr(8)); // /eyebot_?
    uint8_t component_id = 200;
    uint32_t id = system_id;

	setInitPos(id);
	ROS_INFO("[%s] position : %f %f %f", name.c_str(), g_v3_curPos[0], g_v3_curPos[1], g_v3_curPos[2]);

    // At first, ros controllers need to get a signal from argos simulator.
    ros::Rate signal_late(10); // 10 Hz
    while (ros::ok() && !g_signal) {
        ros::spinOnce();
        signal_late.sleep();
	}
	g_signal = false;

	// take-off
	ROS_INFO("[%s] start to take-off", name.c_str());
    while (ros::ok() && !g_signal) {
        ros::spinOnce();
        signal_late.sleep();
    }
	g_signal = false;
	g_v3_curPos[2] = FIXED_HEIGHT;

	// fly (height is fixed to 3.0)
	int loops=1, count= 0, limit;
	float remRatio;
    argos_bridge::Position ab_curPos;
	argos::CVector3 v3_tgtPos, v3_posDiff, v3_unitDiff;
	ab_curPos.z = v3_tgtPos[2] = FIXED_HEIGHT; // coordinate of z-axis is fixed
    ros::Rate loop_late(100); // 100 Hz
	ROS_INFO("[%s] start to fly", name.c_str());
	while (ros::ok()) {
        if (++count == 1) {
			v3_tgtPos[0] = dis(gen);
			v3_tgtPos[1] = dis(gen);
			setLimit(limit, remRatio, v3_tgtPos, v3_posDiff);
			v3_unitDiff = v3_posDiff / limit;
			// ROS_INFO("[%s]limit(%d): %d", name.c_str(), loops++, limit);
        }

		if (count <= limit) {
			ab_curPos.x = g_v3_curPos[0] += v3_unitDiff[0];
			ab_curPos.y = g_v3_curPos[1] += v3_unitDiff[1];
			curPosPub.publish(ab_curPos);
		} 
		else if (count == limit + 1) {
			ab_curPos.x = g_v3_curPos[0] += v3_unitDiff[0] * remRatio;
			ab_curPos.y = g_v3_curPos[1] += v3_unitDiff[1] * remRatio;
			curPosPub.publish(ab_curPos);
		}

        // publish mavlink <> ros rmsg communication, 1 time/s
		if (count % 100 == 0) {
			uint64_t timestamp = (uint64_t) std::time(NULL);
			// float estimate = dis(gen);
			// uint8_t belief = (estimate >= 0.5) ? 1 : 0;
			mavlink_msg_agent_info_pack(system_id, component_id, &mmsg, 
					id, timestamp, ab_curPos.x, ab_curPos.y, ab_curPos.z);
			mavros_msgs::mavlink::convert(mmsg, rmsg);
			rmsgPub.publish(rmsg);
		}

		// generate virtual target position, 1 time/3s
		if (count == 300) {
			count = 0;
		}
        ros::spinOnce();
        loop_late.sleep();
    }
    return 0;
}

void setInitPos(int id)
{
	std::string init_pos_file = "/tmp/mavlink_argos_2_init_position.txt"; 
	std::ifstream ifs(init_pos_file);
	std::string line;
	float tempCurPos[3];
	for (int n_line = 0; ifs >> line; n_line++) {
		if (n_line != id)
			continue;
		std:: cout << line << std::endl;
		auto p = line.find('=');
		line = line.substr(p+1);
		decltype(p) q = -1;
		for (int i = 0; i < 3; ++i) {
			p = q+1;
			q = line.find(',', p);
			tempCurPos[i] = std::stof(line.substr(p, q-p));
		}
		g_v3_curPos[0] = tempCurPos[0];
		g_v3_curPos[1] = tempCurPos[1];
		g_v3_curPos[2] = tempCurPos[2];
		// std::cout << id << "'s position : [x=" << a_cPos[0] 
		//	<< ",y=" << a_cPos[1] << ",z=" << a_cPos[2] << "]" << std::endl;
		return;
	}
}

void setLimit(int &limit, float &remRatio, argos::CVector3 &v3_tgtPos, argos::CVector3 &v3_posDiff)
{
	v3_posDiff = v3_tgtPos - g_v3_curPos;
	float dist = v3_posDiff.Length();
	limit = dist / MOVING_THRESHOLD;
	remRatio = (dist - limit * MOVING_THRESHOLD) / MOVING_THRESHOLD;
}

void rmsgCallback(const mavros_msgs::Mavlink::ConstPtr &p_rmsg)
{
    mavlink_message_t mmsg;
    mavros_msgs::mavlink::convert(*p_rmsg, mmsg);

    const std::string name = ros::this_node::getNamespace().substr(1); // /eyebot_?
    int sysid = (int) mmsg.sysid; // (uint8_t -> int)
    if (sysid == std::stoi(name.substr(7))) // eyebot_?
        return;

    uint64_t timestamp = mavlink_msg_agent_info_get_timestamp(&mmsg);
    float x = mavlink_msg_agent_info_get_x(&mmsg);
    float y = mavlink_msg_agent_info_get_y(&mmsg);
    float z = mavlink_msg_agent_info_get_z(&mmsg);
    argos::CVector3 src_pos(x, y, z);
    float dist = argos::Distance(src_pos, g_v3_curPos);
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
