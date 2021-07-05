/* Include the controller definition */
#include "mavlink_argos_2.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Function definitions for logging */
#include <argos3/core/utility/logging/argos_log.h>

#include <iostream>
#include <sstream>

#include <ros/callback_queue.h>
#include <std_msgs/Empty.h>

#define BOTS 5

/****************************************/
/****************************************/

ros::NodeHandle* initROS() {
	int argc = 0;
	char *argv = (char *) "";
	ros::init(argc, &argv, "mavlink_argos_2");
	return new ros::NodeHandle();
}

ros::NodeHandle* CMavlinkArgos2::nodeHandle = initROS();

/* Tolerance for the distance to a target point to decide to do something else */
static const Real POSITIONING_TOLERANCE = 0.01f;

/****************************************/
/****************************************/

CMavlinkArgos2::CMavlinkArgos2() :
	m_pcPosAct(NULL),
	m_pcRABAct(NULL),
	m_pcRABSens(NULL),
	m_pcPosSens(NULL) {}

/****************************************/
/****************************************/

void CMavlinkArgos2::Init(TConfigurationNode& t_node) {
	/* Get sensor/actuator handles */
	m_pcPosAct    = GetActuator<CCI_QuadRotorPositionActuator>("quadrotor_position");
	m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator  >("range_and_bearing" );
	m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor    >("range_and_bearing" );
	m_pcPosSens   = GetSensor  <CCI_PositioningSensor        >("positioning"       );

	/* Perform further initialization */
    std::stringstream curPosTopic;
    curPosTopic << "/" << GetId() << "/current_pos";
    m_curPosSub = nodeHandle->subscribe(curPosTopic.str(), 1, &CMavlinkArgos2::curPosCallback, this);

	std::stringstream signalTopic;
    signalTopic << "/" << GetId() << "/signal";
	m_signalPub = nodeHandle->advertise<std_msgs::Empty>(signalTopic.str(), 1);

    Reset();
}

/****************************************/
/****************************************/

void CMavlinkArgos2::Reset() {
	/* Switch to state start */
	m_eState = STATE_START;
	/* Tell robots around that this robot is starting */
	m_pcRABAct->SetData(0, STATE_START);
}

/****************************************/
/****************************************/

void CMavlinkArgos2::ControlStep() {
	if (m_signal == true) {
		std_msgs::Empty msg;
		m_signalPub.publish(msg);
		m_signal = false;
	}
	switch(m_eState) {
		case STATE_START:
			TakeOff();
			break;
		case STATE_TAKE_OFF:
			TakeOff();
			break;
		case STATE_FLY:
			Fly();
			break;
		default:
			LOGERR << "[BUG] Unknown robot state: " << m_eState << std::endl;
   }
}

/****************************************/
/****************************************/

void CMavlinkArgos2::TakeOff() {
	if(m_eState != STATE_TAKE_OFF) {
		/* Switch to state take off */
		m_eState = STATE_TAKE_OFF;
		/* Set the target position on the vertical of the current position */
		m_curPos = m_pcPosSens->GetReading().Position;
		m_curPos.SetZ(3.0f);
		m_pcPosAct->SetAbsolutePosition(m_curPos);
		/* Tell robots around that this eye-bot is taking off */
		m_pcRABAct->SetData(0, STATE_TAKE_OFF);
	}
	if(Distance(m_curPos, m_pcPosSens->GetReading().Position) < POSITIONING_TOLERANCE) {
		std_msgs::Empty msg;
		m_signalPub.publish(msg);
		/* State transition */
		Fly();
	}
}

/****************************************/
/****************************************/

void CMavlinkArgos2::Fly() {
	if(m_eState != STATE_FLY) {
		/* Switch to state flock */
		m_eState = STATE_FLY;
		/* Tell robots around that this robot is ready to flock */
		m_pcRABAct->SetData(0, STATE_FLY);
	}
	ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));
}

/****************************************/
/****************************************/

void CMavlinkArgos2::curPosCallback(const argos_bridge::Position::ConstPtr &p_curPos)
{
    float x = p_curPos->x;
    float y = p_curPos->y;
    std::cout << "curPosCallback: " << GetId() << " [x="<< x <<",y="<< y << "]" << std::endl;
    m_pcPosAct->SetAbsolutePosition(
        CVector3(
            x,
            y,
            // coordinate of z-axis is fixed
            3.f));
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CMavlinkArgos2, "mavlink_argos_2_controller")
