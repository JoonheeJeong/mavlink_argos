/*
 * AUTHOR: Joonhee Jeong <jeonggoo75@gmail.com>
 *
 * An example controller communicating through mavlink for the eye-bot.
 *
 * This controller is meant to be used with the XML file:
 *    argos_worlds/mavlink_argos_2.argos
 */

#ifndef MAVLINK_ARGOS_2_H 
#define MAVLINK_ARGOS_2_H 

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the quadrotor positioning actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
/* Definition of the range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>

#include <ros/ros.h>
#include "argos_bridge/Position.h"


/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CMavlinkArgos2: public CCI_Controller {

public:

   /* Class constructor. */
   CMavlinkArgos2();

   /* Class destructor. */
   virtual ~CMavlinkArgos2() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML file
    * in the <controllers><eyebot_flocking_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Destroy() {}

private:

   /*
    * Takes off the robot.
    */
   void TakeOff();

   /*
    * Lets the robot perform flying.
    */
   void Fly();

   /*
    *  callback for setting up the given curent position to display.
    */
   void curPosCallback(const argos_bridge::Position::ConstPtr &p_curPos);

private:

   /* Current robot state */
   enum EState {
      STATE_START = 0,
      STATE_TAKE_OFF,
      STATE_FLY
   };

private:

	/* Pointer to the quadrotor position actuator */
	CCI_QuadRotorPositionActuator* m_pcPosAct;
	/* Pointer to the range-and-bearing actuator */
	CCI_RangeAndBearingActuator* m_pcRABAct;
	/* Pointer to the range-and-bearing sensor */
	CCI_RangeAndBearingSensor* m_pcRABSens;
	/* Pointer to the positioning sensor */
	CCI_PositioningSensor* m_pcPosSens;

	/* Current robot state */
	EState m_eState;

	/* Actual current position to display */
	CVector3 m_curPos;

	bool m_signal = true;

	ros::Subscriber m_curPosSub;
	ros::Publisher m_signalPub;

public:
  // We need only a single ROS node, although there are individual publishers
  // and subscribers for each instance of the class.
	static ros::NodeHandle* nodeHandle;
};

#endif
