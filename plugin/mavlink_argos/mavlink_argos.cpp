/* Include the controller definition */
#include "mavlink_argos.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Function definitions for logging */
#include <argos3/core/utility/logging/argos_log.h>

#include <sstream>

// timestamp
#include <string>
#include <time.h>

// for generating random real numbers
#include <random>
#include <limits>
#include <cmath>



/****************************************/
/****************************************/

ros::NodeHandle* initROS() {
  int argc = 0;
  char *argv = (char *) "";
  ros::init(argc, &argv, "mavlink_argos");
  return new ros::NodeHandle();
}

ros::NodeHandle* CMavlinkArgos::m_nodeHandle = initROS();

// generating random real number
std::random_device rd;
std::mt19937 gen(rd());
float upper = std::nextafter(1, std::numeric_limits<float>::max());
std::uniform_real_distribution<> dis(0, upper); 

/* Tolerance for the distance to a target point to decide to do something else */
static const Real POSITIONING_TOLERANCE = 0.01f;

/****************************************/
/****************************************/

void CMavlinkArgos::SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "target_distance", TargetDistance);
      GetNodeAttribute(t_node, "gain", Gain);
      GetNodeAttribute(t_node, "exponent", Exponent);
      GetNodeAttribute(t_node, "max_interaction", MaxInteraction);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
   }
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential
 */
Real CMavlinkArgos::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

CMavlinkArgos::CMavlinkArgos() :
   m_pcPosAct(NULL),
   m_pcRABAct(NULL),
   m_pcRABSens(NULL),
   m_pcLightSens(NULL) {}

/****************************************/
/****************************************/

void CMavlinkArgos::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "quadrotor_positioning") corresponds to the XML tag of the
    * device whose handle we want to have. For a list of allowed values, type at the
    * command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors internally, on the basis of
    *       the lists provided the configuration file at the
    *       <controllers><eyebot_diffusion><actuators> and
    *       <controllers><eyebot_diffusion><sensors> sections. If you forgot to
    *       list a device in the XML and then you request it here, an error occurs.
    */
   m_pcPosAct    = GetActuator<CCI_QuadRotorPositionActuator>("quadrotor_position");
   m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator  >("range_and_bearing" );
   m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor    >("range_and_bearing" );
   m_pcLightSens = GetSensor  <CCI_EyeBotLightSensor        >("eyebot_light"      );
   m_pcPosSens   = GetSensor  <CCI_PositioningSensor        >("positioning"       );
   /*
    * Parse the config file
    */
   try {
      /* Flocking-related */
      m_sFlockingParams.Init(GetNode(t_node, "flocking"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
   }
   /* Perform further initialization */
   std::stringstream rmsgTopic;
   rmsgTopic << "/" << GetId() << "/rmsg";
   m_rmsgPub = m_nodeHandle->advertise<mavros_msgs::Mavlink>(rmsgTopic.str(), 1);
   m_rmsgSub = m_nodeHandle->subscribe(rmsgTopic.str(), 1, &CMavlinkArgos::rmsgCallback, this);
   Reset();
}

/****************************************/
/****************************************/

void CMavlinkArgos::Reset() {
   /* Switch to state start */
   m_eState = STATE_START;
   /* Tell robots around that this robot is starting */
   m_pcRABAct->SetData(0, STATE_START);
}

/****************************************/
/****************************************/

void CMavlinkArgos::ControlStep() {
   switch(m_eState) {
      case STATE_START:
         TakeOff();
         break;
      case STATE_TAKE_OFF:
         TakeOff();
         break;
      case STATE_FLOCK:
         Flock();
         break;
      default:
         LOGERR << "[BUG] Unknown robot state: " << m_eState << std::endl;
   }
}

/****************************************/
/****************************************/

void CMavlinkArgos::TakeOff() {
   if(m_eState != STATE_TAKE_OFF) {
      /* Switch to state take off */
      m_eState = STATE_TAKE_OFF;
      /* Set the target position on the vertical of the current position */
      m_cTargetPos = m_pcPosSens->GetReading().Position;
      m_cTargetPos.SetZ(3.0f);
      m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
      /* Tell robots around that this eye-bot is taking off */
      m_pcRABAct->SetData(0, STATE_TAKE_OFF);
   }
   if(Distance(m_cTargetPos, m_pcPosSens->GetReading().Position) < POSITIONING_TOLERANCE) {
      /* State transition */
      Flock();
   }
}

/****************************************/
/****************************************/

void CMavlinkArgos::Flock() {
   if(m_eState != STATE_FLOCK) {
      /* Switch to state flock */
      m_eState = STATE_FLOCK;
      /* Tell robots around that this robot is ready to flock */
      m_pcRABAct->SetData(0, STATE_FLOCK);
   }
   CVector2 cDirection = VectorToLight() + FlockingVector();
   m_pcPosAct->SetRelativePosition(
      CVector3(cDirection.GetX(),
               cDirection.GetY(),
               0.0f));

   // mavlink <> ros
   mavlink_message_t mmsg;
   mavros_msgs::Mavlink rmsg;
   uint8_t system_id = 1;
   uint8_t component_id = 200;
   uint32_t id = stoi(GetId().substr(2));
   uint64_t timestamp = (uint64_t) time(NULL);
   float estimate = dis(gen);
   uint8_t belief = (estimate >= 0.5) ? 1 : 0;
   mavlink_msg_agent_info_pack(system_id, component_id, &mmsg, 
           id, timestamp, estimate, belief);
   mavros_msgs::mavlink::convert(mmsg, rmsg);
   m_rmsgPub.publish(rmsg);
}

/****************************************/
/****************************************/

CVector2 CMavlinkArgos::VectorToLight() {
   /* Get light readings */
   const CCI_EyeBotLightSensor::TReadings& tReadings = m_pcLightSens->GetReadings();
   /* Calculate a normalized vector that points to the closest light */
   CVector2 cAccum;
   for(size_t i = 0; i < tReadings.size(); ++i) {
      cAccum += CVector2(tReadings[i].Value, tReadings[i].Angle);
   }
   if(cAccum.Length() > 0.0f) {
      /* Make the vector long as 1/10 of the max speed */
      cAccum.Normalize();
      cAccum *= 0.1f * m_sFlockingParams.MaxInteraction;
   }
   return cAccum;
}

/****************************************/
/****************************************/

CVector2 CMavlinkArgos::FlockingVector() {
   /* Get RAB messages from nearby eye-bots */
   const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
   /* Go through them to calculate the flocking interaction vector */
   if(! tMsgs.empty()) {
      /* This will contain the final interaction vector */
      CVector2 cAccum;
      /* Used to calculate the vector length of each neighbor's contribution */
      Real fLJ;
      /* A counter for the neighbors in state flock */
      UInt32 unPeers = 0;
      for(size_t i = 0; i < tMsgs.size(); ++i) {
         /*
          * We consider only the neighbors in state flock
          */
         if(tMsgs[i].Data[0] == STATE_FLOCK) {
            /*
             * Take the message sender range and horizontal bearing
             * With the range, calculate the Lennard-Jones interaction force
             * Form a 2D vector with the interaction force and the bearing
             * Sum such vector to the accumulator
             */
            /* Calculate LJ */
            fLJ = m_sFlockingParams.GeneralizedLennardJones(tMsgs[i].Range);
            /* Sum to accumulator */
            cAccum += CVector2(fLJ,
                               tMsgs[i].HorizontalBearing);
            /* Count one more flocking neighbor */
            ++unPeers;
         }
      }
      if(unPeers > 0) {
         /* Divide the accumulator by the number of flocking neighbors */
         cAccum /= unPeers;
         /* Limit the interaction force */
         if(cAccum.Length() > m_sFlockingParams.MaxInteraction) {
            cAccum.Normalize();
            cAccum *= m_sFlockingParams.MaxInteraction;
         }
      }
      /* All done */
      return cAccum;
   }
   else {
      /* No messages received, no interaction */
      return CVector2();
   }
}

/****************************************/
/****************************************/

void CMavlinkArgos::rmsgCallback(const mavros_msgs::Mavlink &rmsg)
{
    ROS_INFO("rmsg callback");
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CMavlinkArgos, "mavlink_argos_controller")
