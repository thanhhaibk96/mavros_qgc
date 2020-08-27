#ifndef PARAMETER_PROTOCOL_H
#define PARAMETER_PROTOCOL_H

#include <mavconn/interface.h>
#include <ros/ros.h>

#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>

#include "mavlink.h"

using namespace mavconn;
using namespace mavlink;
using namespace mavlink::common;
using namespace mavlink::common::msg;
using namespace mavros_msgs;

class GCSTransceiver;

/**
 * @brief Manager for parameter microservice.
 *
 * See https://mavlink.io/en/services/parameter.html for more details.
 */
class ParameterProtocol
{
public:
  using StringList = std::list<std::string>;

  ParameterProtocol();
  ParameterProtocol(GCSTransceiver* trans, const MAVConnInterface::Ptr link);

  /**
   * @brief Handler for message reception from the parented GCSTransceiver.
   * @param[in] msg Message to handle.
   *
   * This handler serves to branch messages to proper sub-handlers.
   */
  void handleParameter(const mavlink_message_t* msg);

private:
  ros::ServiceClient reqSetHeadingPID;
  ros::ServiceClient reqGetHeadingPID;
  ros::ServiceClient reqSetLOSParams;
  ros::ServiceClient reqGetLOSParams;
  ros::ServiceClient reqSetLightParam;
  ros::ServiceClient reqGetLightParam;
  ros::ServiceClient reqGetComPassParam;
  ros::ServiceClient reqGetBatteryParam;
  ros::ServiceClient reqGetFailSafeParam;
  ros::ServiceClient reqGetAccelParam;
  ros::ServiceClient reqGetATCParam;
  ros::ServiceClient reqGetArmingParam;

  ros::ServiceClient reqGetDOITRONGParam;
  ros::ServiceClient reqSetDOITRONGParam;

  /**
   * @brief Link to handle the parameter microservice.
   */
  MAVConnInterface::Ptr link;

  /**
   * @brief Parented GCSTranceiver.
   *
   * This pointer is needed as the microservice needs to filter the autopilot sysid and compid, in some cases to update
   * the system heartbeat and status.
   */
  GCSTransceiver* trans;

  StringList headingPID;
  StringList LOSParams;
  StringList Light;
  StringList ComPass;
  StringList Serial;
  StringList Battery;
  StringList FailSafe;
  StringList FailSafeF;
  StringList FailSafe8;
  StringList Accel;
  StringList ATC;
  StringList ARMING;
  StringList ARMING_CHECK;
  StringList ParamInt16;
  StringList DOITRONGparam;
  /**
   * @brief Handler for message sequence for reading all parameters of the
   * system.
   * @param[in] msg Message to handle.
   *
   * By default, after startup, QGroundControl automatically requests reading all parameters from the connected
   * autopilot. If the autopilot does not respond appropriately to this request, many of the GCS's functionalities will
   * not work.
   *
   * This function is accomplished by calling ROS services that connect to suitable parameter lists. See
   * readParamList(..) for more details.
   */
  void readAllParameters(const mavlink_message_t* msg);
  void readParameter(const mavlink_message_t* msg);

  /**
   * @brief Handler for message sequence for writing a single parameter.
   * @param[in] msg Message to handle.
   */
  void writeParameter(const mavlink_message_t* msg);

  /**
   * @brief Handler for ROS services for writing a single parameter.
   * @param[in] srv Service message to request/response.
   * @return true if the targeted service properly responses.
   *
   * This handler serves to branch service messages to proper services aimed for specific parameter list.
   */
  bool requestSetParameter(ParamSet& srv);

  /**
   * @brief Handler for ROS services for reading a single parameter.
   * @param[in] srv Service message to request/response.
   * @param[in] paramList Parameter list that the requested parameter belongs
   * to.
   * @return true if the targeted service properly responses.
   *
   * This handler serves to branch service messages to proper services aimed for specific parameter list.
   */
  bool requestGetParameter(ParamGet& srv, const StringList& paramList);

  /**
   * @brief Handler for reading all members in a parameter list.
   * @param paramList Requested parameter list.
   * @param numParams Total number of parameters in the system.
   * @param paramId Current id of the requested parameter.
   */
  void readParamList(const StringList& paramList, const uint16_t& numParams, uint16_t& paramId);

  /**
   * @brief Helper function to check if a parameter belongs to a parameter list.
   * @param paramList The parameter list.
   * @param paramId The paramter id.
   * @return
   */
  inline bool paramListMatched(const StringList& paramList, const char* paramId)
  {
    for (auto it = paramList.begin(); it != paramList.end(); it++)
      if (!strncmp(paramId, it->data(), it->size()))
        return true;
    return false;
  }
};

#endif // PARAMETER_PROTOCOL_H
