#ifndef MISSION_PROTOCOL_H
#define MISSION_PROTOCOL_H

#include <mavconn/interface.h>
#include <ros/ros.h>

#include <mavros_msgs/WaypointList.h>

#include "mavlink.h"

using namespace mavconn;
using namespace mavlink;
using namespace mavlink::common;
using namespace mavlink::common::msg;
using namespace mavros_msgs;

class GCSTransceiver;

/**
 * @brief Manager for mission microservice.
 *
 * See https://mavlink.io/en/services/mission.html for more details.
 */
class MissionProtocol
{
public:
  MissionProtocol();
  MissionProtocol(GCSTransceiver* trans, const MAVConnInterface::Ptr link);

  /**
   * @brief Handler for message reception from the parented GCSTransceiver.
   * @param[in] msg Message to handle.
   *
   * This handler serves to branch messages to proper sub-handlers.
   */
  void handleMission(const mavlink_message_t* msg);

private:
  ros::Publisher pubItemList;

  /**
   * @brief Link to handle the mission microservice.
   */
  MAVConnInterface::Ptr link;

  /**
   * @brief Parented GCSTranceiver
   *
   * This pointer is needed as the microservice needs to filter the autopilot sysid and compid, in some cases to update
   * the system heartbeat and status.
   */
  GCSTransceiver* trans;

  WaypointList itemList;
  uint16_t itemCount;
  uint8_t missionType;

  /**
   * @brief Handler for message sequence for uploading mission to system.
   * @param[in] msg Message to handle.
   */
  void uploadMissionToVehicle(const mavlink_message_t* msg);
  void downloadMissionFromVehicle(const mavlink_message_t* msg);
  void setCurrentMissionItem(const mavlink_message_t* msg);
  void clearMission(const mavlink_message_t* msg);
};

#endif // MISSION_PROTOCOL_H
