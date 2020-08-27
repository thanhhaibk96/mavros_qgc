#ifndef COMMAND_PROTOCOL_H
#define COMMAND_PROTOCOL_H

#include <mavconn/interface.h>
#include <ros/ros.h>

#include <mavros_msgs/CommandInt.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>

#include "mavlink.h"

using namespace mavconn;
using namespace mavlink;
using namespace mavlink::common;
using namespace mavlink::common::msg;
using namespace mavros_msgs;

class GCSTransceiver;

/**
 * @brief Manager for command microservice.
 *
 * According to official document, the command microservice only handles COMMAND_LONG and COMMAND_INT message types.
 * However, some other types such as SET_MODE behave as if they belong to command microservice. So to fully implement
 * the protocol, all these additional types are taken into account.
 *
 * See https://mavlink.io/en/services/command.html for more details.
 */
class CommandProtocol
{
public:
  CommandProtocol();
  CommandProtocol(GCSTransceiver* trans, const MAVConnInterface::Ptr link);

  /**
   * @brief Handler for message reception from the parented GCSTransceiver.
   * @param[in] msg Message to handle.
   *
   * This handler serves to branch messages to proper sub-handlers.
   */
  void handleCommand(const mavlink_message_t* msg);

private:
  ros::ServiceClient reqSetArming;
  ros::ServiceClient reqStartMission;
  ros::ServiceClient reqSetMode;

  /**
   * @brief Link to handle the command microservice.
   */
  MAVConnInterface::Ptr link;

  /**
   * @brief Parented GCSTranceiver.
   *
   * This pointer is needed as the microservice needs to filter the autopilot sysid and compid, in some cases to update
   * the system heartbeat and status.
   */
  GCSTransceiver* trans;

  /**
   * @brief Handler for COMMAND_LONG messages.
   * @param[in] msg Message to handle.
   */
  void handleCommandLong(const mavlink_message_t* msg);
  void handleCommandInt(const mavlink_message_t* msg);

  /**
   * @brief Handler for ROS services for COMMAND_LONG messages.
   * @param[in] srv Service message to request/response.
   * @return true if the targeted service properly responses.
   *
   * Each type of COMMAND_LONG (differing in MAV_CMD field) requires a separate ROS service to carry out command. This
   * handler serves to branch service messages to proper services aimed for specific command types.
   */
  bool requestCommandLong(CommandLong& srv);
  bool requestCommandInt(CommandInt& srv);

  /**
   * @brief Handler for system status update.
   * @param[in] srv Service content to update from.
   *
   * This handler serves to branch service content to proper sub-updaters aimed for specific command types.
   */
  void updateSystemStatus(const CommandLong& srv);
  void updateSystemStatus(const CommandInt& srv);

  /**
   * @brief Updater for arming status.
   * @param[in] srv Service content to update from.
   */
  void updateArmStatus(const CommandLong& srv);

  /**
   * @brief Handler for SET_MODE messages.
   * @param[in] msg Message to handle.
   */
  void handleSetMode(const mavlink_message_t* msg);
};

#endif // COMMAND_PROTOCOL_H
