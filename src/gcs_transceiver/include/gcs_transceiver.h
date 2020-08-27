#ifndef GCS_TRANSCEIVER_H
#define GCS_TRANSCEIVER_H

#include <mavconn/interface.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs_stamped/Float64MultiArrayStamped.h>

#include "geo.h"
#include "mavlink.h"

#include "command_protocol.h"
#include "mission_protocol.h"
#include "parameter_protocol.h"

using namespace mavconn;
using namespace mavlink;
using namespace mavlink::common;
using namespace mavlink::common::msg;
using namespace nav_msgs;
using namespace sensor_msgs;
using namespace std_msgs_stamped;

/**
 * @brief Bridge for MAVLink-compliant communication with QGroundControl.
 *
 * In MAVLink's terminology, a system is a complete set of hardware/software. A single piece of entity which makes up
 * the system is termed a component. For example, a vehicle (USV, ROV, AUV, etc), an antenna tracker or a ground control
 * center can all be called systems. Lidar, camera, GNSS/INS, AHRS, servos, etc can be considered the hardware
 * components and path planner, avoidance planner is the software ones of the system. Every component has their unique
 * component IDs, and every system has their unique system IDs also.
 *
 * An autopilot is a piece of software that fuses sensors' data for environmental perception and make maneveuring
 * decisions via setpoints to reach a particular target. In reality, not all sensors and actuators are able to directly
 * handle MAVLink messages. As a result, a common design philosophy is to consider autopilot, sensors and actuators a
 * single "generalized" MAVLink component with extended abilities to measure physical quantities and exert pulses to
 * motor drivers. ArduPilot and PX4 are the two most popular autopilots of that kind.
 *
 * A ground control station (GCS) is a ground-based set of facilities for operators to remotely control other systems.
 * This typically including human-machine interface, video capture, mission planner, logger, map system, etc, all of
 * which are usually integrated into a computer software running on desktop and mobile OSes. Some popular GCSes include
 * QGroundControl and MissionPlanner. Similar to autopilot, GCS is considered a single MAVLink component, and quite
 * frequently a model of "1 system, 1 component" for GCS is implemented when the overall system is not too
 * sophisticated.
 *
 * This class is responsible for transceiving messages between GCS and other MAVLink-enabled systems. Traditionlly, only
 * PX4, ArduPilot, etc running on predesigned microcontroller-centered boards such as Pixhawks can communicate with GCS.
 * This class attempts to mimic some ArduPilot's functionalities by means of ROS tools, thus providing any embedded
 * computer like Raspberry PI with the capability to directly handle data with GCS.
 *
 * The mavros package must be preinstalled into the ROS system as this class relies on underneath UDP/TCP/Serial
 * implementations of libmavconn library.
 */
class GCSTransceiver
{
public:
  GCSTransceiver();
  ~GCSTransceiver();

  /**
   * @brief Interface for link management between GCS and autopilot.
   *
   * This link is termed MainLink because most of important system's info is delivered through an autopilot, following
   * the concept of "generalized" autopilot discussed aboved.
   *
   * A connection link is defined by an URL, which composes of a transporting method (UDP/TCP/Serial), IP addresses and
   * ports of the two end systems, as well as component ID of the autopilot and system ID of the system that the
   * autopilot belongs to.
   *
   * See http://wiki.ros.org/mavros for all supported URL's schemas.
   */
  MAVConnInterface::Ptr MainLink;

  uint8_t autopilot_sysid;
  uint8_t autopilot_compid;
  uint8_t gcs_sysid;
  uint8_t gcs_compid;
  double ned_lat;
  double ned_lon;

  /**
   * @brief Message to maintain connection between GCS and autopilot.
   *
   * This message informs GCS of the system type (surface boat, submarine, antenna tracker, etc), autopilot type
   * (generic, PX4, ArduPilot, etc), modes of operation (manual, guided, auto, etc) and system status (standby, active,
   * poweroff, etc). A typical system must only poccess one autopilot. Periodic transmission of heartbeat to signal the
   * system presence to GCS is required.
   *
   * See https://mavlink.io/en/messages/common.html#HEARTBEAT for detailed information of all message fields.
   */
  HEARTBEAT Heartbeat;

  /**
   * @brief Message to inform GCS of the system state, including sensors' health and battery's info.
   *
   * See https://mavlink.io/en/messages/common.html#SYS_STATUS for detailed information of all message fields.
   */
  SYS_STATUS SysStatus;

private:
  double main_period;
  std::string main_url;

  /**
   * @brief Callback when a MAVLink message on main link arrives.
   * @param[in] msg Structure of MAVLink message.
   * @param[in] framing state of message.
   *
   * Raw bytestream is received via the underlying UDP/TCP/Serial implementations of MainLink. Then it is unpacked,
   * checked for error and integrity before callback is triggered.
   *
   * Each message type is uniquely defined by a message ID. Depending on message ID, each message reception can be
   * intepreted as belonging to a standard microservice. A microservice is a sequence of message requests and responses
   * to accomplish a particular task. For ease of implementation, each microservice is handled separately by a class.
   * Currently, three microservices (MissionProtocol, CommandProtocol, ParameterProtocol) have been supported.
   *
   * See https://mavlink.io/en/services/ for more information about all standard microservices.
   */
  void onMainLinkCallBack(const mavlink_message_t* msg, const Framing framing);

  /**
   * @brief Manager to handle mission microservice.
   */
  MissionProtocol* missionProtocol = nullptr;

  /**
   * @brief Manager to handle command microservice.
   */
  CommandProtocol* commandProtocol = nullptr;

  /**
   * @brief Manager to handle parameter microservice.
   */
  ParameterProtocol* parameterProtocol = nullptr;

  ros::Subscriber subMassShifter;

  void onMassShifterCallBack(const Float64MultiArrayStamped::ConstPtr& msg);

  ros::Subscriber subPressure;

  void onPressureCallBack(const FluidPressure::ConstPtr& msg);
  /**
   * @brief ROS Subscriber to raw GPS data on the system.
   *
   * Raw GPS measurements include latitude, longitude and altitude. The altitude returned by GPS is often of AMSL (Above
   * Mean Sea Level) type. Be aware that QGroundControl also support relative alitude, which indicates height of the
   * vehicle above a launch position.
   */
  ros::Subscriber subGPS;

  /**
   * @brief Callback for GPS subscription.
   * @param[in] msg Raw GPS message.
   */
  void onGPSCallBack(const NavSatFix::ConstPtr& msg);

  /**
   * @brief ROS Subscriber to raw IMU data on the system.
   *
   * Raw IMU measurements include angular velocity from IMU's gyroscope and linear acceleration from IMU's
   * accelerometer. Some IMUs provides magnetic flux density from their internal magnetometer as well.
   */
  ros::Subscriber subIMU;

  /**
   * @brief Callback for IMU subscription.
   * @param[in] msg Raw IMU message.
   */
  void onIMUCallBack(const Imu::ConstPtr& msg);

  /**
   * @brief ROS Subscriber to system odometry.
   *
   * Odometry refers to the pose (position & orientation) and twist (linear & angular velocities) of a vehicle. In most
   * cases, odometric data is local and expressed in NED reference frame. Developers should fuse raw dead-reckoning
   * measurements from encoders, GPSes, IMUs with more accurate data from lidars, cameras to create a final filtered
   * odometry.
   */
  ros::Subscriber subOdom;

  /**
   * @brief Callback for odometry subscription.
   * @param[in] msg Filtered odometry message.
   */
  void onOdomCallBack(const Odometry::ConstPtr& msg);

  /**
   * @brief ROS Subscriber to system setpoint.
   *
   * Setpoint refers to a set of desired motion variables that must be obeyed by a vehicle to reach a particular state.
   * For a rover system (USV, AGV), this typically includes desired surge and desired yaw.
   */
  ros::Subscriber subSetpoint;

  /**
   * @brief Callback for setpoint subscription.
   * @param[in] msg Setpoint message.
   */
  void onSetpointCallBack(const Float64MultiArrayStamped::ConstPtr& msg);

  /**
   * @brief ROS Subscriber to commands a system sends to thrusters.
   *
   * Most motors are commanded by generating PWM pulses to control their rotating speed and direction. From an
   * autopilot's perspective, this means sending a normalized floating-point number between -1 and 1, called duty cycle,
   * to a pulse generator, typically a microcontroller or a FPGA.
   */
  ros::Subscriber subThrusterCmd;

  /**
   * @brief Callback for thruster commands subscription.
   * @param[in] msg Thruster commands message.
   */
  void onThrusterCmdCallBack(const Float64MultiArrayStamped::ConstPtr& msg);

  /**
   * @brief ROS Timer to carry out period transmission of Heartbeat, SysStatus,
   * etc.
   */
  ros::Timer loopMainLink;

  /**
   * @brief Callback when timer overruns to handle Heartbeat, SysStatus
   * transmisstion.
   * @param[in] event Structure providing timestamp in ROS system clock.
   */
  void onMainLinkLoop(const ros::TimerEvent& event);

  /**
   * @brief Initial values of Heartbeat
   */
  void setupHeartbeat();

  /**
   * @brief Initial values of SysStatus
   */
  void setupSysStatus();
};

#endif // GCS_TRANSCEIVER_H
