#include "gcs_transceiver.h"
#include "mission_protocol.h"
GCSTransceiver::GCSTransceiver()
{
  /// Get local parameters of this node through roslaunch file.
  ros::NodeHandle private_nh("~");
  private_nh.getParam("main_period", main_period);
  private_nh.getParam("main_url", main_url);

  /// Temporary holders for autopilot_sysid and gcs_sysid as this method cannot parse uint8_t type.
  int autopilot_sysid_, gcs_sysid_;
  private_nh.getParam("autopilot_sysid", autopilot_sysid_);
  private_nh.getParam("gcs_sysid", gcs_sysid_);
  autopilot_sysid = static_cast<uint8_t>(autopilot_sysid_);
  gcs_sysid = static_cast<uint8_t>(gcs_sysid_);

  /// Autopilot's compid must be 1 (MAV_COMP_ID_AUTOPILOT1) in order for QGroundControl to recognize.
  autopilot_compid = static_cast<uint8_t>(MAV_COMPONENT::COMP_ID_AUTOPILOT1);

  /// GCS's compid is always 0 (MAV_COMP_ID_ALL) in QGroundControl, which means that its message must be processed by
  /// all autopilots in the network.
  gcs_compid = static_cast<uint8_t>(MAV_COMPONENT::COMP_ID_ALL);

  /// Construct a new MAVLink link management from main_url.
  MainLink = MAVConnInterface::open_url(main_url);

  /// Set the sender's sysid and compid of the link.
  MainLink->set_system_id(autopilot_sysid);
  MainLink->set_component_id(autopilot_compid);

  /// Assign a callback function to trigger when a new MAVLink message arrives.
  MainLink->message_received_cb = boost::bind(&GCSTransceiver::onMainLinkCallBack, this, _1, _2);

  /// Use main link to handle all microservices.
  missionProtocol = new MissionProtocol(this, MainLink);
  commandProtocol = new CommandProtocol(this, MainLink);
  parameterProtocol = new ParameterProtocol(this, MainLink);

  /// Initialize values for Hearbeat and SysStatus.
  setupHeartbeat();
  setupSysStatus();

  ros::NodeHandle nh;

  /// Get global parameters of the ROS ecosystem through roslaunch file.
  /// Global position of NED reference frame is necessary for displaying filtered odometry to GCS.
  nh.getParam("ned_lat", ned_lat);
  nh.getParam("ned_lon", ned_lon);

  /// ROS subscription and publication from and to ROS messages from other nodes.
  subGPS = nh.subscribe("gps/fix", 10, &GCSTransceiver::onGPSCallBack, this);
  subPressure = nh.subscribe("pressure/data", 10, &GCSTransceiver::onPressureCallBack, this);
  subIMU = nh.subscribe("imu/data", 10, &GCSTransceiver::onIMUCallBack, this);
  subOdom = nh.subscribe("odom", 10, &GCSTransceiver::onOdomCallBack, this);
  subSetpoint = nh.subscribe("setpoint", 10, &GCSTransceiver::onSetpointCallBack, this);
  subThrusterCmd = nh.subscribe("thruster/cmd", 10, &GCSTransceiver::onThrusterCmdCallBack, this);
  subMassShifter = nh.subscribe("massshifter/data", 10, &GCSTransceiver::onMassShifterCallBack, this);

  /// ROS timer for periodic publication of Hearbeat and SysStatus.
  loopMainLink = nh.createTimer(ros::Duration(main_period), &GCSTransceiver::onMainLinkLoop, this);

}

GCSTransceiver::~GCSTransceiver()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void GCSTransceiver::setupHeartbeat()
{
  /// Specify which system (or vehicle) the autopilot resides in.
  Heartbeat.type = static_cast<uint8_t>(MAV_TYPE::SUBMARINE);

  /// Specify the type of autopilot (must always be ARDUPILOTMEGA for now).
  Heartbeat.autopilot = static_cast<uint8_t>(MAV_AUTOPILOT::ARDUPILOTMEGA);

  /// Base mode is only useful if we use GENERIC autopilot. For ARDUPILOTMEGA, always set this to CUSTOM_MODE_ENABLED.
  Heartbeat.base_mode = static_cast<uint8_t>(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED);

  /// This is the one that we can choose in GCS if ARDUPILOTMEGA is used.
  /// See /opt/ros/melodic/include/mavlink/v2.0/ardupilotmega/ardupilotmega.hpp for list of supported custom modes.
  Heartbeat.custom_mode = static_cast<uint8_t>(mavlink::ardupilotmega::ROVER_MODE::INITIALIZING);

  /// Initial system status.
  Heartbeat.system_status = static_cast<uint8_t>(MAV_STATE::STANDBY);
}

void GCSTransceiver::setupSysStatus()
{
  SysStatus.onboard_control_sensors_present = static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::GPS) |
                                              static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_GYRO) |
                                              static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_ACCEL);
  SysStatus.onboard_control_sensors_enabled = static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::GPS) |
                                              static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_GYRO) |
                                              static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_ACCEL);
  SysStatus.onboard_control_sensors_health = static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::GPS) |
                                             static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_GYRO) |
                                             static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_ACCEL);
  SysStatus.voltage_battery = 24000;
  SysStatus.current_battery = 190;
  SysStatus.battery_remaining = 70;//%Pin con lai
}

void GCSTransceiver::onMainLinkLoop(const ros::TimerEvent& /*event*/)
{
  pack_and_send_mavlink_message_t(Heartbeat, MainLink);
  pack_and_send_mavlink_message_t(SysStatus, MainLink);
}

void GCSTransceiver::onMainLinkCallBack(const mavlink_message_t* msg, const Framing framing)
{
  /// Only procceed if message is error-free.
  if (framing == mavconn::Framing::ok)
  {
    ROS_INFO_STREAM("msgid = " << int(msg->msgid));

    /// In multivehicle scenario, multiple vehicles with their own autopilots can connect any exchange MAVLink messages
    /// with one another. Handling of such vehicle-related data is not the goal of this class, so non-GCS messages are
    /// filtered out.
    if (msg->sysid == gcs_sysid && msg->compid == gcs_compid)
    {
      /// Direct message to appropriate microservice manager based on its ID.
      switch (msg->msgid)
      {
      case 44:
      case 73:
      case 43:
      case 51:
      case 41:
      case 45:
        missionProtocol->handleMission(msg);
        break;
      case 76:
      case 75:
      case 11:
        commandProtocol->handleCommand(msg);
        break;
      case 21:
      case 20:
      case 23:
        parameterProtocol->handleParameter(msg);
        break;
      default:
        break;
      }
    }
  }
}

void GCSTransceiver::onPressureCallBack(const FluidPressure::ConstPtr& msg)
{
  SCALED_PRESSURE2 pack;

  pack.time_boot_ms = static_cast<uint64_t>(msg->header.stamp.toNSec() * 1e-6);
  pack.press_abs = static_cast<float>(msg->fluid_pressure*1e-3);
  pack.press_diff = static_cast<float>(msg->variance);
  pack.temperature = 3000;

  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onGPSCallBack(const NavSatFix::ConstPtr& msg)
{
  GPS_RAW_INT pack;

  pack.time_usec = static_cast<uint64_t>(msg->header.stamp.toNSec() * 1e-3);
  pack.lat = static_cast<int32_t>(msg->latitude * 1e7);
  pack.lon = static_cast<int32_t>(msg->longitude * 1e7);
  pack.alt = static_cast<int32_t>(msg->altitude * 1e3);
  //ROS_INFO("--TIME STAMP--");
  //ROS_INFO("stamp: %lu", static_cast<uint64_t>(msg->header.stamp.toSec()*1e3));
  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onIMUCallBack(const Imu::ConstPtr& msg)
{
  RAW_IMU pack;
  pack.time_usec = static_cast<uint64_t>(msg->header.stamp.toNSec() * 1e-3);
  pack.xacc = static_cast<int16_t>(msg->linear_acceleration.x);
  pack.yacc = static_cast<int16_t>(msg->linear_acceleration.y);
  pack.zacc = static_cast<int16_t>(msg->linear_acceleration.z);
  pack.xgyro = static_cast<int16_t>(msg->angular_velocity.x);
  pack.ygyro = static_cast<int16_t>(msg->angular_velocity.y);
  pack.zgyro = static_cast<int16_t>(msg->angular_velocity.z);

  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onOdomCallBack(const Odometry::ConstPtr& msg)
{
  GLOBAL_POSITION_INT pack1;
  pack1.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e-6);
  double lat, lon;
  convert_local_to_global_coords(msg->pose.pose.position.x, msg->pose.pose.position.y, ned_lat, ned_lon, lat, lon);
  pack1.lat = static_cast<int32_t>(lat * 1e7);
  pack1.lon = static_cast<int32_t>(lon * 1e7);
  pack1.alt = static_cast<int32_t>(-msg->pose.pose.position.z * 1e3);
  pack1.vx = static_cast<int16_t>(msg->twist.twist.linear.x * 1e2);
  pack1.vy = static_cast<int16_t>(msg->twist.twist.linear.y * 1e2);
  pack1.vz = static_cast<int16_t>(msg->twist.twist.linear.z * 1e2);

  float quarternion[4];
  quarternion[0] = static_cast<float>(msg->pose.pose.orientation.w);
  quarternion[1] = static_cast<float>(msg->pose.pose.orientation.x);
  quarternion[2] = static_cast<float>(msg->pose.pose.orientation.y);
  quarternion[3] = static_cast<float>(msg->pose.pose.orientation.z);
  float euler[3];
  mavlink_quaternion_to_euler(quarternion, euler, euler + 1, euler + 2);
  ATTITUDE pack2;
  pack2.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e-6);
  pack2.roll = euler[0];
  pack2.pitch = euler[1];
  pack2.yaw = euler[2];
  pack2.rollspeed = static_cast<float>(msg->twist.twist.angular.x);
  pack2.pitchspeed = static_cast<float>(msg->twist.twist.angular.y);
  pack2.yawspeed = static_cast<float>(msg->twist.twist.angular.z);

  pack_and_send_mavlink_message_t(pack1, MainLink);
  pack_and_send_mavlink_message_t(pack2, MainLink);
}

void GCSTransceiver::onSetpointCallBack(const Float64MultiArrayStamped::ConstPtr& msg)
{
  POSITION_TARGET_GLOBAL_INT pack;
  pack.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e-6);
  pack.coordinate_frame = static_cast<uint8_t>(MAV_FRAME::GLOBAL_INT);
  pack.type_mask = static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::X_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::Y_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::Z_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::VY_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::VZ_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::AX_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::AY_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::AZ_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::YAW_RATE_IGNORE);
  pack.vx = static_cast<float>(msg->data[0]);
  pack.yaw = static_cast<float>(msg->data[1]);

  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onThrusterCmdCallBack(const Float64MultiArrayStamped::ConstPtr& msg)
{
  ACTUATOR_CONTROL_TARGET pack;
  pack.time_usec = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e-3);
  for (size_t i = 0; i < msg->data.size(); i++)
    pack.controls.data()[i] = static_cast<float>(msg->data[i]);

  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onMassShifterCallBack(const Float64MultiArrayStamped::ConstPtr& msg)
{
  CUSTOM_MSG_MASS_SHIFTER pack;
  pack.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e-6);
  pack.position = static_cast<float>(msg->data[0]);
  pack.motor_duty = static_cast<float>(msg->data[1]);
  pack.motor_temp = static_cast<float>(msg->data[2]);
  pack.motor_current = static_cast<float>(msg->data[3]);
  pack.motor_rspeed = static_cast<float>(msg->data[4]);
  pack.motor_dspeed = static_cast<float>(msg->data[5]);
//  PLAY_TUNE pack;
//  pack.target_system = 1;
//  pack.target_component = 1;
//  pack.tune[0] = 'H';
  pack_and_send_mavlink_message_t(pack, MainLink);
//  ROS_INFO("--CUSTOM MESSAGE--");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gcs_transceiver");
  GCSTransceiver trans;
  ros::spin();
}
