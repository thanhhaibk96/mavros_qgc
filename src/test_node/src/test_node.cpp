#include "test_node.h"

TestNode::TestNode()
{
  ros::NodeHandle nh;

  pubGPS = nh.advertise<NavSatFix>("gps/fix", 1);
  pubIMU = nh.advertise<Imu>("imu/data", 1);
  pubPressure = nh.advertise<FluidPressure>("pressure/data", 1);
  pubOdom = nh.advertise<Odometry>("odom", 1);
  pubSetpoint = nh.advertise<Float64MultiArrayStamped>("setpoint", 1);
  pubThrusterCmd = nh.advertise<Float64MultiArrayStamped>("thruster/cmd", 1);
  pubMassShifter = nh.advertise<Float64MultiArrayStamped>("massshifter/data", 1);
  loopTestData = nh.createTimer(ros::Duration(0.5), &TestNode::onTestDataLoopCallBack, this);

  subItemList = nh.subscribe("mission/item_list", 1, &TestNode::onItemListCallBack, this);

  resSetArming = nh.advertiseService("command/set_arming", &TestNode::onSetArmingCallBack, this);
  resStartMission = nh.advertiseService("command/start_mission", &TestNode::onStartMissionCallBack, this);
  resSetMode = nh.advertiseService("command/set_mode", &TestNode::onSetModeCallBack, this);

  resSetLOSParams = nh.advertiseService("parameter/set_LOS_params", &TestNode::onSetLOSParamsCallBack, this);
  resGetLOSParams = nh.advertiseService("parameter/get_LOS_params", &TestNode::onGetLOSParamsCallBack, this);

  resSetHeadingPID = nh.advertiseService("parameter/set_heading_PID", &TestNode::onSetHeadingPIDCallBack, this);
  resGetHeadingPID = nh.advertiseService("parameter/get_heading_PID", &TestNode::onGetHeadingPIDCallBack, this);

  resSetLightParam=  nh.advertiseService("parameter/set_Light_Param", &TestNode::OnSetLightParamCallBack,this);
  resGetLightParam=  nh.advertiseService("parameter/get_Light_Param", &TestNode::onGetLightParamCallBack,this);

  resGetComPassParam=nh.advertiseService("parameter/get_ComPass_param",&TestNode::OnGetComPassCallBack,this);

  resGetBatteryParam=nh.advertiseService("parameter/get_battery_param",&TestNode::OnGetBatteryCallBack,this);

  resGetFailSafeParam=nh.advertiseService("parameter/get_failsafe_param",&TestNode::OnGetFailSafeCallBack,this);

  resGetAccelParam=nh.advertiseService("parameter/get_accel_param",&TestNode::OnGetAccelCallBack,this);

  resGetATCParam=nh.advertiseService("parameter/get_atc_param",&TestNode::OnGetATCCallBack,this);

  resGetARMINGParam=nh.advertiseService("parameter/get_arming_param",&TestNode::OnGetARMINGCallBack,this);

  resGetDOITRONGParam=nh.advertiseService("parameter/get_doitrong_param",&TestNode::onGetDOITRONGCallBack,this);

  resSetDOITRONGParam=nh.advertiseService("parameter/set_doitrong_param",&TestNode::onSetDOITRONGCallBack,this);

  start_time = ros::WallTime::now();
}

TestNode::~TestNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void TestNode::onTestDataLoopCallBack(const ros::TimerEvent& event)
{
  last_time = ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();

  /// Test Mass Shifter data
  Float64MultiArrayStamped massShifter_Status;
  massShifter_Status.header.stamp = static_cast<ros::Time>(last_time);
//  float position; /*<  The position of mass shifter in AUV */
//  float motor_duty; /*<  The duty cycle (PWM) of mass shifter's motor */
//  float motor_temp; /*<  The temperature of mass shifter's motor */
//  float motor_current; /*<  The current of mass shifter's motor */
//  float motor_rspeed; /*<  The current speed of mass shifter's motor */
//  float motor_dspeed; /*<  The desired speed of mass shifter's motor */
  massShifter_Status.data.push_back(35.9); // position
  massShifter_Status.data.push_back(40.0); // duty cycle
  massShifter_Status.data.push_back(29.5); // temperature
  massShifter_Status.data.push_back(5.0); // current
  massShifter_Status.data.push_back(1350.5); // current speed
  massShifter_Status.data.push_back(1500); // desired speed
  pubMassShifter.publish(massShifter_Status);

  /// Test GPS raw data
  NavSatFix gpsMsg;
  gpsMsg.header.stamp = static_cast<ros::Time>(last_time);
  gpsMsg.latitude = 10.7855361;
  gpsMsg.longitude = 106.6641680;
  gpsMsg.altitude = 0.0;
  pubGPS.publish(gpsMsg);

  /// Test IMU raw data
  Imu imuMsg;
  imuMsg.header.stamp = static_cast<ros::Time>(last_time);
  imuMsg.angular_velocity.x = 0.25;
  imuMsg.angular_velocity.y = 1.36;
  imuMsg.angular_velocity.z = -0.45;
  imuMsg.linear_acceleration.x = 1.36;
  imuMsg.linear_acceleration.y = 2.36;
  imuMsg.linear_acceleration.z = -1.27;
  pubIMU.publish(imuMsg);

  /// Test filtered odometry
  Odometry odomMsg;
  odomMsg.header.stamp = static_cast<ros::Time>(last_time);
  odomMsg.pose.pose.position.x = 2.36;
  odomMsg.pose.pose.position.y = -4.36;
  odomMsg.pose.pose.position.z = -0.56;
  tf2::Quaternion quat;
  quat.setRPY(0.1, -0.45, 1.36);
  odomMsg.pose.pose.orientation.w = quat.w();
  odomMsg.pose.pose.orientation.x = quat.x();
  odomMsg.pose.pose.orientation.y = quat.y();
  odomMsg.pose.pose.orientation.z = quat.z();
  odomMsg.twist.twist.linear.x = 0.36;
  odomMsg.twist.twist.linear.y = 1.66;
  odomMsg.twist.twist.linear.z = -0.12;
  odomMsg.twist.twist.angular.x = 1.36;
  odomMsg.twist.twist.angular.y = 0.45;
  odomMsg.twist.twist.angular.z = -1.36;
  pubOdom.publish(odomMsg);

  /// Test setpoint
  Float64MultiArrayStamped setpointMsg;
  setpointMsg.header.stamp = static_cast<ros::Time>(last_time);
  setpointMsg.data.push_back(1.2);
  setpointMsg.data.push_back(0.6);
  pubSetpoint.publish(setpointMsg);

  /// Test thruster commands
  Float64MultiArrayStamped cmdMsg;
  cmdMsg.header.stamp = static_cast<ros::Time>(last_time);
  cmdMsg.data.push_back(0.6);
  cmdMsg.data.push_back(-0.3);
  pubThrusterCmd.publish(cmdMsg);

  /// Pressure sensor data
  FluidPressure fluidPressure;
  fluidPressure.header.stamp = static_cast<ros::Time>(last_time);
  fluidPressure.fluid_pressure = 154123.0;
  fluidPressure.variance = 0;
  pubPressure.publish(fluidPressure);
}

void TestNode::onItemListCallBack(const WaypointList::ConstPtr& msg)
{
  for (auto it = msg->waypoints.begin(); it != msg->waypoints.end(); it++)
  {
    switch (it->command)
    {
    case 16: // MAV_CMD_NAV_WAYPOINT
      ROS_INFO_STREAM("Waypoint: "
                      << "lat = " << it->x_lat * 1e-7 << ", lon = " << it->y_long * 1e-7 << ", alt = " << it->z_alt
                      << ".");
      break;
    case 20: // MAV_CMD_NAV_RETURN_TO_LAUNCH
      ROS_INFO_STREAM("Waypoint (return to launch): "
                      << "lat = (1st waypoint lat), lon = (1st waypoint lon)");
      break;
    }
  }
}

bool TestNode::onSetArmingCallBack(CommandLongRequest& req, CommandLongResponse& res)
{
  if (req.param1 == 1.0f)
    ROS_INFO("Thruster unlocked.");
  else if (req.param1 == 0.0f)
    ROS_INFO("Thruster locked.");
  res.result = 0; // MAV_RESULT_ACCEPTED

  return true;
}

bool TestNode::onStartMissionCallBack(CommandLongRequest& /*req*/, CommandLongResponse& res)
{
  ROS_INFO("Mission started.");
  res.result = 0; // MAV_RESULT_ACCEPTED

  return true;
}

bool TestNode::onSetModeCallBack(SetModeRequest& req, SetModeResponse& res)
{
  ROS_INFO_STREAM("Base mode set to " << int(req.base_mode) << ", custom mode set to " << req.custom_mode << ".");
  res.mode_sent = 0; // MAV_RESULT_ACCEPTED

  return true;
}

bool TestNode::onSetLOSParamsCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "LOS_radius"))
  {
    ROS_INFO_STREAM("LOS_radius set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_min_delta"))
  {
    ROS_INFO_STREAM("LOS_min_delta set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_max_delta"))
  {
    ROS_INFO_STREAM("LOS_max_delta set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_beta"))
  {
    ROS_INFO_STREAM("LOS_beta set to " << req.value.real << ".");
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool TestNode::onGetLOSParamsCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "LOS_radius"))
  {
    res.value.real = 3.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_min_delta"))
  {
    res.value.real = 2.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_max_delta"))
  {
    res.value.real = 5.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_beta"))
  {
    res.value.real = 0.2;
    res.success = true;
  }

  return res.success;
}

bool TestNode::onSetHeadingPIDCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "heading_Kp"))
  {
    ROS_INFO_STREAM("heading_Kp set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "heading_Ki"))
  {
    ROS_INFO_STREAM("heading_Ki set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "heading_Kd"))
  {
    ROS_INFO_STREAM("heading_Kd set to " << req.value.real << ".");
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool TestNode::onGetHeadingPIDCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "heading_Kp"))
  {
    res.value.real = 3.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "heading_Ki"))
  {
    res.value.real = 0.01;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "heading_Kd"))
  {
    res.value.real = 1.2;
    res.success = true;
  }

  return res.success;
}
bool TestNode::OnSetLightParamCallBack(ParamSetRequest &req, ParamSetResponse &res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "SERVO6_FUNCTION"))
  {
    ROS_INFO_STREAM("Light set to Channel 6 with value: " << req.value.integer << ".");
    res.success = true;
  }
  else if (compareString(req.param_id.data(), "SERVO5_FUNCTION"))
  {
    ROS_INFO_STREAM("Light set to Channel 5 with value: " << req.value.integer << ".");
    res.success = true;
  }
  else if (compareString(req.param_id.data(), "KAUTO_MODE"))
  {
    ROS_INFO_STREAM("Set Auto Mode : " << req.value.integer << ".");
    res.success = true;
  }
  else if (compareString(req.param_id.data(), "MOTOR_STATUS"))
  {
    ROS_INFO_STREAM("Set Motor Status : " << req.value.integer << ".");
    res.success = true;
  }
  else if (compareString(req.param_id.data(), "MODE_STATUS"))
  {
    if (req.value.integer == 0)
    {
      ROS_INFO_STREAM(" Status disable: "<<req.value.integer );
    }
    else if (req.value.integer == 1)
    {
      ROS_INFO_STREAM(" Status enable : "<<req.value.integer);
    }
    res.success = true;
  }


  res.value.integer = req.value.integer;
  return res.success;

}

bool TestNode::onGetLightParamCallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;

  if(compareString(req.param_id.data(),"SERVO5_FUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"SERVO6_FUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"SERVO7_FUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"SERVO8_FUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"SERVO9_FUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }

  else if(compareString(req.param_id.data(),"SERVO11_FUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"SERVO12_FUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"SERVO13_FUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"SERVO14_FUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"SERVO10_MAX"))
  {
     res.value.integer=1900;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"SERVO10_MIN"))
  {
     res.value.integer=1000;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"SERVO10_REVERSED"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"SERVO10_TRIM"))
  {
     res.value.integer=1500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"JS_LIGHTS_STEP"))
  {
     res.value.integer=100;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BRD_TYPE"))
  {
     res.value.integer=2;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BRD_PWM_COUNT"))
  {
     res.value.integer=4;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BRD_CAN_ENABLE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BRD_IMU_TARGTEMP"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BRD_SAFETY_MASK"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BRD_SBUS_OUT"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BRD_SER1_RTSCTS"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BRD_SER2_RTSCTS"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOTOR_STATUS"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MODE_STATUS"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"KAUTO_MODE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"AHRS_TRIM_X"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"AHRS_TRIM_Y"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BRD_SAFETYENABLE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BRD_SERIAL_NUM"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"LEAK1_LOGIC"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"LEAK2_LOGIC"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"LEAK3_LOGIC"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"LEAK1_PIN"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"LEAK2_PIN"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"LEAK3_PIN"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"LOG_BACKEND_TYPE"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"LOG_DISARMED"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"LOG_FILE_BUFSIZE"))
  {
     res.value.integer=16;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"LOG_FILE_DSRMROT"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"LOG_REPLAY"))
  {
     res.value.integer=0;
     res.success=true;
  }


  //  if(compareString(req.param_id.data(),"SERIAL0_BAUD"))
  //  {
  //     res.value.integer=115;
  //     res.success=true;
  //  }
   return res.success;
}

bool TestNode::onGetDOITRONGCallBack(ParamGetRequest&  req, ParamGetResponse& res)
{
  res.success=false;
  if(compareString(req.param_id.data(),"DOITRONG"))
  {
     res.value.real=10;
     res.success=true;
  }
  return res.success;
}
bool TestNode::onSetDOITRONGCallBack(ParamSetRequest&  req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "DOITRONG"))
  {
    res.value.real = 10.0;
    res.success = true;
  }
  res.value.real = req.value.real;
  return res.success;
}
//bool onSetDOITRONGCallBack(ParamSetRequest&  req, ParamSetResponse& res)
//{

//}
bool TestNode::OnGetComPassCallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;
  if(compareString(req.param_id.data(),"COMPASS_DEC"))
  {
     res.value.integer=10;
     res.success=true;
  }
  return res.success;
}

bool TestNode::OnGetBatteryCallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;
  if(compareString(req.param_id.data(),"BATT_MONITOR"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT2_MONITOR"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT_CAPACITY"))
  {
     res.value.integer=13000;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT2_CAPACITY"))
  {
     res.value.integer=13000;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT_CURR_PIN"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT2_CURR_PIN"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT_VOLT_PIN"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT2_VOLT_PIN"))
  {
     res.value.integer=1;
     res.success=true;
  }

  return res.success;
}

bool TestNode::OnGetFailSafeCallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;
  if(compareString(req.param_id.data(),"FS_PRESS_ENABLE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_PRESS_MAX"))
  {
     res.value.integer=10000;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_TEMP_ENABLE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_TEMP_MAX"))
  {
     res.value.integer=60;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_EKF_ACTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_EKF_THRESH"))
  {
     res.value.real=0.8;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_BATT_ENABLE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_BATT_MAH"))
  {
     res.value.integer=10000;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_BATT_VOLTAGE"))
  {
     res.value.real=12.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_TERRAIN_ENAB"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_LEAK_ENABLE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_GCS_ENABLE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_PILOT_INPUT"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_PILOT_TIMEOUT"))
  {
     res.value.real=1.00;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"SERVO10_FUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"RC_FEEL_RP"))
  {
     res.value.integer=50;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"WPNAV_RFND_USE"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_AUTO_CONFIG"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_AUTO_SWITCH"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_BLEND_MASK"))
  {
     res.value.integer=5;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_GNSS_MODE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_GNSS_MODE2"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_INJECT_TO"))
  {
     res.value.integer=127;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_MIN_DGPS"))
  {
     res.value.integer=100;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_MIN_ELEV"))
  {
     res.value.integer=-100;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_NAVFILTER"))
  {
     res.value.integer=8;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_DELAY_MS"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_DELAY_MS2"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_ANGMAX_PAN"))
  {
     res.value.integer=4500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_ANGMAX_ROL"))
  {
     res.value.integer=4500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_ANGMAX_TIL"))
  {
     res.value.integer=4500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_ANGMIN_PAN"))
  {
     res.value.integer=-4500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_ANGMIN_ROL"))
  {
     res.value.integer=-4500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_ANGMIN_TIL"))
  {
     res.value.integer=-4500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_DEFLT_MODE"))
  {
     res.value.integer=3;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_JSTICK_SPD"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_RC_IN_PAN"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_RC_IN_ROLL"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_RC_IN_TILT"))
  {
     res.value.integer=8;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_STAB_PAN"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_STAB_ROLL"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_STAB_TILT"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_TYPE"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_1_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_2_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_3_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_4_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_5_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_6_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_7_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_8_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACCEL_FILTER"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC_BODYFIX"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_FAST_SAMPLE"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYRO_FILTER"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR_CAL"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_TRIM_OPTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_USE"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_USE2"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_USE3"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC2_ID"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC3_ID"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC_ID"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR2_ID"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR3_ID"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR3_ID"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR_ID"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_ALT_SOURCE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_BCN_DELAY"))
  {
     res.value.integer=50;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_ENABLE"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_FLOW_DELAY"))
  {
     res.value.integer=10;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_GLITCH_RAD"))
  {
     res.value.integer=25;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_GPS_CHECK"))
  {
     res.value.integer=31;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_GPS_TYPE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_IMU_MASK"))
  {
     res.value.integer=3;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_LOG_MASK"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_MAG_CAL"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_MAG_MASK"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_RNG_USE_HGT"))
  {
     res.value.integer=-1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_TAU_OUTPUT"))
  {
     res.value.integer=25;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK3_ENABLE"))
  {
     res.value.integer=0;
     res.success=true;
  }

  else if(compareString(req.param_id.data(),"EK2_BCN_I_GTE"))
  {
     res.value.integer=500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_CHECK_SCALE"))
  {
     res.value.integer=100;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_EAS_I_GATE"))
  {
     res.value.integer=400;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_FLOW_I_GATE"))
  {
     res.value.integer=300;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_GPS_DELAY"))
  {
     res.value.integer=220;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_HGT_DELAY"))
  {
     res.value.integer=60;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_HGT_I_GATE"))
  {
     res.value.integer=500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_MAG_I_GATE"))
  {
     res.value.integer=300;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_POS_I_GATE"))
  {
     res.value.integer=500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_RNG_I_GATE"))
  {
     res.value.integer=500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_VEL_I_GATE"))
  {
     res.value.integer=500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_MAG_I_GATE"))
  {
     res.value.integer=300;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_YAW_I_GATE"))
  {
     res.value.integer=300;
     res.success=true;
  }




  return res.success;

}

bool TestNode::OnGetAccelCallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;
  if(compareString(req.param_id.data(),"ACCEL_Z_D"))
  {
     res.value.real=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ACCEL_Z_FILT"))
  {
     res.value.real=20.000;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ACCEL_Z_I"))
  {
     res.value.real=0.1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ACCEL_Z_IMAX"))
  {
     res.value.real=100.000;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ACCEL_Z_P"))
  {
     res.value.real=0.5;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ACRO_RP_P"))
  {
     res.value.real=4.5;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ACRO_YAW_P"))
  {
     res.value.real=4.5;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT_AMP_OFFSET"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT_AMP_PERVOLT"))
  {
     res.value.real=17.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT2_AMP_OFFSET"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT2_AMP_PERVOL"))
  {
     res.value.real=17;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"VEL_Z_P"))
  {
     res.value.real=8.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"SURFACE_DEPTH"))
  {
     res.value.real=-10.00;
     res.success=true;
  }

  else if(compareString(req.param_id.data(),"ControlA"))
  {
     res.value.real=10;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ControlB"))
  {
     res.value.real=10;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ControlC"))
  {
     res.value.real=10;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"VEL_XY_FILT_HZ"))
  {
     res.value.real=5.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"VEL_XY_I"))
  {
     res.value.real=0.5;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"VEL_XY_IMAX"))
  {
     res.value.real=1000.00;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"VEL_XY_P"))
  {
     res.value.real=1.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"WPNAV_ACCEL"))
  {
     res.value.real=100.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"WPNAV_ACCEL_Z"))
  {
     res.value.real=100.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"WPNAV_LOIT_JERK"))
  {
     res.value.real=1000.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"WPNAV_LOIT_MAXA"))
  {
     res.value.real=250.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"WPNAV_LOIT_MINA"))
  {
     res.value.real=25.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"WPNAV_LOIT_SPEED"))
  {
     res.value.real=500.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"WPNAV_RADIUS"))
  {
     res.value.real=200.0;
     res.success=true;
  }

  else if(compareString(req.param_id.data(),"WPNAV_SPEED"))
  {
     res.value.real=500.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"WPNAV_SPEED_DN"))
  {
     res.value.real=150.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"WPNAV_SPEED_UP"))
  {
     res.value.real=250.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_POS1_X"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_POS1_Y"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_POS1_Z"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_POS2_X"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_POS2_Y"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_POS2_Z"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_BLEND_TC"))
  {
     res.value.real=10.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC2OFFS_X"))
  {
     res.value.real=0.263029277324676514;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC2OFFS_Y"))
  {
     res.value.real=0.348525434732437134;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC2OFFS_Z"))
  {
     res.value.real=0.893091499805450439;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC2SCAL_X"))
  {
     res.value.real=0.984305381774902344;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC2SCAL_Y"))
  {
     res.value.real=0.986271858215332031;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC2SCAL_Z"))
  {
     res.value.real=1.040588498115539551;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC3OFFS_X"))
  {
     res.value.real=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC3OFFS_Y"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC3OFFS_Z"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC3SCAL_X"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC3SCAL_Y"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC3SCAL_Z"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACCOFFS_X"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACCOFFS_Y"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACCOFFS_Z"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACCSCAL_X"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACCSCAL_Y"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACCSCAL_Z"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR2OFFS_X"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR2OFFS_Y"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR2OFFS_Z"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR3OFFS_X"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR3OFFS_Y"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR3OFFS_Z"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYROFFS_X"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYROFFS_Y"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYROFFS_Z"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_POS1_X"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_POS1_Y"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_POS1_Z"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_POS2_X"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_POS2_Y"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_POS2_Z"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_POS3_X"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_POS3_Y"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_POS3_Z"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_STILL_THRESH"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_ABIAS_P_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_ACC_P_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_ALT_M_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_BCN_M_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_EAS_M_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_FLOW_M_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_GBIAS_P_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_GSCL_P_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_GYRO_P_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_MAGB_P_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_MAGB_P_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_MAGE_P_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_MAG_M_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_MAX_FLOW"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_NOAID_M_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_POSNE_M_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_RNG_M_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_RNG_USE_SPD"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_TERR_GRAD"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_VELD_M_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_VELNE_M_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_WIND_PSCALE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_WIND_P_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_YAW_M_NSE"))
  {
     res.value.real=0.0;
     res.success=true;
  }
  return res.success;
}

bool TestNode::OnGetATCCallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;
  if(compareString(req.param_id.data(),"ATC_ANG_PIT_P"))
  {
     res.value.real=6;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_ANG_RLL_P"))
  {
     res.value.real=6;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_ANG_YAW_P"))
  {
     res.value.real=6;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_PIT_D"))
  {
     res.value.real=0.0003;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_PIT_FF"))
  {
     res.value.real=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_PIT_I"))
  {
     res.value.real=0.03;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_PIT_FILT"))
  {
     res.value.real=30;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_PIT_IMAX"))
  {
     res.value.real=0.44;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_PIT_P"))
  {
     res.value.real=0.135;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_RLL_D"))
  {
     res.value.real=0.003;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_RLL_FF"))
  {
     res.value.real=0.000;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_RLL_FILT"))
  {
     res.value.real=30;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_RLL_I"))
  {
     res.value.real=0.09;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_RLL_IMAX"))
  {
     res.value.real=0.44;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_RLL_P"))
  {
     res.value.real=0.135;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_YAW_D"))
  {
     res.value.real=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_YAW_FF"))
  {
     res.value.real=0.000;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_YAW_FILT"))
  {
     res.value.real=5;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_YAW_I"))
  {
     res.value.real=0.018;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_YAW_IMAX"))
  {
     res.value.real=0.22;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ATC_RAT_YAW_P"))
  {
     res.value.real=0.18;
     res.success=true;
  }


  return res.success;
}

bool TestNode::OnGetARMINGCallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;
  if(compareString(req.param_id.data(),"ARMING_CHECK"))
  {
     res.value.integer=1478;
     res.success=true;
  }
   if(compareString(req.param_id.data(),"ARMING_MIN_VOLT"))
  {
     res.value.real=5.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ARMING_MIN_VOLT2"))
  {
     res.value.real=5.0;
     res.success=true;
  }
  return res.success;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");
  TestNode test;
  ros::spin();
}
