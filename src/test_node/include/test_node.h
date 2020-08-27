#ifndef TEST_NODE_H
#define TEST_NODE_H

#include <ros/ros.h>

#include <mavros_msgs/WaypointList.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs_stamped/Float64MultiArrayStamped.h>

#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/SetMode.h>

#include <tf2/LinearMath/Quaternion.h>

using namespace nav_msgs;
using namespace sensor_msgs;
using namespace std_msgs_stamped;
using namespace mavros_msgs;

/**
 * @brief Test class for checking GCSTransciever's functionalities.
 */
class TestNode
{
public:
  TestNode();
  ~TestNode();

private:
  ros::Publisher pubGPS;
  ros::Publisher pubIMU;
  ros::Publisher pubOdom;
  ros::Publisher pubSetpoint;
  ros::Publisher pubThrusterCmd;
  ros::Publisher pubPressure;
  ros::Publisher pubMassShifter;
  ros::Timer loopTestData;
  ros::WallTime start_time;
  double last_time;

  void onTestDataLoopCallBack(const ros::TimerEvent& event);

  ros::Subscriber subItemList;
  void onItemListCallBack(const WaypointList::ConstPtr& msg);

  ros::ServiceServer resSetArming;
  ros::ServiceServer resStartMission;
  ros::ServiceServer resSetMode;
  bool onSetArmingCallBack(CommandLongRequest& req, CommandLongResponse& res);
  bool onStartMissionCallBack(CommandLongRequest& req, CommandLongResponse& res);
  bool onSetModeCallBack(SetModeRequest& req, SetModeResponse& res);

  ros::ServiceServer resSetLOSParams;
  ros::ServiceServer resGetLOSParams;

  ros::ServiceServer resSetHeadingPID;
  ros::ServiceServer resGetHeadingPID;

  ros::ServiceServer resGetLightParam;
  ros::ServiceServer resSetLightParam;

  ros::ServiceServer resGetComPassParam;
  ros::ServiceServer resGetBatteryParam;
  ros::ServiceServer resGetFailSafeParam;
  ros::ServiceServer resGetAccelParam;
  ros::ServiceServer resGetATCParam;
  ros::ServiceServer resGetARMINGParam;

  ros::ServiceServer resGetDOITRONGParam;
  ros::ServiceServer resSetDOITRONGParam;

  bool onGetDOITRONGCallBack(ParamGetRequest&  req, ParamGetResponse& res);
  bool onSetDOITRONGCallBack(ParamSetRequest&  req, ParamSetResponse& res);

  bool onSetLOSParamsCallBack(ParamSetRequest&  req, ParamSetResponse& res);
  bool onGetLOSParamsCallBack(ParamGetRequest&  req, ParamGetResponse& res);

  bool onSetHeadingPIDCallBack(ParamSetRequest& req, ParamSetResponse& res);
  bool onGetHeadingPIDCallBack(ParamGetRequest& req, ParamGetResponse& res);

  bool onGetLightParamCallBack(ParamGetRequest& req, ParamGetResponse& res);
  bool OnSetLightParamCallBack(ParamSetRequest& req, ParamSetResponse& res);

  bool OnGetComPassCallBack(ParamGetRequest& req, ParamGetResponse& res);
  bool OnGetBatteryCallBack(ParamGetRequest& req,ParamGetResponse& res);
  bool OnGetFailSafeCallBack(ParamGetRequest& req,ParamGetResponse& res);
  bool OnGetAccelCallBack(ParamGetRequest& req,ParamGetResponse& res);
  bool OnGetATCCallBack(ParamGetRequest& req,ParamGetResponse& res);
  bool OnGetARMINGCallBack(ParamGetRequest& req,ParamGetResponse& res);


  inline bool compareString(const char* str1, const char* str2) { return !strncmp(str1, str2, strlen(str2)); }
};

#endif // TEST_NODE_H
