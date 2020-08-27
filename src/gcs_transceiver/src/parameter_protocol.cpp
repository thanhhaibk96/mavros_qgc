#include "parameter_protocol.h"
#include "gcs_transceiver.h"

ParameterProtocol::ParameterProtocol(GCSTransceiver* trans, const MAVConnInterface::Ptr link) : link(link), trans(trans)
{
  headingPID   = {"heading_Kp", "heading_Ki", "heading_Kd"};
  LOSParams    = {"LOS_radius", "LOS_min_delta", "LOS_max_delta", "LOS_beta"};
  Light        = {"SERVO11_FUNCTION","SERVO9_FUNCTION","SERVO8_FUNCTION",
                  "SERVO7_FUNCTION","SERVO6_FUNCTION","SERVO5_FUNCTION","SERVO12_FUNCTION",
                  "SERVO13_FUNCTION","SERVO14_FUNCTION",
                  "SERVO10_MAX","SERVO10_MIN","SERVO10_REVERSED","SERVO10_TRIM","JS_LIGHTS_STEP"
                 ,"BRD_TYPE","BRD_PWM_COUNT","BRD_CAN_ENABLE","BRD_IMU_TARGTEMP"
                 ,"BRD_SAFETY_MASK","BRD_SBUS_OUT","BRD_SER1_RTSCTS","BRD_SER2_RTSCTS"
                 ,"MOTOR_STATUS","MODE_STATUS","KAUTO_MODE"
                 ,"AHRS_TRIM_X","AHRS_TRIM_Y"
                 ,"BRD_SAFETYENABLE","BRD_SERIAL_NUM"
                 ,"LEAK1_LOGIC","LEAK1_PIN","LEAK2_LOGIC","LEAK2_PIN","LEAK3_LOGIC","LEAK3_PIN"
                 ,"LOG_BACKEND_TYPE","LOG_DISARMED","LOG_FILE_BUFSIZE","LOG_FILE_DSRMROT","LOG_REPLAY"

                 };
  Serial       = {"SERIAL0_BAUD"};
  ComPass      = {"COMPASS_DEC"};
  Battery      = {"BATT_MONITOR","BATT2_MONITOR","BATT_CAPACITY","BATT2_CAPACITY"
                  ,"BATT_CURR_PIN","BATT_VOLT_PIN","BATT2_CURR_PIN","BATT2_VOLT_PIN"
                 };
  FailSafe     = {"FS_PRESS_ENABLE","FS_PRESS_MAX","FS_TEMP_ENABLE","FS_TEMP_MAX","FS_EKF_ACTION","FS_BATT_ENABLE","FS_BATT_MAH"
                 ,"FS_TERRAIN_ENAB","FS_LEAK_ENABLE","FS_GCS_ENABLE"
                 ,"INS_ACC3_ID","INS_ACC_ID","INS_GYR2_ID","INS_GYR3_ID","INS_GYR_ID"

                 };
  FailSafeF    = {"FS_EKF_THRESH","FS_BATT_VOLTAGE","FS_PILOT_TIMEOUT"};

  FailSafe8    ={"FS_PILOT_INPUT","SERVO10_FUNCTION","RC_FEEL_RP"
                ,"WPNAV_RFND_USE"
                ,"GPS_AUTO_CONFIG","GPS_AUTO_SWITCH","GPS_BLEND_MASK"
                ,"GPS_GNSS_MODE"
                ,"GPS_GNSS_MODE2","GPS_INJECT_TO","GPS_MIN_DGPS","GPS_MIN_ELEV"
                ,"GPS_NAVFILTER","MNT_DEFLT_MODE","MNT_JSTICK_SPD","MNT_RC_IN_PAN"
                ,"MNT_RC_IN_ROLL","MNT_RC_IN_TILT","MNT_STAB_PAN","MNT_STAB_ROLL"
                ,"MNT_STAB_TILT","MNT_TYPE","MOT_1_DIRECTION","MOT_2_DIRECTION","MOT_3_DIRECTION"
                ,"MOT_4_DIRECTION","MOT_5_DIRECTION","MOT_6_DIRECTION","MOT_7_DIRECTION","MOT_8_DIRECTION"
                ,"INS_ACCEL_FILTER","INS_ACC_BODYFIX","INS_FAST_SAMPLE","INS_GYRO_FILTER","INS_GYR_CAL"
                ,"INS_TRIM_OPTION","INS_USE","INS_USE2","INS_USE3"
                ,"EK2_ALT_SOURCE","EK2_BCN_DELAY","EK2_ENABLE"
                ,"EK2_FLOW_DELAY","EK2_GLITCH_RAD","EK2_GPS_CHECK"
                ,"EK2_GPS_TYPE","EK2_IMU_MASK","EK2_LOG_MASK","EK2_MAG_CAL"
                ,"EK2_MAG_MASK","EK2_RNG_USE_HGT","EK2_TAU_OUTPUT"
                ,"EK3_ENABLE"

                };
  Accel        ={"ACCEL_Z_D","ACCEL_Z_FILT","ACCEL_Z_I","ACCEL_Z_IMAX","ACCEL_Z_P"
                ,"ACRO_RP_P","ACRO_YAW_P"
                ,"BATT_AMP_OFFSET","BATT_AMP_PERVOLT" ,"BATT2_AMP_PERVOL" ,"BATT2_AMP_OFFSET"
                ,"VEL_Z_P" ,"SURFACE_DEPTH"
                ,"ControlA","ControlB","ControlC"
                ,"VEL_XY_FILT_HZ","VEL_XY_I","VEL_XY_IMAX","VEL_XY_P"
                ,"WPNAV_ACCEL","WPNAV_ACCEL_Z","WPNAV_LOIT_JERK","WPNAV_LOIT_MAXA","WPNAV_LOIT_MINA"
                ,"WPNAV_LOIT_SPEED","WPNAV_RADIUS","WPNAV_SPEED"
                ,"WPNAV_SPEED_DN","WPNAV_SPEED_UP"
                ,"GPS_POS1_X","GPS_POS1_Y","GPS_POS1_Z","GPS_POS2_X","GPS_POS2_Y","GPS_POS2_Z"
                ,"GPS_BLEND_TC"
                ,"INS_ACC2OFFS_X","INS_ACC2OFFS_Y","INS_ACC2OFFS_Z","INS_ACC2SCAL_X"
                ,"INS_ACC2SCAL_Y","INS_ACC2SCAL_Z","INS_ACC3OFFS_X","INS_ACC3OFFS_Y"
                ,"INS_ACC3OFFS_Z","INS_ACC3SCAL_X","INS_ACC3SCAL_Y","INS_ACC3SCAL_Z"
                ,"INS_ACCOFFS_X","INS_ACCOFFS_Y","INS_ACCOFFS_Z","INS_ACCSCAL_X"
                ,"INS_ACCSCAL_Y","INS_ACCSCAL_Z","INS_GYR2OFFS_X","INS_GYR2OFFS_Y"
                ,"INS_GYR2OFFS_Z","INS_GYR3OFFS_X","INS_GYR3OFFS_Y","INS_GYR3OFFS_Z"
                ,"INS_GYROFFS_X","INS_GYROFFS_Y","INS_GYROFFS_Z","INS_POS1_X","INS_POS1_Y"
                ,"INS_POS1_Z","INS_POS2_X","INS_POS2_Y","INS_POS2_Z","INS_POS3_X","INS_POS3_Y"
                ,"INS_POS3_Z","INS_STILL_THRESH"
                ,"EK2_ABIAS_P_NSE","EK2_ACC_P_NSE","EK2_ALT_M_NSE","EK2_BCN_M_NSE"
                ,"EK2_EAS_M_NSE","EK2_FLOW_M_NSE","EK2_GBIAS_P_NSE"
                ,"EK2_GBIAS_P_NSE","EK2_GYRO_P_NSE","EK2_MAGB_P_NSE"
                ,"EK2_MAGE_P_NSE","EK2_MAG_M_NSE","EK2_MAX_FLOW"
                ,"EK2_NOAID_M_NSE","EK2_POSNE_M_NSE","EK2_RNG_M_NSE"
                ,"EK2_RNG_M_NSE","EK2_TERR_GRAD","EK2_VELNE_M_NSE"
                ,"EK2_WIND_PSCALE","EK2_WIND_P_NSE","EK2_YAW_M_NSE"

                };

  ATC          ={"ATC_ANG_PIT_P","ATC_ANG_RLL_P","ATC_ANG_YAW_P","ATC_RAT_PIT_D","ATC_RAT_PIT_FF","ATC_RAT_PIT_FILT","ATC_RAT_PIT_I"
                ,"ATC_RAT_PIT_IMAX","ATC_RAT_PIT_P","ATC_RAT_RLL_D","ATC_RAT_RLL_FF"
                ,"ATC_RAT_RLL_FILT","ATC_RAT_RLL_I","ATC_RAT_RLL_IMAX","ATC_RAT_RLL_P"
                ,"ATC_RAT_YAW_D","ATC_RAT_YAW_FF","ATC_RAT_YAW_FILT","ATC_RAT_YAW_I"
                ,"ATC_RAT_YAW_IMAX","ATC_RAT_YAW_P"};

  ARMING      ={"ARMING_MIN_VOLT","ARMING_MIN_VOLT2"};
  ARMING_CHECK={"ARMING_CHECK"};
  ParamInt16  ={"GPS_DELAY_MS","GPS_DELAY_MS2"
               ,"MNT_ANGMAX_PAN","MNT_ANGMAX_ROL","MNT_ANGMAX_TIL","MNT_ANGMIN_PAN"
               ,"MNT_ANGMIN_ROL","MNT_ANGMIN_TIL"
               ,"EK2_BCN_I_GTE","EK2_CHECK_SCALE","EK2_EAS_I_GATE"
               ,"EK2_FLOW_I_GATE","EK2_GPS_DELAY","EK2_HGT_DELAY"
                ,"EK2_HGT_I_GATE","EK2_MAG_I_GATE","EK2_POS_I_GATE"
                ,"EK2_RNG_I_GATE","EK2_VEL_I_GATE","EK2_YAW_I_GATE"
               };
  DOITRONGparam ={"DOITRONG"};

  ros::NodeHandle nh;
  reqSetHeadingPID = nh.serviceClient<ParamSet>("parameter/set_heading_PID");
  reqGetHeadingPID = nh.serviceClient<ParamGet>("parameter/get_heading_PID");

  reqSetLOSParams = nh.serviceClient<ParamSet>("parameter/set_LOS_params");
  reqGetLOSParams = nh.serviceClient<ParamGet>("parameter/get_LOS_params");

  reqSetLightParam=nh.serviceClient<ParamSet>("parameter/set_Light_Param");
  reqGetLightParam=nh.serviceClient<ParamGet>("parameter/get_Light_Param");

  reqGetComPassParam=nh.serviceClient<ParamGet>("parameter/get_ComPass_param");

  reqGetBatteryParam=nh.serviceClient<ParamGet>("parameter/get_battery_param");

  reqGetFailSafeParam=nh.serviceClient<ParamGet>("parameter/get_failsafe_param");

  reqGetAccelParam=nh.serviceClient<ParamGet>("parameter/get_accel_param");

  reqGetATCParam=nh.serviceClient<ParamGet>("parameter/get_atc_param");

  reqGetArmingParam=nh.serviceClient<ParamGet>("parameter/get_arming_param");

  reqGetDOITRONGParam=nh.serviceClient<ParamGet>("parameter/get_doitrong_param");
  reqSetDOITRONGParam=nh.serviceClient<ParamSet>("parameter/set_doitrong_param");

}

void ParameterProtocol::handleParameter(const mavlink_message_t* msg)
{
  switch (msg->msgid)
  {
  case 21: // PARAM_REQUEST_LIST
    readAllParameters(msg);
    break;
  case 20: // PARAM_REQUEST_READ
    readParameter(msg);
    break;
  case 23: // PARAM_SET
    writeParameter(msg);
    break;
  }
}

void ParameterProtocol::readAllParameters(const mavlink_message_t* msg)
{
  PARAM_REQUEST_LIST reqPack;
  unpack_mavlink_message_t(msg, reqPack);

  /// Only procceed if the received message is aimed at this autopilot.
  if (check_for_target_id_matching(reqPack, trans->autopilot_sysid, trans->autopilot_compid))
  {
    ROS_INFO_STREAM(reqPack.to_yaml());

    uint16_t numParams = static_cast<uint16_t>(headingPID.size() + LOSParams.size()+FailSafe.size()+FailSafeF.size()
                                               +Light.size()+ComPass.size()+Battery.size()+FailSafe8.size()+Accel.size()
                                               +ATC.size()+ARMING.size()+ARMING_CHECK.size()+ParamInt16.size()
                                               +DOITRONGparam.size());
    uint16_t paramId = 0;

    readParamList(headingPID, numParams, paramId);
    readParamList(LOSParams, numParams, paramId);
    readParamList(Light,numParams, paramId);
    readParamList(ComPass,numParams, paramId);
    readParamList(Battery,numParams,paramId);
    readParamList(FailSafe,numParams,paramId);
    readParamList(FailSafeF,numParams,paramId);
    readParamList(FailSafe8,numParams,paramId);
    readParamList(Accel,numParams,paramId);
    readParamList(ATC,numParams,paramId);
    readParamList(ARMING,numParams,paramId);
    readParamList(ARMING_CHECK,numParams,paramId);
    readParamList(ParamInt16,numParams,paramId);

    readParamList(DOITRONGparam,numParams,paramId);
  }
}

void ParameterProtocol::readParameter(const mavlink_message_t* /*msg*/) {}

void ParameterProtocol::writeParameter(const mavlink_message_t* msg)
{
  PARAM_SET reqPack;
  unpack_mavlink_message_t(msg, reqPack);

  /// Only procceed if the received message is aimed at this autopilot.
  if (check_for_target_id_matching(reqPack, trans->autopilot_sysid, trans->autopilot_compid))
  {
    ROS_INFO_STREAM(reqPack.to_yaml());

    ParamSet srv;
    srv.request.param_id = reqPack.param_id.data();
    if (reqPack.param_type == static_cast<uint8_t>(MAV_PARAM_TYPE::INT32))
     /* memcpy(&srv.request.value.integer, &reqPack.param_value, sizeof(float));*/
      srv.request.value.integer=static_cast<int>(reqPack.param_value);
    else if (reqPack.param_type == static_cast<uint8_t>(MAV_PARAM_TYPE::REAL32))
      srv.request.value.real = static_cast<double>(reqPack.param_value);

    if (requestSetParameter(srv))
    {
      PARAM_VALUE resPack;
      resPack.param_id = reqPack.param_id;
      if (reqPack.param_type == static_cast<uint8_t>(MAV_PARAM_TYPE::INT32))
        /*memcpy(&resPack.param_value, &srv.response.value.integer, sizeof(float));*/
        resPack.param_value=static_cast<int>(srv.response.value.integer);
      else if (reqPack.param_type == static_cast<uint8_t>(MAV_PARAM_TYPE::REAL32))
        resPack.param_value = static_cast<float>(srv.response.value.real);
      resPack.param_type = reqPack.param_type;
      pack_and_send_mavlink_message_t(resPack, link);
    }
  }
}

void ParameterProtocol::readParamList(const StringList& paramList, const uint16_t& numParams, uint16_t& paramId)
{
  ParamGet srv;
  PARAM_VALUE resPack;

  for (auto it = paramList.begin(); it != paramList.end(); it++)
  {
    srv.request.param_id = it->data();

    if (requestGetParameter(srv, paramList))
    {
      strcpy(resPack.param_id.data(), it->data());

      /// Assuming that all parameters in headingPID and LOSParams is real.
      /// This piece of code needs more refinement in the future.
      if (paramList == headingPID || paramList == LOSParams|| paramList == FailSafeF
          || paramList == Accel||paramList == ATC || paramList == ARMING || paramList == DOITRONGparam)
      {
        resPack.param_value = static_cast<float>(srv.response.value.real);
        resPack.param_type = static_cast<uint8_t>(MAV_PARAM_TYPE::REAL32);
      }
      else if (paramList==ComPass ||  paramList==Battery || paramList==FailSafe
               || paramList == ARMING_CHECK || paramList == Light)
      {
        resPack.param_value=static_cast<int>(srv.response.value.integer);
        resPack.param_type=static_cast<uint8_t>(MAV_PARAM_TYPE::INT32);
      }

      else if ( paramList==FailSafe8)
      {
        resPack.param_value=static_cast<int>(srv.response.value.integer);
        resPack.param_type=static_cast<uint8_t>(MAV_PARAM_TYPE::INT8);
      }
      else if ( paramList == ParamInt16)
      {
        resPack.param_value = static_cast<int>(srv.response.value.integer);
        resPack.param_type = static_cast<uint8_t>(MAV_PARAM_TYPE::INT16);
      }


      /// The total number of parameters in the system must be assigned to every
      /// responding message back to GCS.
      resPack.param_count = numParams;
      resPack.param_index = paramId++;
      pack_and_send_mavlink_message_t(resPack, link);
    }
  }
}

bool ParameterProtocol::requestSetParameter(ParamSet& srv)
{
  if (paramListMatched(headingPID, srv.request.param_id.data()))
    return reqSetHeadingPID.call(srv);
  if (paramListMatched(LOSParams, srv.request.param_id.data()))
    return reqSetLOSParams.call(srv);
  if (paramListMatched(Light, srv.request.param_id.data()))
    return reqSetLightParam.call(srv);
  if (paramListMatched(DOITRONGparam,srv.request.param_id.data()))
    return reqSetDOITRONGParam.call(srv);
  return false;
}

bool ParameterProtocol::requestGetParameter(ParamGet& srv, const StringList& paramList)
{
  if (paramList == headingPID)
    return reqGetHeadingPID.call(srv);
  if (paramList == LOSParams)
    return reqGetLOSParams.call(srv);
  if (paramList == Light)
    return reqGetLightParam.call(srv);
  if(paramList == ComPass)
    return reqGetComPassParam.call(srv);
  if (paramList == Battery)
    return reqGetBatteryParam.call(srv);
  if (paramList == FailSafe || paramList == FailSafeF||paramList == FailSafe8 || paramList == ParamInt16)
    return reqGetFailSafeParam.call(srv);
  if (paramList == Accel)
    return reqGetAccelParam.call(srv);
  if(paramList == ATC)
    return reqGetATCParam.call(srv);
  if(paramList ==ARMING || paramList ==ARMING_CHECK )
    return reqGetArmingParam.call(srv);
  if (paramList == DOITRONGparam)
    return reqGetDOITRONGParam.call(srv);
  return false;
}
