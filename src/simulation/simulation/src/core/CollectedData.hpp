#ifndef CollectedData_HPP
#define CollectedData_HPP

#include "types.hpp"

#include "dji_battery.hpp"      // SmartBatteryDynamicInfo
#include "dji_camera_image.hpp" // CameraRGBImage
#include "osdk_platform.h"      // T_OsdkMutexHandle
#include "dji_perception.hpp"   // ImageInfoType
#include "dji_telemetry.hpp"    // TypeMap
#include <dji_vehicle_callback.hpp>
#include <string>

using DJI::OSDK::Perception;
using DJI::OSDK::Telemetry::TypeMap;
using DJI::OSDK::Telemetry::TopicName;

typedef struct StereoImagePacketType {
    Perception::ImageInfoType info;
    uint8_t *imageRawBuffer;
    T_OsdkMutexHandle mutex;
    bool gotData;
} StereoImagePacketType;

struct CollectedData { // 62 params, 2416 bytes
    // battery data, 96 bytes
    DJI::OSDK::BatteryWholeInfo batteryWholeInfo;
    DJI::OSDK::SmartBatteryDynamicInfo firstBatteryDynamicInfo;
    DJI::OSDK::SmartBatteryDynamicInfo secondBatteryDynamicInfo;

    // images data, 400 bytes
    CameraRGBImage rgbImageMain;
    CameraRGBImage rgbImageFpv;
    StereoImagePacketType stereoImagePacketFront;
    StereoImagePacketType stereoImagePacketRear;
    StereoImagePacketType stereoImagePacketRight;
    StereoImagePacketType stereoImagePacketLeft;
    StereoImagePacketType stereoImagePacketUp;
    StereoImagePacketType stereoImagePacketBottom;

    // telemetry data, 636 bytes
    TypeMap<DJI::OSDK::Telemetry::TOPIC_QUATERNION              >::type quaternion;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_ACCELERATION_GROUND     >::type accelerationGround;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_ACCELERATION_BODY       >::type accelerationBody;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_ACCELERATION_RAW        >::type accelerationRaw;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_VELOCITY                >::type velocity;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_ANGULAR_RATE_FUSIONED   >::type angularRateFusioned;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_ANGULAR_RATE_RAW        >::type angularRateRaw;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED       >::type altitudeFusioned;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_BAROMETER      >::type altitudeBarometer;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_OF_HOMEPOINT   >::type altitudeOfHomepoint;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_HEIGHT_FUSION           >::type heightFusioned;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED               >::type latLonFused;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_DATE                >::type gpsDate;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_TIME                >::type gpsTime;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_POSITION            >::type gpsPosition;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_VELOCITY            >::type gpsVelocity;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_DETAILS             >::type gpsDetails;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_POSITION            >::type rtkPosition;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_VELOCITY            >::type rtkVelocity;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_YAW                 >::type rtkYaw;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_POSITION_INFO       >::type rtkPositionInfo;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_YAW_INFO            >::type rtkYawInfo;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_COMPASS                 >::type compass;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_RC                      >::type rc;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_GIMBAL_ANGLES           >::type gimbalAngles;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_GIMBAL_STATUS           >::type gimbalStatus;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT           >::type statusFlight;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE      >::type statusDisplayMode;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_STATUS_LANDINGGEAR      >::type statusLandingGear;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_STATUS_MOTOR_START_ERROR>::type statusMotorStartError;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO            >::type batteryInfo;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE          >::type controlDevice;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_HARD_SYNC               >::type hardSync;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_SIGNAL_LEVEL        >::type gpsSignalLevel;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_GPS_CONTROL_LEVEL       >::type gpsControlLevel;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_RC_FULL_RAW_DATA        >::type rcFullRawData;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_RC_WITH_FLAG_DATA       >::type rcWithFlagData;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_ESC_DATA                >::type escData;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_RTK_CONNECT_STATUS      >::type statusRtkConnect;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_GIMBAL_CONTROL_MODE     >::type gimbalControlMode;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_FLIGHT_ANOMALY          >::type flightAnomaly;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_POSITION_VO             >::type positionVO;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_AVOID_DATA              >::type avoidData;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_HOME_POINT_SET_STATUS   >::type statusHomePointSet;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_HOME_POINT_INFO         >::type homePointInfo;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_DUAL_GIMBAL_DATA        >::type dualGimbalData;
    TypeMap<DJI::OSDK::Telemetry::TOPIC_THREE_GIMBAL_DATA       >::type threeGimbalData;

    // time data, 1280
    DJI::OSDK::RecvContainer time_nmea;
    DJI::OSDK::RecvContainer time_utc;
    DJI::OSDK::RecvContainer time_fct; // Flight controller time in UTC reference
    DJI::OSDK::RecvContainer time_pps; // { 0: "0", 1: "INTERNAL_GPS", 2: "EXTERNAL_GPS", 3: "RTK" }
    std::string timestamp;

    bool operator==(const CollectedData& rhs) const;
};

#endif // CollectedData_HPP
