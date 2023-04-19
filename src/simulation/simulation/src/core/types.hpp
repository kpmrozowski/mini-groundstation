#ifndef Types_HPP
#define Types_HPP

#include "dji_telemetry.hpp"

namespace Types {


using DJI::OSDK::Telemetry::TypeMap;
using namespace DJI::OSDK::Telemetry;

typedef TypeMap<TOPIC_QUATERNION              >::type Quaternion;
typedef TypeMap<TOPIC_ACCELERATION_GROUND     >::type AccelerationGround;
typedef TypeMap<TOPIC_ACCELERATION_BODY       >::type AccelerationBody;
typedef TypeMap<TOPIC_ACCELERATION_RAW        >::type AccelerationRaw;
typedef TypeMap<TOPIC_VELOCITY                >::type Velocity;
typedef TypeMap<TOPIC_ANGULAR_RATE_FUSIONED   >::type AngularRateFusioned;
typedef TypeMap<TOPIC_ANGULAR_RATE_RAW        >::type AngularRateRaw;
typedef TypeMap<TOPIC_ALTITUDE_FUSIONED       >::type AltitudeFusioned;
typedef TypeMap<TOPIC_ALTITUDE_BAROMETER      >::type AltitudeBarometer;
typedef TypeMap<TOPIC_ALTITUDE_OF_HOMEPOINT   >::type AltitudeOfHomepoint;
typedef TypeMap<TOPIC_HEIGHT_FUSION           >::type HeightFusioned;
typedef TypeMap<TOPIC_GPS_FUSED               >::type LatLonFused;
typedef TypeMap<TOPIC_GPS_DATE                >::type GpsDate;
typedef TypeMap<TOPIC_GPS_TIME                >::type GpsTime;
typedef TypeMap<TOPIC_GPS_POSITION            >::type GpsPosition;
typedef TypeMap<TOPIC_GPS_VELOCITY            >::type GpsVelocity;
typedef TypeMap<TOPIC_GPS_DETAILS             >::type GpsDetails;
typedef TypeMap<TOPIC_RTK_POSITION            >::type RtkPosition;
typedef TypeMap<TOPIC_RTK_VELOCITY            >::type RtkVelocity;
typedef TypeMap<TOPIC_RTK_YAW                 >::type RtkYaw;
typedef TypeMap<TOPIC_RTK_POSITION_INFO       >::type RtkPositionInfo;
typedef TypeMap<TOPIC_RTK_YAW_INFO            >::type RtkYawInfo;
typedef TypeMap<TOPIC_COMPASS                 >::type Compass;
typedef TypeMap<TOPIC_RC                      >::type Rc;
typedef TypeMap<TOPIC_GIMBAL_ANGLES           >::type GimbalAngles;
typedef TypeMap<TOPIC_GIMBAL_STATUS           >::type GimbalStatus;
typedef TypeMap<TOPIC_STATUS_FLIGHT           >::type StatusFlight;
typedef TypeMap<TOPIC_STATUS_DISPLAYMODE      >::type StatusDisplayMode;
typedef TypeMap<TOPIC_STATUS_LANDINGGEAR      >::type StatusLandingGear;
typedef TypeMap<TOPIC_STATUS_MOTOR_START_ERROR>::type StatusMotorStartError;
typedef TypeMap<TOPIC_BATTERY_INFO            >::type BatteryInfo;
typedef TypeMap<TOPIC_CONTROL_DEVICE          >::type ControlDevice;
typedef TypeMap<TOPIC_HARD_SYNC               >::type HardSync;
typedef TypeMap<TOPIC_GPS_SIGNAL_LEVEL        >::type GpsSignalLevel;
typedef TypeMap<TOPIC_GPS_CONTROL_LEVEL       >::type GpsControlLevel;
typedef TypeMap<TOPIC_RC_FULL_RAW_DATA        >::type RcFullRawData;
typedef TypeMap<TOPIC_RC_WITH_FLAG_DATA       >::type RcWithFlagData;
typedef TypeMap<TOPIC_ESC_DATA                >::type EscData;
typedef TypeMap<TOPIC_RTK_CONNECT_STATUS      >::type StatusRtkConnect;
typedef TypeMap<TOPIC_GIMBAL_CONTROL_MODE     >::type GimbalControlMode;
typedef TypeMap<TOPIC_FLIGHT_ANOMALY          >::type FlightAnomaly;
typedef TypeMap<TOPIC_POSITION_VO             >::type PositionVO;
typedef TypeMap<TOPIC_AVOID_DATA              >::type AvoidData;
typedef TypeMap<TOPIC_HOME_POINT_SET_STATUS   >::type StatusHomePointSet;
typedef TypeMap<TOPIC_HOME_POINT_INFO         >::type HomePointInfo;
typedef TypeMap<TOPIC_DUAL_GIMBAL_DATA        >::type DualGimbalData;
typedef TypeMap<TOPIC_THREE_GIMBAL_DATA       >::type ThreeGimbalData;

} // namespace Types

#endif // Types_HPP
