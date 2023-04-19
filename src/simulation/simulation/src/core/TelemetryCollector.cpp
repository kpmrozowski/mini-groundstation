#include "TelemetryCollector.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "CsvWriter.hpp"
#include "dji_vehicle.hpp"
#include "dji_telemetry.hpp"

#include "utils.hpp"

using DJI::OSDK::ACK;
using DJI::OSDK::Telemetry::TopicName;

TelemetryCollector::TelemetryCollector(const Constants& consts, DJI::OSDK::Vehicle* vehicle)
 : p_vehicle(vehicle)
{
    // TODO: create dir if does not exist
    if (not consts.subscribeToTelemetry()) {
        printf("subscribeToTelemetry is false, TelemetryCollector will not run.\n");
        return;
    }
    printf("Subscribing to Telemetry.\n");
    if (nullptr != p_vehicle) {
        subscribe();
    } else {
        printf("p_vehicle is nullptr, to continue change dryRun to false.\n");
    }
}

void TelemetryCollector::subscribe() {
    if (trySubscribe()) {
        throw std::runtime_error("TelemetryCollector could have not subscribe to data.");
    }
    printf("Successfully subscribed to telemetry.\n");
    // m_collectTelemetryDataThread = std::thread(&TelemetryCollector::collectTelemetryData, this);
}

void TelemetryCollector::unsubscribe() {
    m_terminateCollector = true;
    m_collectTelemetryDataThread.join();
}

// ----------------------------------------------------------------------
void TelemetryCollector::callback400Hz(Vehicle* vehicle, RecvContainer recvFrame, UserData userData)
{
    printf("callback400Hz\n");
    TelemetryCollector* self = (TelemetryCollector*)userData;
    self->s_data.timestamp = utils::time_in_YYYYMMDDHHMMSSMMM(6);
    self->s_data.hardSync        = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_HARD_SYNC>();        //400 Hz, 5
    self->s_data.accelerationRaw = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_RAW>(); //400 Hz, 5
    self->s_data.angularRateRaw  = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_ANGULAR_RATE_RAW>(); //400 Hz, 5
    s_updatedTelemetry.at(5) = true;
    if (nullptr != self->mp_csvWriter) {
        self->mp_csvWriter->append();
    }
}

// ----------------------------------------------------------------------
void TelemetryCollector::callback200Hz(Vehicle* vehicle, RecvContainer recvFrame, UserData userData)
{
    printf("callback200Hz\n");
    TelemetryCollector* self = (TelemetryCollector*)userData;
    self->s_data.timestamp = utils::time_in_YYYYMMDDHHMMSSMMM(6);
    self->s_data.altitudeFusioned    = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_ALTITUDE_FUSIONED>();    // 200 Hz, 4
    self->s_data.altitudeBarometer   = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_ALTITUDE_BAROMETER>();   // 200 Hz, 4
    self->s_data.velocity            = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_VELOCITY>();             // 200 Hz, 4
    self->s_data.accelerationGround  = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_GROUND>();  // 200 Hz, 4
    self->s_data.accelerationBody    = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_BODY>();    // 200 Hz, 4
    self->s_data.angularRateFusioned = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>();// 200 Hz, 4
    s_updatedTelemetry.at(4) = true;
    if (nullptr != self->mp_csvWriter) {
        self->mp_csvWriter->append();
    }
}

// ----------------------------------------------------------------------
void TelemetryCollector::callback100Hz(Vehicle* vehicle, RecvContainer recvFrame, UserData userData)
{
    printf("callback100Hz\n");
    TelemetryCollector* self = (TelemetryCollector*)userData;
    self->s_data.timestamp = utils::time_in_YYYYMMDDHHMMSSMMM(6);
    self->s_data.compass        = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_COMPASS>();       // 100 Hz, 3
    self->s_data.heightFusioned = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>(); // 100 Hz, 3
    self->s_data.positionVO     = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_POSITION_VO>();   // 100 Hz, 3
    self->s_data.avoidData      = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_AVOID_DATA>();    // 100 Hz, 3
    s_updatedTelemetry.at(3) = true;
    if (nullptr != self->mp_csvWriter) {
        self->mp_csvWriter->append();
    }
}

// ----------------------------------------------------------------------
void TelemetryCollector::callback50Hz(Vehicle* vehicle, RecvContainer recvFrame, UserData userData)
{
    printf("callback50Hz\n");
    TelemetryCollector* self = (TelemetryCollector*)userData;
    self->s_data.timestamp = utils::time_in_YYYYMMDDHHMMSSMMM(6);
    self->s_data.gimbalStatus          = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_GIMBAL_STATUS>();          //50 Hz, 2
    self->s_data.statusRtkConnect      = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_CONNECT_STATUS>();     //50 Hz, 2
    self->s_data.statusMotorStartError = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_MOTOR_START_ERROR>();//50Hz, 2
    self->s_data.gpsSignalLevel        = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_SIGNAL_LEVEL>();       //50 Hz, 2
    self->s_data.statusLandingGear     = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_LANDINGGEAR>();     //50 Hz, 2
    self->s_data.statusDisplayMode     = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();     //50 Hz, 2
    self->s_data.statusFlight          = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();          //50 Hz, 2
    self->s_data.batteryInfo           = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_BATTERY_INFO>();          // 50 Hz, 2
    self->s_data.gpsControlLevel       = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_CONTROL_LEVEL>();     // 50 Hz, 2
    self->s_data.latLonFused           = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();             // 50 Hz, 2
    self->s_data.gimbalAngles          = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>();         // 50 Hz, 2
    self->s_data.gimbalControlMode     = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_GIMBAL_CONTROL_MODE>();   // 50 Hz, 2
    self->s_data.flightAnomaly         = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_FLIGHT_ANOMALY>();        // 50 Hz, 2
    self->s_data.rc                    = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_RC>();                    // 50 Hz, 2
    self->s_data.quaternion            = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_QUATERNION>();            // 50 Hz, 2
    s_updatedTelemetry.at(2) = true;
    if (nullptr != self->mp_csvWriter) {
        self->mp_csvWriter->append();
    }
}

// ----------------------------------------------------------------------
void TelemetryCollector::callback5Hz(Vehicle* vehicle, RecvContainer recvFrame, UserData userData)
{
    printf("callback5Hz\n");
    TelemetryCollector* self = (TelemetryCollector*)userData;
    self->s_data.timestamp = utils::time_in_YYYYMMDDHHMMSSMMM(6);
    s_data.controlDevice       = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_CONTROL_DEVICE>();         // 5 Hz, 1
    s_data.statusHomePointSet  = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_HOME_POINT_SET_STATUS>();  // 5 Hz, 1
    s_data.homePointInfo       = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_HOME_POINT_INFO>();        // 5 Hz, 1
    s_data.gpsTime             = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_TIME>();               // 5 Hz, 1
    s_data.gpsDate             = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_DATE>();               // 5 Hz, 1
    s_data.gpsPosition         = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_POSITION>();           // 5 Hz, 1
    s_data.gpsVelocity         = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_VELOCITY>();           // 5 Hz, 1
    s_data.gpsDetails          = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_DETAILS>();            // 5 Hz, 1
    if(self->m_rtkAvailable) {
        s_data.rtkPosition     = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_POSITION>();           // 5 Hz, 1
        s_data.rtkVelocity     = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_VELOCITY>();           // 5 Hz, 1
        s_data.rtkYaw          = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_YAW>();                // 5 Hz, 1
        s_data.rtkPositionInfo = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_POSITION_INFO>();      // 5 Hz, 1
        s_data.rtkYawInfo      = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_YAW_INFO>();           // 5 Hz, 1

        std::cout << "RTK if available   (lat/long/alt/velocity_x/velocity_y/velocity_z/yaw/yaw_info/pos_info) ="
                << s_data.rtkPosition.latitude << "," << s_data.rtkPosition.longitude << "," << s_data.rtkPosition.HFSL << ","
                << s_data.rtkVelocity.x << ","<< s_data.rtkVelocity.y<< "," << s_data.rtkVelocity.z
                << "," << s_data.rtkYaw << "," << (uint16_t)s_data.rtkYawInfo <<","<< (uint16_t)s_data.rtkPositionInfo<< "\n";
    } // TODO: RTK does not work indoor
    s_updatedTelemetry.at(1) = true;
    if (nullptr != self->mp_csvWriter) {
        self->mp_csvWriter->append();
    }
}

// ----------------------------------------------------------------------
void TelemetryCollector::callback1Hz(Vehicle* vehicle, RecvContainer recvFrame, UserData userData)
{
    TelemetryCollector* self = (TelemetryCollector*)userData;
    self->s_data.timestamp = utils::time_in_YYYYMMDDHHMMSSMMM(6);
    self->s_data.altitudeOfHomepoint = self->p_vehicle->subscribe->getValue<Telemetry::TOPIC_ALTITUDE_OF_HOMEPOINT>();  // 1 Hz, 0
    s_updatedTelemetry.at(0) = true;
    if (nullptr != self->mp_csvWriter) {
        self->mp_csvWriter->append();
    }
}

void TelemetryCollector::collectTelemetryData() {
    while (not m_terminateCollector){
        printf("telemetry callback\n");
        s_data.hardSync              = p_vehicle->subscribe->getValue<Telemetry::TOPIC_HARD_SYNC>();             //400 Hz, 5
        s_data.accelerationRaw       = p_vehicle->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_RAW>();      //400 Hz, 5
        s_data.angularRateRaw        = p_vehicle->subscribe->getValue<Telemetry::TOPIC_ANGULAR_RATE_RAW>();      //400 Hz, 5

        s_data.altitudeFusioned      = p_vehicle->subscribe->getValue<Telemetry::TOPIC_ALTITUDE_FUSIONED>();    // 200 Hz, 4
        s_data.altitudeBarometer     = p_vehicle->subscribe->getValue<Telemetry::TOPIC_ALTITUDE_BAROMETER>();   // 200 Hz, 4
        s_data.velocity              = p_vehicle->subscribe->getValue<Telemetry::TOPIC_VELOCITY>();             // 200 Hz, 4
        s_data.accelerationGround    = p_vehicle->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_GROUND>();  // 200 Hz, 4
        s_data.accelerationBody      = p_vehicle->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_BODY>();    // 200 Hz, 4
        s_data.angularRateFusioned   = p_vehicle->subscribe->getValue<Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>();// 200 Hz, 4

        s_data.compass               = p_vehicle->subscribe->getValue<Telemetry::TOPIC_COMPASS>();               // 100 Hz, 3
        s_data.heightFusioned        = p_vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();         // 100 Hz, 3
        s_data.positionVO            = p_vehicle->subscribe->getValue<Telemetry::TOPIC_POSITION_VO>();           // 100 Hz, 3
        s_data.avoidData             = p_vehicle->subscribe->getValue<Telemetry::TOPIC_AVOID_DATA>();            // 100 Hz, 3

        s_data.gimbalStatus          = p_vehicle->subscribe->getValue<Telemetry::TOPIC_GIMBAL_STATUS>();          //50 Hz, 2
        s_data.statusRtkConnect      = p_vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_CONNECT_STATUS>();     //50 Hz, 2
        s_data.statusMotorStartError = p_vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_MOTOR_START_ERROR>();//50Hz, 2
        s_data.gpsSignalLevel        = p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_SIGNAL_LEVEL>();       //50 Hz, 2
        s_data.statusLandingGear     = p_vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_LANDINGGEAR>();     //50 Hz, 2
        s_data.statusDisplayMode     = p_vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();     //50 Hz, 2
        s_data.statusFlight          = p_vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();          //50 Hz, 2
        s_data.batteryInfo           = p_vehicle->subscribe->getValue<Telemetry::TOPIC_BATTERY_INFO>();          // 50 Hz, 2
        s_data.gpsControlLevel       = p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_CONTROL_LEVEL>();     // 50 Hz, 2
        s_data.latLonFused           = p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();             // 50 Hz, 2
        s_data.gimbalAngles          = p_vehicle->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>();         // 50 Hz, 2
        s_data.gimbalControlMode     = p_vehicle->subscribe->getValue<Telemetry::TOPIC_GIMBAL_CONTROL_MODE>();   // 50 Hz, 2
        s_data.flightAnomaly         = p_vehicle->subscribe->getValue<Telemetry::TOPIC_FLIGHT_ANOMALY>();        // 50 Hz, 2
        s_data.rc                    = p_vehicle->subscribe->getValue<Telemetry::TOPIC_RC>();                    // 50 Hz, 2
        s_data.quaternion            = p_vehicle->subscribe->getValue<Telemetry::TOPIC_QUATERNION>();            // 50 Hz, 2

        s_data.controlDevice         = p_vehicle->subscribe->getValue<Telemetry::TOPIC_CONTROL_DEVICE>();         // 5 Hz, 1
        s_data.statusHomePointSet    = p_vehicle->subscribe->getValue<Telemetry::TOPIC_HOME_POINT_SET_STATUS>();  // 5 Hz, 1
        s_data.homePointInfo         = p_vehicle->subscribe->getValue<Telemetry::TOPIC_HOME_POINT_INFO>();        // 5 Hz, 1
        s_data.gpsTime               = p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_TIME>();               // 5 Hz, 1
        s_data.gpsDate               = p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_DATE>();               // 5 Hz, 1
        s_data.gpsPosition           = p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_POSITION>();           // 5 Hz, 1
        s_data.gpsVelocity           = p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_VELOCITY>();           // 5 Hz, 1
        s_data.gpsDetails            = p_vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_DETAILS>();            // 5 Hz, 1
        if(m_rtkAvailable) {
            s_data.rtkPosition       = p_vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_POSITION>();           // 5 Hz, 1
            s_data.rtkVelocity       = p_vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_VELOCITY>();           // 5 Hz, 1
            s_data.rtkYaw            = p_vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_YAW>();                // 5 Hz, 1
            s_data.rtkPositionInfo   = p_vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_POSITION_INFO>();      // 5 Hz, 1
            s_data.rtkYawInfo        = p_vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_YAW_INFO>();           // 5 Hz, 1
        }

        s_data.altitudeOfHomepoint   = p_vehicle->subscribe->getValue<Telemetry::TOPIC_ALTITUDE_OF_HOMEPOINT>();  // 1 Hz, 0
        // rcFullRawData         = p_vehicle->subscribe->getValue<Telemetry::TOPIC_RC_FULL_RAW_DATA>();
        // rcWithFlagData        = p_vehicle->subscribe->getValue<Telemetry::TOPIC_RC_WITH_FLAG_DATA>();
        // escData               = p_vehicle->subscribe->getValue<Telemetry::TOPIC_ESC_DATA>();
        // dualGimbalData        = p_vehicle->subscribe->getValue<Telemetry::TOPIC_DUAL_GIMBAL_DATA>();
        // threeGimbalData       = p_vehicle->subscribe->getValue<Telemetry::TOPIC_THREE_GIMBAL_DATA>();

        std::cout << "-------\n";
        std::cout << "Flight Status                         = " << (int)s_data.statusFlight
                  << "\n";
        std::cout << "Position              (LLA)           = " << s_data.latLonFused.latitude
                  << ", " << s_data.latLonFused.longitude << ", " << s_data.altitudeFusioned << "\n";
        std::cout << "RC Commands           (r/p/y/thr)     = " << s_data.rc.roll << ", "
                  << s_data.rc.pitch << ", " << s_data.rc.yaw << ", " << s_data.rc.throttle << "\n";
        std::cout << "Velocity              (vx,vy,vz)      = " << s_data.velocity.data.x
                  << ", " << s_data.velocity.data.y << ", " << s_data.velocity.data.z << "\n";
        std::cout << "Attitude Quaternion   (w,x,y,z)       = " << s_data.quaternion.q0
                  << ", " << s_data.quaternion.q1 << ", " << s_data.quaternion.q2 << ", "
                  << s_data.quaternion.q3 << "\n";
        if(m_rtkAvailable) {
          std::cout << "RTK if available   (lat/long/alt/velocity_x/velocity_y/velocity_z/yaw/yaw_info/pos_info) ="
                    << s_data.rtkPosition.latitude << "," << s_data.rtkPosition.longitude << "," << s_data.rtkPosition.HFSL << ","
                    << s_data.rtkVelocity.x << ","<< s_data.rtkVelocity.y<< "," << s_data.rtkVelocity.z
                    << "," << s_data.rtkYaw << "," << (uint16_t)s_data.rtkYawInfo <<","<< (uint16_t)s_data.rtkPositionInfo<< "\n";
        } // TODO: RTK does not work indoor
        std::cout << "-------\n\n";
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }
}

// bool HAL::subscribeToTopic(Topics _topics, int _freq)
// {
//     ACK::ErrorCode subscribeStatus = vehicle_->subscribe->verify(functionTimeout_);
//     if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
//     {
//         ACK::getErrorCodeMessage(subscribeStatus, __func__);
//         std::cout << "\033[31mNot verified subscription to Telemetry, exiting \033[m" << std::endl;
//         return false;
//     }

//     TopicName topicList[_topics.size()];   
//     for (unsigned int i = 0; i < _topics.size(); i++ )
//     {
//         topicList[i] = _topics[i];
//     }

//     int numTopic = sizeof(topicList) / sizeof(topicList[0]);
//     bool enableTimestamp = false;

//     bool pkgStatus = vehicle_->subscribe->initPackageFromTopicList(pkgIndex_, numTopic, topicList, enableTimestamp, _freq);
//     if (!pkgStatus)
//     {
//         std::cout << "\033[31mNot init package \033[m" << pkgIndex_ << "\033[31m from topic list, exiting \033[m" << std::endl;
//         return false;
//     }

//     subscribeStatus = vehicle_->subscribe->startPackage(pkgIndex_, functionTimeout_);
//     if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
//     {
//         ACK::getErrorCodeMessage(subscribeStatus, __func__);
//         unsubscribeAllTopics(); 
//         std::cout << "\033[31mStart package \033[m" << pkgIndex_ << "\033[31m error, exiting \033[m" << std::endl;
//         return false;
//     }

//     pkgIndex_++;
    
//     return true;
// }

bool TelemetryCollector::trySubscribe() {
    printf("Subscribing to telemetry.\n");
    // We will subscribe to six kinds of data:
    // Package 0: Altitude at 1 Hz
    // Package 1: GPS Fused and Battery at 10 Hz
    // Package 2: GPS and Statuses at 5 Hz
    // Package 3: Acceleration, Velocity, PositionVO, RC, Compass at 50 Hz
    // Package 4: Hardware Sync 400 Hz
    // Package 5: RTK at 5 Hz

    // Please make sure your drone is in simulation mode. You can fly the drone
    // with your RC to get different values.

    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = p_vehicle->subscribe->verify(m_responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        return EXIT_FAILURE;
    }

    // Package 0
    int       pkgIndex        = 0;
    int       freq            = 1;
    m_topicList1Hz = {
        Telemetry::TOPIC_ALTITUDE_OF_HOMEPOINT,
    };
    int       numTopic        = sizeof(m_topicList1Hz) / sizeof(m_topicList1Hz[0]);
    bool      enableTimestamp = false;

    p_vehicle->subscribe->registerUserPackageUnpackCallback(pkgIndex, TelemetryCollector::callback1Hz, this);
    bool pkgStatus = p_vehicle->subscribe->initPackageFromTopicList(
        pkgIndex, numTopic, m_topicList1Hz.data(), enableTimestamp, freq);
    if (!(pkgStatus))
    {
        printf("could not init package %d.\n", pkgIndex);
        return EXIT_FAILURE;
    }
    subscribeStatus = p_vehicle->subscribe->startPackage(pkgIndex, m_responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        printf("could not start package %d.\n", pkgIndex);
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup before return
        p_vehicle->subscribe->removePackage(pkgIndex, m_responseTimeout);
        return EXIT_FAILURE;
    }

    // Package 1
    pkgIndex                  = 1;
    freq                      = 5;
    m_topicList5Hz = {
        Telemetry::TOPIC_HOME_POINT_SET_STATUS,
        Telemetry::TOPIC_CONTROL_DEVICE,
        Telemetry::TOPIC_HOME_POINT_INFO,
        Telemetry::TOPIC_GPS_TIME,
        Telemetry::TOPIC_GPS_DATE,
        Telemetry::TOPIC_GPS_POSITION,
        Telemetry::TOPIC_GPS_VELOCITY,
        Telemetry::TOPIC_GPS_DETAILS,
        Telemetry::TOPIC_RTK_POSITION,
        Telemetry::TOPIC_RTK_YAW_INFO,
        Telemetry::TOPIC_RTK_POSITION_INFO,
        Telemetry::TOPIC_RTK_VELOCITY,
        Telemetry::TOPIC_RTK_YAW,
    };
    numTopic                  = sizeof(m_topicList5Hz) / sizeof(m_topicList5Hz[0]);
    enableTimestamp           = false;

    p_vehicle->subscribe->registerUserPackageUnpackCallback(pkgIndex, TelemetryCollector::callback5Hz, this);
    pkgStatus = p_vehicle->subscribe->initPackageFromTopicList(
        pkgIndex, numTopic, m_topicList5Hz.data(), enableTimestamp, freq);
    if (!(pkgStatus))
    {
        printf("could not init package %d.\n", pkgIndex);
        return EXIT_FAILURE;
    }
    subscribeStatus = p_vehicle->subscribe->startPackage(pkgIndex, m_responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        printf("could not start package %d.\n", pkgIndex);
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup before return
        p_vehicle->subscribe->removePackage(pkgIndex, m_responseTimeout);
        return EXIT_FAILURE;
    }

    // Package 2
    pkgIndex                   = 2;
    freq                       = 50;
    m_topicList50Hz = {
        Telemetry::TOPIC_GIMBAL_STATUS,
        Telemetry::TOPIC_RTK_CONNECT_STATUS,
        Telemetry::TOPIC_STATUS_MOTOR_START_ERROR,
        Telemetry::TOPIC_GPS_SIGNAL_LEVEL,
        Telemetry::TOPIC_STATUS_LANDINGGEAR,
        Telemetry::TOPIC_STATUS_DISPLAYMODE,
        Telemetry::TOPIC_STATUS_FLIGHT,
        Telemetry::TOPIC_BATTERY_INFO,
        Telemetry::TOPIC_GPS_CONTROL_LEVEL,
        Telemetry::TOPIC_GPS_FUSED,
        Telemetry::TOPIC_GIMBAL_ANGLES,
        Telemetry::TOPIC_GIMBAL_CONTROL_MODE,
        Telemetry::TOPIC_FLIGHT_ANOMALY,
        Telemetry::TOPIC_RC,
        Telemetry::TOPIC_QUATERNION,
    };
    numTopic        = sizeof(m_topicList50Hz) / sizeof(m_topicList50Hz[0]);
    enableTimestamp = false;

    p_vehicle->subscribe->registerUserPackageUnpackCallback(pkgIndex, TelemetryCollector::callback50Hz, this);
    pkgStatus = p_vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, m_topicList50Hz.data(), enableTimestamp, freq);
    if (!(pkgStatus))
    {
        printf("could not init package %d.\n", pkgIndex);
        return EXIT_FAILURE;
    }
    subscribeStatus = p_vehicle->subscribe->startPackage(pkgIndex, m_responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        printf("could not start package %d.\n", pkgIndex);
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup before return
        p_vehicle->subscribe->removePackage(pkgIndex, m_responseTimeout);
        return EXIT_FAILURE;
    }

    // Package 3
    pkgIndex                  = 3;
    freq                      = 100;
    m_topicList100Hz = {
        Telemetry::TOPIC_COMPASS,
        Telemetry::TOPIC_HEIGHT_FUSION,
        Telemetry::TOPIC_POSITION_VO,//??Hz
        Telemetry::TOPIC_AVOID_DATA,//??Hz
    };
    numTopic                  = sizeof(m_topicList100Hz) / sizeof(m_topicList100Hz[0]);
    enableTimestamp           = false;

    p_vehicle->subscribe->registerUserPackageUnpackCallback(pkgIndex, TelemetryCollector::callback100Hz, this);
    pkgStatus = p_vehicle->subscribe->initPackageFromTopicList(
        pkgIndex, numTopic, m_topicList100Hz.data(), enableTimestamp, freq);
    if (!(pkgStatus))
    {
        printf("could not init package %d.\n", pkgIndex);
        return EXIT_FAILURE;
    }
    subscribeStatus = p_vehicle->subscribe->startPackage(pkgIndex, m_responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        printf("could not start package %d.\n", pkgIndex);
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup before return
        p_vehicle->subscribe->removePackage(pkgIndex, m_responseTimeout);
        return EXIT_FAILURE;
    }

    // Package 4
    pkgIndex                   = 4;
    freq                       = 200;
    m_topicList200Hz = {
        Telemetry::TOPIC_ALTITUDE_FUSIONED,
        Telemetry::TOPIC_ALTITUDE_BAROMETER,
        Telemetry::TOPIC_VELOCITY,
        Telemetry::TOPIC_ACCELERATION_GROUND,
        Telemetry::TOPIC_ACCELERATION_BODY,
        Telemetry::TOPIC_ANGULAR_RATE_FUSIONED,
    };
    numTopic        = sizeof(m_topicList200Hz) / sizeof(m_topicList200Hz[0]);
    enableTimestamp = false;

    p_vehicle->subscribe->registerUserPackageUnpackCallback(pkgIndex, TelemetryCollector::callback200Hz, this);
    pkgStatus = p_vehicle->subscribe->initPackageFromTopicList(
        pkgIndex, numTopic, m_topicList200Hz.data(), enableTimestamp, freq);
    if (!(pkgStatus))
    {
        printf("could not init package %d.\n", pkgIndex);
        return EXIT_FAILURE;
    }
    else {
        subscribeStatus = p_vehicle->subscribe->startPackage(pkgIndex, m_responseTimeout);
        if(subscribeStatus.data == ErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE)
        {
            std::cout << "RTK Not Available" << "\n";
            m_rtkAvailable = false;
        }
        else
        {
            m_rtkAvailable = true;
            if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
                printf("could not start package %d.\n", pkgIndex);
                ACK::getErrorCodeMessage(subscribeStatus, __func__);
                // Cleanup before return
                p_vehicle->subscribe->removePackage(pkgIndex, m_responseTimeout);
                return EXIT_FAILURE;
            }
        }
    }

    // Package 5
    pkgIndex                   = 5;
    freq                       = 400;
    m_topicList400Hz = {
        Telemetry::TOPIC_ACCELERATION_RAW,
        Telemetry::TOPIC_ANGULAR_RATE_RAW,
        Telemetry::TOPIC_HARD_SYNC
    };
    numTopic        = sizeof(m_topicList400Hz) / sizeof(m_topicList400Hz[0]);
    enableTimestamp = false;

    p_vehicle->subscribe->registerUserPackageUnpackCallback(pkgIndex, TelemetryCollector::callback400Hz, this);
    pkgStatus = p_vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, m_topicList400Hz.data(), enableTimestamp, freq);
    if (!(pkgStatus))
    {
        printf("could not init package %d.\n", pkgIndex);
        return EXIT_FAILURE;
    }
    subscribeStatus = p_vehicle->subscribe->startPackage(pkgIndex, m_responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        printf("could not start package %d.\n", pkgIndex);
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup before return
        p_vehicle->subscribe->removePackage(pkgIndex, m_responseTimeout);
        return EXIT_FAILURE;
    }

    // Wait for the data to start coming in.
    std::this_thread::sleep_for(std::chrono::milliseconds{1000});
    return EXIT_SUCCESS;
}
