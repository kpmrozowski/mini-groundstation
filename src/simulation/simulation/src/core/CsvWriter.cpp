#include "CsvWriter.hpp"

#include <fmt/format.h>
#include <type_traits>
#include "utils.hpp"

#ifndef __has_include
static_assert(false, "__has_include not supported");
#else
#    if __cplusplus >= 201703L && __has_include(<filesystem>)
#        include <filesystem>
namespace fs = std::filesystem;
#    elif __has_include(<experimental/filesystem>)
#        include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#    elif __has_include(<boost/filesystem.hpp>)
#        include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#    endif
#endif

template <typename T>
static inline std::string stringify(T val) {
    if constexpr (std::is_integral_v<T> or std::is_enum_v<T>) {
        return fmt::format("{}", val);
    } else if constexpr (std::is_floating_point_v<T>) {
        return fmt::format("{:.12f}", val);
    }
    return {};
}

CsvWriter::CsvWriter(const std::string& save_path, uint16_t csvRefreshRate)
        : m_mut()
        , m_csvFile(save_path + "/telemetry.csv")
        , m_loopSleepTime(1.f / static_cast<float>(csvRefreshRate)) {
    this->initCsv();
}

void CsvWriter::initCsv() {
    m_csvFile  // 145 variables
        << "time,"
        << "1Hz,"
        << "5Hz,"
        << "50Hz,"
        << "100Hz,"
        << "200Hz,"
        << "400Hz,"
        /*50Hz*/
        << "quaternion.q0,"
        << "quaternion.q1,"
        << "quaternion.q2,"
        << "quaternion.q3,"
        << "accelerationGround.x,"
        << "accelerationGround.y,"
        << "accelerationGround.z,"
        << "accelerationBody.x,"
        << "accelerationBody.y,"
        << "accelerationBody.z,"
        << "accelerationRaw.x,"
        << "accelerationRaw.y,"
        << "accelerationRaw.z,"
        << "velocity.data.x,"
        << "velocity.data.y,"
        << "velocity.data.z,"
        << "angularRateFusioned.x,"
        << "angularRateFusioned.y,"
        << "angularRateFusioned.z,"
        << "angularRateRaw.x,"
        << "angularRateRaw.y,"
        << "angularRateRaw.z,"
        << "heightFusioned,"
        << "compass.x,"
        << "compass.y,"
        << "compass.z,"
        << "rc.gear,"
        << "rc.mode,"
        << "rc.roll,"
        << "rc.pitch,"
        << "rc.yaw,"
        << "rc.throttle,"
        << "positionVO.x,"
        << "positionVO.y,"
        << "positionVO.z,"
        << "positionVO.xHealth,"
        << "positionVO.yHealth,"
        << "positionVO.zHealth,"
        << "flightAnomaly.aircraftIsFalling,"
        << "flightAnomaly.atLeastOneEscDisconnected,"
        << "flightAnomaly.compassInstallationError,"
        << "flightAnomaly.escTemperatureHigh,"
        << "flightAnomaly.gpsYawError,"
        << "flightAnomaly.heightCtrlFail,"
        << "flightAnomaly.impactInAir,"
        << "flightAnomaly.imuInstallationError,"
        << "flightAnomaly.randomFly,"
        << "flightAnomaly.reserved,"
        << "flightAnomaly.rollPitchCtrlFail,"
        << "flightAnomaly.strongWindLevel1,"
        << "flightAnomaly.strongWindLevel2,"
        << "flightAnomaly.yawCtrlFail,"
        << "avoidData.front,"
        << "avoidData.back,"
        << "avoidData.right,"
        << "avoidData.left,"
        << "avoidData.up,"
        << "avoidData.down,"
        << "avoidData.frontHealth,"
        << "avoidData.backHealth,"
        << "avoidData.rightHealth,"
        << "avoidData.leftHealth,"
        << "avoidData.upHealth,"
        << "avoidData.downHealth,"
        << "avoidData.reserved,"
        << "gimbalAngles.x,"
        << "gimbalAngles.y,"
        << "gimbalAngles.z,"
        << "gimbalControlMode,"
        /*10Hz*/
        << "altitudeFusioned,"
        << "latLonFused.latitude,"
        << "latLonFused.longitude,"
        << "latLonFused.altitude,"
        << "latLonFused.visibleSatelliteNumber,"
        << "gpsControlLevel,"
        << "batteryInfo.capacity,"
        << "batteryInfo.current,"
        << "batteryInfo.percentage,"
        << "batteryInfo.voltage,"
        /*5 Hz*/
        << "gpsDate,"
        << "gpsTime,"
        << "gpsPosition.x,"
        << "gpsPosition.y,"
        << "gpsPosition.z,"
        << "gpsVelocity.x,"
        << "gpsVelocity.y,"
        << "gpsVelocity.z,"
        << "gpsDetails.fix,"
        << "gpsDetails.gnssStatus,"
        << "gpsDetails.GPScounter,"
        << "gpsDetails.hacc,"
        << "gpsDetails.hdop,"
        << "gpsDetails.NSV,"
        << "gpsDetails.pdop,"
        << "gpsDetails.sacc,"
        << "gpsDetails.usedGLN,"
        << "gpsDetails.usedGPS,"
        << "gpsSignalLevel,"
        << "controlDevice.controlMode,"
        << "controlDevice.deviceStatus,"
        << "controlDevice.flightStatus,"
        << "controlDevice.vrcStatus,"
        << "controlDevice.reserved,"
        << "homePointInfo.latitude,"
        << "homePointInfo.longitude,"
        << "statusFlight,"
        << "statusDisplayMode,"
        << "statusLandingGear,"
        << "statusMotorStartError,"
        << "statusRtkConnect.rtkConnected,"
        << "statusRtkConnect.reserve,"
        << "statusHomePointSet.status,"
        << "gimbalStatus.calibrating,"
        << "gimbalStatus.disabled_mvo,"
        << "gimbalStatus.droneDataRecv,"
        << "gimbalStatus.escRollStatus,"
        << "gimbalStatus.escPitchStatus,"
        << "gimbalStatus.escYawStatus,"
        << "gimbalStatus.FWUpdating,"
        << "gimbalStatus.gear_show_unable,"
        << "gimbalStatus.gyroFalut,"
        << "gimbalStatus.initUnfinished,"
        << "gimbalStatus.installedDirection,"
        << "gimbalStatus.isBusy,"
        << "gimbalStatus.mountStatus,"
        << "gimbalStatus.prevCalibrationgResult,"
        << "gimbalStatus.rollLimited,"
        << "gimbalStatus.pitchLimited,"
        << "gimbalStatus.yawLimited,"
        << "gimbalStatus.reserved2,"
        /*5 Hz*/
        << "rtkPosition.HFSL,"
        << "rtkPosition.longitude,"
        << "rtkPosition.latitude,"
        << "rtkVelocity.x,"
        << "rtkVelocity.y,"
        << "rtkVelocity.z,"
        << "rtkYaw,"
        << "rtkPositionInfo,"
        << "rtkYawInfo\n";
}

CsvWriter::~CsvWriter() {
    m_terminateLoop = true;
    m_csvSavingThread.join();
    m_csvFile.close();
}

#define MAYBE_STRINGIFY(member)               \
    if (s_data.member != m_lastData.member) { \
        m_lastData.member = s_data.member;    \
        row += stringify(m_lastData.member);  \
        change = true;                        \
    }                                         \
    row += ",";

#define SURELY_STRINGIFY(member)              \
    if (s_data.member != m_lastData.member) { \
        m_lastData.member = s_data.member;    \
        change = true;                        \
    }                                         \
    row += stringify(m_lastData.member);      \
    row += ",";

void CsvWriter::append() {
    std::string row;
    row.reserve(3000);
    bool change = true;

    row += utils::time_in_YYYYMMDDHHMMSSMMM(6) + ",";
    row += fmt::format("{},", s_updatedTelemetry.at(0));
    row += fmt::format("{},", s_updatedTelemetry.at(1));
    row += fmt::format("{},", s_updatedTelemetry.at(2));
    row += fmt::format("{},", s_updatedTelemetry.at(3));
    row += fmt::format("{},", s_updatedTelemetry.at(4));
    row += fmt::format("{},", s_updatedTelemetry.at(5));
    s_updatedTelemetry = {false, false, false, false, false, false};
    SURELY_STRINGIFY(quaternion.q0)
    SURELY_STRINGIFY(quaternion.q1)
    SURELY_STRINGIFY(quaternion.q2)
    SURELY_STRINGIFY(quaternion.q3)
    SURELY_STRINGIFY(accelerationGround.x)
    SURELY_STRINGIFY(accelerationGround.y)
    SURELY_STRINGIFY(accelerationGround.z)
    SURELY_STRINGIFY(accelerationBody.x)
    SURELY_STRINGIFY(accelerationBody.y)
    SURELY_STRINGIFY(accelerationBody.z)
    SURELY_STRINGIFY(accelerationRaw.x)
    SURELY_STRINGIFY(accelerationRaw.y)
    SURELY_STRINGIFY(accelerationRaw.z)
    SURELY_STRINGIFY(velocity.data.x)
    SURELY_STRINGIFY(velocity.data.y)
    SURELY_STRINGIFY(velocity.data.z)
    SURELY_STRINGIFY(angularRateFusioned.x)
    SURELY_STRINGIFY(angularRateFusioned.y)
    SURELY_STRINGIFY(angularRateFusioned.z)
    SURELY_STRINGIFY(angularRateRaw.x)
    SURELY_STRINGIFY(angularRateRaw.y)
    SURELY_STRINGIFY(angularRateRaw.z)
    SURELY_STRINGIFY(heightFusioned)
    SURELY_STRINGIFY(compass.x)
    SURELY_STRINGIFY(compass.y)
    SURELY_STRINGIFY(compass.z)
    SURELY_STRINGIFY(rc.gear)
    SURELY_STRINGIFY(rc.mode)
    SURELY_STRINGIFY(rc.roll)
    SURELY_STRINGIFY(rc.pitch)
    SURELY_STRINGIFY(rc.yaw)
    SURELY_STRINGIFY(rc.throttle)
    SURELY_STRINGIFY(positionVO.x)
    SURELY_STRINGIFY(positionVO.y)
    SURELY_STRINGIFY(positionVO.z)
    SURELY_STRINGIFY(positionVO.xHealth)
    SURELY_STRINGIFY(positionVO.yHealth)
    SURELY_STRINGIFY(positionVO.zHealth)
    SURELY_STRINGIFY(flightAnomaly.aircraftIsFalling)
    SURELY_STRINGIFY(flightAnomaly.atLeastOneEscDisconnected)
    SURELY_STRINGIFY(flightAnomaly.compassInstallationError)
    SURELY_STRINGIFY(flightAnomaly.escTemperatureHigh)
    SURELY_STRINGIFY(flightAnomaly.gpsYawError)
    SURELY_STRINGIFY(flightAnomaly.heightCtrlFail)
    SURELY_STRINGIFY(flightAnomaly.impactInAir)
    SURELY_STRINGIFY(flightAnomaly.imuInstallationError)
    SURELY_STRINGIFY(flightAnomaly.randomFly)
    SURELY_STRINGIFY(flightAnomaly.reserved)
    SURELY_STRINGIFY(flightAnomaly.rollPitchCtrlFail)
    SURELY_STRINGIFY(flightAnomaly.strongWindLevel1)
    SURELY_STRINGIFY(flightAnomaly.strongWindLevel2)
    SURELY_STRINGIFY(flightAnomaly.yawCtrlFail)
    SURELY_STRINGIFY(avoidData.front)
    SURELY_STRINGIFY(avoidData.back)
    SURELY_STRINGIFY(avoidData.right)
    SURELY_STRINGIFY(avoidData.left)
    SURELY_STRINGIFY(avoidData.up)
    SURELY_STRINGIFY(avoidData.down)
    SURELY_STRINGIFY(avoidData.frontHealth)
    SURELY_STRINGIFY(avoidData.backHealth)
    SURELY_STRINGIFY(avoidData.rightHealth)
    SURELY_STRINGIFY(avoidData.leftHealth)
    SURELY_STRINGIFY(avoidData.upHealth)
    SURELY_STRINGIFY(avoidData.downHealth)
    SURELY_STRINGIFY(avoidData.reserved)
    SURELY_STRINGIFY(gimbalAngles.x)
    SURELY_STRINGIFY(gimbalAngles.y)
    SURELY_STRINGIFY(gimbalAngles.z)
    SURELY_STRINGIFY(gimbalControlMode)
    SURELY_STRINGIFY(altitudeFusioned)
    SURELY_STRINGIFY(latLonFused.latitude)
    SURELY_STRINGIFY(latLonFused.longitude)
    SURELY_STRINGIFY(latLonFused.altitude)
    SURELY_STRINGIFY(latLonFused.visibleSatelliteNumber)
    SURELY_STRINGIFY(gpsControlLevel)
    SURELY_STRINGIFY(batteryInfo.capacity)
    SURELY_STRINGIFY(batteryInfo.current)
    SURELY_STRINGIFY(batteryInfo.percentage)
    SURELY_STRINGIFY(batteryInfo.voltage)
    SURELY_STRINGIFY(gpsDate)
    SURELY_STRINGIFY(gpsTime)
    SURELY_STRINGIFY(gpsPosition.x)
    SURELY_STRINGIFY(gpsPosition.y)
    SURELY_STRINGIFY(gpsPosition.z)
    SURELY_STRINGIFY(gpsVelocity.x)
    SURELY_STRINGIFY(gpsVelocity.y)
    SURELY_STRINGIFY(gpsVelocity.z)
    SURELY_STRINGIFY(gpsDetails.fix)
    SURELY_STRINGIFY(gpsDetails.gnssStatus)
    SURELY_STRINGIFY(gpsDetails.GPScounter)
    SURELY_STRINGIFY(gpsDetails.hacc)
    SURELY_STRINGIFY(gpsDetails.hdop)
    SURELY_STRINGIFY(gpsDetails.NSV)
    SURELY_STRINGIFY(gpsDetails.pdop)
    SURELY_STRINGIFY(gpsDetails.sacc)
    SURELY_STRINGIFY(gpsDetails.usedGLN)
    SURELY_STRINGIFY(gpsDetails.usedGPS)
    SURELY_STRINGIFY(gpsSignalLevel)
    SURELY_STRINGIFY(controlDevice.controlMode)
    SURELY_STRINGIFY(controlDevice.deviceStatus)
    SURELY_STRINGIFY(controlDevice.flightStatus)
    SURELY_STRINGIFY(controlDevice.vrcStatus)
    SURELY_STRINGIFY(controlDevice.reserved)
    SURELY_STRINGIFY(homePointInfo.latitude)
    SURELY_STRINGIFY(homePointInfo.longitude)
    SURELY_STRINGIFY(statusFlight)
    SURELY_STRINGIFY(statusDisplayMode)
    SURELY_STRINGIFY(statusLandingGear)
    SURELY_STRINGIFY(statusMotorStartError)
    SURELY_STRINGIFY(statusRtkConnect.rtkConnected)
    SURELY_STRINGIFY(statusRtkConnect.reserve)
    SURELY_STRINGIFY(statusHomePointSet.status)
    SURELY_STRINGIFY(gimbalStatus.calibrating)
    SURELY_STRINGIFY(gimbalStatus.disabled_mvo)
    SURELY_STRINGIFY(gimbalStatus.droneDataRecv)
    SURELY_STRINGIFY(gimbalStatus.escRollStatus)
    SURELY_STRINGIFY(gimbalStatus.escPitchStatus)
    SURELY_STRINGIFY(gimbalStatus.escYawStatus)
    SURELY_STRINGIFY(gimbalStatus.FWUpdating)
    SURELY_STRINGIFY(gimbalStatus.gear_show_unable)
    SURELY_STRINGIFY(gimbalStatus.gyroFalut)
    SURELY_STRINGIFY(gimbalStatus.initUnfinished)
    SURELY_STRINGIFY(gimbalStatus.installedDirection)
    SURELY_STRINGIFY(gimbalStatus.isBusy)
    SURELY_STRINGIFY(gimbalStatus.mountStatus)
    SURELY_STRINGIFY(gimbalStatus.prevCalibrationgResult)
    SURELY_STRINGIFY(gimbalStatus.rollLimited)
    SURELY_STRINGIFY(gimbalStatus.pitchLimited)
    SURELY_STRINGIFY(gimbalStatus.yawLimited)
    SURELY_STRINGIFY(gimbalStatus.reserved2)
    SURELY_STRINGIFY(rtkPosition.HFSL)
    SURELY_STRINGIFY(rtkPosition.longitude)
    SURELY_STRINGIFY(rtkPosition.latitude)
    SURELY_STRINGIFY(rtkVelocity.x)
    SURELY_STRINGIFY(rtkVelocity.y)
    SURELY_STRINGIFY(rtkVelocity.z)
    SURELY_STRINGIFY(rtkYaw)
    SURELY_STRINGIFY(rtkPositionInfo)
    SURELY_STRINGIFY(rtkYawInfo)
    // if (change) {
    row[row.size() - 1] = '\n';
    std::unique_lock lck{m_mut};
    m_csvFile << row;
    // }
}

void CsvWriter::save_csv_in_a_loop() {
    while (not m_terminateLoop) {
        std::this_thread::sleep_for(m_loopSleepTime);
        this->append();
    }
}

void CsvWriter::start_loop() {
    // m_csvSavingThread = std::thread(&CsvWriter::save_csv_in_a_loop, this);
}
