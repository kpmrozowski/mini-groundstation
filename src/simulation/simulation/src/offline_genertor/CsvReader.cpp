#include "CsvReader.hpp"

#include <fstream>

#include <spdlog/spdlog.h>

template <typename T>
static inline T unstringify([[maybe_unused]] T val, const std::string& val_str) {
    // fmt::print("unstringifying: {}, ", val_str);
    T ret{};
    if constexpr (std::is_same<T, size_t>::value) {
        ret = std::stoull(val_str);
    } else if constexpr (std::is_same<T, double>::value) {
        ret = std::stod(val_str);
    } else if constexpr (std::is_same<T, float>::value) {
        ret = std::stof(val_str);
    } else if constexpr (std::is_unsigned_v<T>) {
        ret = std::stoul(val_str);
    } else if constexpr (std::is_integral_v<T>) {
        ret = std::stoi(val_str);
    } else if constexpr (std::is_same<T, std::string>::value) {
        ret = val_str;
    }
    // fmt::print("unstringified: {}\n", ret);
    return ret;
}

#define READ_VALUE(member)                                                                                          \
    ++end_poz_idx;                                                                                                  \
    start_poz_idx = end_poz_idx;                                                                                    \
    end_poz_idx = line.find_first_of(",", end_poz_idx);                                                             \
    try {                                                                                                           \
        member = unstringify(member, line.substr(start_poz_idx, end_poz_idx - start_poz_idx));                      \
    } catch (const std::invalid_argument& e) {                                                                      \
        spdlog::error("what: {}, vsl_str='{}'", e.what(), line.substr(start_poz_idx, end_poz_idx - start_poz_idx)); \
    }

std::map<int, CollectedData> CsvReader::readCsvRaw(const std::string& path) {
    if (path.empty()) {
        return {};
    }
    spdlog::info("Reading CSV: {}", path);
    std::fstream csv_file;
    csv_file.open(path);
    if (!csv_file.good()) {
        spdlog::warn("Non empty file path provided, but fail to read. Got {}. Returning empty map", path);
        return {};
    }

    std::map<int, CollectedData> data;

    std::string line;
    while (std::getline(csv_file, line)) {
        // spdlog::info("{}", line);

        // we expect data in order of id | values
        size_t start_poz_idx = 0;
        size_t end_poz_idx = line.find_first_of(",");
        const std::string idx_str = line.substr(start_poz_idx, end_poz_idx - start_poz_idx);
        if (idx_str == "id") {
            continue;
        }
        const int idx = std::stoi(idx_str);

        if (data.find(idx) != data.cend()) {
            spdlog::error("detected multiple indexes. Got second time {}.", idx);
            throw std::invalid_argument("inconsistent indexing");
        }
        std::string omit;

        CollectedData& row = data[idx];
        READ_VALUE(row.timestamp)
        READ_VALUE(omit)
        READ_VALUE(omit)
        READ_VALUE(omit)
        READ_VALUE(omit)
        READ_VALUE(omit)
        READ_VALUE(omit)
        READ_VALUE(row.quaternion.q0)
        READ_VALUE(row.quaternion.q1)
        READ_VALUE(row.quaternion.q2)
        READ_VALUE(row.quaternion.q3)
        READ_VALUE(row.accelerationGround.x)
        READ_VALUE(row.accelerationGround.y)
        READ_VALUE(row.accelerationGround.z)
        READ_VALUE(row.accelerationBody.x)
        READ_VALUE(row.accelerationBody.y)
        READ_VALUE(row.accelerationBody.z)
        READ_VALUE(row.accelerationRaw.x)
        READ_VALUE(row.accelerationRaw.y)
        READ_VALUE(row.accelerationRaw.z)
        READ_VALUE(row.velocity.data.x)
        READ_VALUE(row.velocity.data.y)
        READ_VALUE(row.velocity.data.z)
        READ_VALUE(row.angularRateFusioned.x)
        READ_VALUE(row.angularRateFusioned.y)
        READ_VALUE(row.angularRateFusioned.z)
        READ_VALUE(row.angularRateRaw.x)
        READ_VALUE(row.angularRateRaw.y)
        READ_VALUE(row.angularRateRaw.z)
        READ_VALUE(row.heightFusioned)
        READ_VALUE(row.compass.x)
        READ_VALUE(row.compass.y)
        READ_VALUE(row.compass.z)
        READ_VALUE(row.rc.gear)
        READ_VALUE(row.rc.mode)
        READ_VALUE(row.rc.roll)
        READ_VALUE(row.rc.pitch)
        READ_VALUE(row.rc.yaw)
        READ_VALUE(row.rc.throttle)
        READ_VALUE(row.positionVO.x)
        READ_VALUE(row.positionVO.y)
        READ_VALUE(row.positionVO.z)
        READ_VALUE(row.positionVO.xHealth)
        READ_VALUE(row.positionVO.yHealth)
        READ_VALUE(row.positionVO.zHealth)
        READ_VALUE(row.flightAnomaly.aircraftIsFalling)
        READ_VALUE(row.flightAnomaly.atLeastOneEscDisconnected)
        READ_VALUE(row.flightAnomaly.compassInstallationError)
        READ_VALUE(row.flightAnomaly.escTemperatureHigh)
        READ_VALUE(row.flightAnomaly.gpsYawError)
        READ_VALUE(row.flightAnomaly.heightCtrlFail)
        READ_VALUE(row.flightAnomaly.impactInAir)
        READ_VALUE(row.flightAnomaly.imuInstallationError)
        READ_VALUE(row.flightAnomaly.randomFly)
        READ_VALUE(row.flightAnomaly.reserved)
        READ_VALUE(row.flightAnomaly.rollPitchCtrlFail)
        READ_VALUE(row.flightAnomaly.strongWindLevel1)
        READ_VALUE(row.flightAnomaly.strongWindLevel2)
        READ_VALUE(row.flightAnomaly.yawCtrlFail)
        READ_VALUE(row.avoidData.front)
        READ_VALUE(row.avoidData.back)
        READ_VALUE(row.avoidData.right)
        READ_VALUE(row.avoidData.left)
        READ_VALUE(row.avoidData.up)
        READ_VALUE(row.avoidData.down)
        READ_VALUE(row.avoidData.frontHealth)
        READ_VALUE(row.avoidData.backHealth)
        READ_VALUE(row.avoidData.rightHealth)
        READ_VALUE(row.avoidData.leftHealth)
        READ_VALUE(row.avoidData.upHealth)
        READ_VALUE(row.avoidData.downHealth)
        READ_VALUE(row.avoidData.reserved)
        READ_VALUE(row.gimbalAngles.x)
        READ_VALUE(row.gimbalAngles.y)
        READ_VALUE(row.gimbalAngles.z)
        READ_VALUE(row.gimbalControlMode)
        READ_VALUE(row.altitudeFusioned)
        READ_VALUE(row.latLonFused.latitude)
        READ_VALUE(row.latLonFused.longitude)
        READ_VALUE(row.latLonFused.altitude)
        READ_VALUE(row.latLonFused.visibleSatelliteNumber)
        READ_VALUE(row.gpsControlLevel)
        READ_VALUE(row.batteryInfo.capacity)
        READ_VALUE(row.batteryInfo.current)
        READ_VALUE(row.batteryInfo.percentage)
        READ_VALUE(row.batteryInfo.voltage)
        READ_VALUE(row.gpsDate)
        READ_VALUE(row.gpsTime)
        READ_VALUE(row.gpsPosition.x)
        READ_VALUE(row.gpsPosition.y)
        READ_VALUE(row.gpsPosition.z)
        READ_VALUE(row.gpsVelocity.x)
        READ_VALUE(row.gpsVelocity.y)
        READ_VALUE(row.gpsVelocity.z)
        READ_VALUE(row.gpsDetails.fix)
        READ_VALUE(row.gpsDetails.gnssStatus)
        READ_VALUE(row.gpsDetails.GPScounter)
        READ_VALUE(row.gpsDetails.hacc)
        READ_VALUE(row.gpsDetails.hdop)
        READ_VALUE(row.gpsDetails.NSV)
        READ_VALUE(row.gpsDetails.pdop)
        READ_VALUE(row.gpsDetails.sacc)
        READ_VALUE(row.gpsDetails.usedGLN)
        READ_VALUE(row.gpsDetails.usedGPS)
        READ_VALUE(row.gpsSignalLevel)
        READ_VALUE(row.controlDevice.controlMode)
        READ_VALUE(row.controlDevice.deviceStatus)
        READ_VALUE(row.controlDevice.flightStatus)
        READ_VALUE(row.controlDevice.vrcStatus)
        READ_VALUE(row.controlDevice.reserved)
        READ_VALUE(row.homePointInfo.latitude)
        READ_VALUE(row.homePointInfo.longitude)
        READ_VALUE(row.statusFlight)
        READ_VALUE(row.statusDisplayMode)
        READ_VALUE(row.statusLandingGear)
        READ_VALUE(row.statusMotorStartError)
        READ_VALUE(row.statusRtkConnect.rtkConnected)
        READ_VALUE(row.statusRtkConnect.reserve)
        READ_VALUE(row.statusHomePointSet.status)
        READ_VALUE(row.gimbalStatus.calibrating)
        READ_VALUE(row.gimbalStatus.disabled_mvo)
        READ_VALUE(row.gimbalStatus.droneDataRecv)
        READ_VALUE(row.gimbalStatus.escRollStatus)
        READ_VALUE(row.gimbalStatus.escPitchStatus)
        READ_VALUE(row.gimbalStatus.escYawStatus)
        READ_VALUE(row.gimbalStatus.FWUpdating)
        READ_VALUE(row.gimbalStatus.gear_show_unable)
        READ_VALUE(row.gimbalStatus.gyroFalut)
        READ_VALUE(row.gimbalStatus.initUnfinished)
        READ_VALUE(row.gimbalStatus.installedDirection)
        READ_VALUE(row.gimbalStatus.isBusy)
        READ_VALUE(row.gimbalStatus.mountStatus)
        READ_VALUE(row.gimbalStatus.prevCalibrationgResult)
        READ_VALUE(row.gimbalStatus.rollLimited)
        READ_VALUE(row.gimbalStatus.pitchLimited)
        READ_VALUE(row.gimbalStatus.yawLimited)
        READ_VALUE(row.gimbalStatus.reserved2)
        READ_VALUE(row.rtkPosition.HFSL)
        READ_VALUE(row.rtkPosition.longitude)
        READ_VALUE(row.rtkPosition.latitude)
        READ_VALUE(row.rtkVelocity.x)
        READ_VALUE(row.rtkVelocity.y)
        READ_VALUE(row.rtkVelocity.z)
        READ_VALUE(row.rtkYaw)
        READ_VALUE(row.rtkPositionInfo)
        READ_VALUE(row.rtkYawInfo)
    }
    return data;
}
