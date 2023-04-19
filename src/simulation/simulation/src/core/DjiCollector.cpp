#include "DjiCollector.hpp"

#include "BatteryCollector.hpp"
#include "CollectedData.hpp"
#include "CsvWriter.hpp"
#include "ImagesCollector.hpp"
#include "TelemetryCollector.hpp"
#include "TimeCollector.hpp"

#ifndef __has_include
static_assert(false, "__has_include not supported");
#else
#    if __cplusplus >= 201703L && __has_include(<filesystem>)
#        include <filesystem>
namespace fs = std::filesystem;
#    elif __has_include(<experimental/filesystem>)
#        include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#    endif
#endif

DjiCollector::DjiCollector(Constants&& consts, DJI::OSDK::Vehicle* vehicle)
        : m_dataPath(consts.dataPath() + "/flight_" + utils::time_in_YYYY_MM_DD_HH_MM_SS())
        , m_batteryCollector(consts, vehicle)
        , m_imagesCollector(consts, vehicle, m_dataPath)
        , m_telemetryCollector(consts, vehicle)
        , m_timeCollector(consts, vehicle)
        , m_csvWriter(m_dataPath, consts.csvRefreshRate()) {
    s_consts = std::move(consts);
    fs::create_directories(m_dataPath);
    this->start_to_save_data();
}

std::unique_ptr<DjiCollector>& DjiCollector::create(std::string configPath, DJI::OSDK::Vehicle* vehicle) {
    Constants consts{configPath};

    static std::unique_ptr<DjiCollector> djiCollector = std::make_unique<DjiCollector>(std::move(consts), vehicle);
    return djiCollector;
}

void DjiCollector::start_to_save_data() {
    if (s_consts.subscribeToTelemetry() and s_consts.shallStartCsvWriter()) {
        m_csvWriter.start_loop();
    }
}
