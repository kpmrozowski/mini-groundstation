#ifndef DjiCollector_HPP
#define DjiCollector_HPP

#include "BatteryCollector.hpp"
#include "CollectedData.hpp"
#include "CsvWriter.hpp"
#include "ImagesCollector.hpp"
#include "SharedData.hpp"
#include "TelemetryCollector.hpp"
#include "TimeCollector.hpp"

class DjiCollector : protected SharedData {
    std::string m_dataPath;

    BatteryCollector m_batteryCollector;
    ImagesCollector m_imagesCollector;
    TelemetryCollector m_telemetryCollector;
    TimeCollector m_timeCollector;

    CsvWriter m_csvWriter;

    DjiCollector() = delete;
    DjiCollector(const DjiCollector&) = delete;
    DjiCollector(DjiCollector&&) = delete;
    DjiCollector& operator=(const DjiCollector&) = delete;
    DjiCollector& operator=(DjiCollector&&) = delete;

public:
    DjiCollector(Constants&& consts, DJI::OSDK::Vehicle* vehicle);
    static std::unique_ptr<DjiCollector>& create(std::string configPath, DJI::OSDK::Vehicle* vehicle);
    void start_to_save_data();
};

#endif  // DjiCollector_HPP
