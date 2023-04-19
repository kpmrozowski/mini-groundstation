#ifndef TelemetryCollector_HPP
#define TelemetryCollector_HPP

#include <thread>

#include "SharedData.hpp"

class CsvWriter;
// docs: deps/Onboard-SDK/osdk-core/api/inc/dji_telemetry_doc.hpp:0

class TelemetryCollector : protected SharedData {

    DJI::OSDK::Vehicle* p_vehicle;
    int m_responseTimeout = 1;  // seconds
    std::thread m_collectTelemetryDataThread;
    bool m_terminateCollector = false;
    CsvWriter* mp_csvWriter;
 
    std::array<DJI::OSDK::Telemetry::TopicName, 1> m_topicList1Hz;
    std::array<DJI::OSDK::Telemetry::TopicName, 13> m_topicList5Hz;
    std::array<DJI::OSDK::Telemetry::TopicName, 15> m_topicList50Hz;
    std::array<DJI::OSDK::Telemetry::TopicName, 4> m_topicList100Hz;
    std::array<DJI::OSDK::Telemetry::TopicName, 6> m_topicList200Hz;
    std::array<DJI::OSDK::Telemetry::TopicName, 3> m_topicList400Hz;

public:
    TelemetryCollector(const Constants& consts, DJI::OSDK::Vehicle* vehicle);
    TelemetryCollector() = delete;
    TelemetryCollector(const TelemetryCollector&) = delete;
    TelemetryCollector(TelemetryCollector&&) = delete;
    TelemetryCollector& operator=(const TelemetryCollector&) = delete;
    TelemetryCollector& operator=(TelemetryCollector&&) = delete;

    void subscribe();
    void unsubscribe();

    // RTK can be detected as unavailable only for Flight controllers that don't support RTK
    bool m_rtkAvailable = false;

private:
    static void callback400Hz(
        DJI::OSDK::Vehicle* vehicle,
        DJI::OSDK::RecvContainer recvFrame,
        DJI::OSDK::UserData userData);
    static void callback200Hz(
        DJI::OSDK::Vehicle* vehicle,
        DJI::OSDK::RecvContainer recvFrame,
        DJI::OSDK::UserData userData);
    static void callback100Hz(
        DJI::OSDK::Vehicle* vehicle,
        DJI::OSDK::RecvContainer recvFrame,
        DJI::OSDK::UserData userData);
    static void callback50Hz(
        DJI::OSDK::Vehicle* vehicle,
        DJI::OSDK::RecvContainer recvFrame,
        DJI::OSDK::UserData userData);
    static void callback5Hz(
        DJI::OSDK::Vehicle* vehicle,
        DJI::OSDK::RecvContainer recvFrame,
        DJI::OSDK::UserData userData);
    static void callback1Hz(
        DJI::OSDK::Vehicle* vehicle,
        DJI::OSDK::RecvContainer recvFrame,
        DJI::OSDK::UserData userData);

    void collectTelemetryData();
    bool trySubscribe();

};

#endif // TelemetryCollector_HPP
