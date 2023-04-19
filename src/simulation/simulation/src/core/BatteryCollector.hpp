#ifndef BatteryCollector_HPP
#define BatteryCollector_HPP

#include <thread>

#include "SharedData.hpp"

class BatteryCollector : protected SharedData {
    
    static constexpr int m_waitTimeMs = 500;

    DJI::OSDK::Vehicle* p_vehicle;
    std::thread m_collectBatteryDataThread;
    bool m_terminateCollector = false;

  public:
    BatteryCollector(const Constants& consts, DJI::OSDK::Vehicle* vehicle);
    BatteryCollector() = delete;
    BatteryCollector(const BatteryCollector&) = delete;
    BatteryCollector(BatteryCollector&&) = delete;
    BatteryCollector& operator=(const BatteryCollector&) = delete;
    BatteryCollector& operator=(BatteryCollector&&) = delete;

    void subscribe();
    void unsubscribe();

  private:
    void collectBatteryData();
};

#endif // BatteryCollector_HPP
