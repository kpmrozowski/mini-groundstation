#include "BatteryCollector.hpp"

#include "dji_vehicle.hpp"
#include "osdkosal_linux.h"

BatteryCollector::BatteryCollector(const Constants& consts, DJI::OSDK::Vehicle* vehicle)
 : p_vehicle(vehicle)
{
    if (not consts.subscribeToBattery()) {
        printf("subscribeToBattery is false, BatteryCollector will not run.\n");
        return;
    }
    printf("Subscribing to Battery.\n");
    
    if (nullptr != p_vehicle) {
        subscribe();
    } else {
        printf("p_vehicle is nullptr, to continue change dryRun to false\n");
    }
}

void BatteryCollector::collectBatteryData() {
    // TODO: try ONBOARD-SDK-ROS sample, because this returns 0 only.
    while (not m_terminateCollector) {
        printf("battery callback\n");
        p_vehicle->djiBattery->getBatteryWholeInfo(s_data.batteryWholeInfo);
        DSTATUS("batteryCapacityPercentage is %ld\n", s_data.batteryWholeInfo.batteryCapacityPercentage);

        p_vehicle->djiBattery->getSingleBatteryDynamicInfo(
            DJI::OSDK::DJIBattery::RequestSmartBatteryIndex::FIRST_SMART_BATTERY, s_data.firstBatteryDynamicInfo);
        DSTATUS("battery index %d batteryCapacityPercent is %ld\n",
            s_data.firstBatteryDynamicInfo.batteryIndex, s_data.firstBatteryDynamicInfo.batteryCapacityPercent);

        p_vehicle->djiBattery->getSingleBatteryDynamicInfo(
            DJI::OSDK::DJIBattery::RequestSmartBatteryIndex::SECOND_SMART_BATTERY, s_data.secondBatteryDynamicInfo);
        DSTATUS("battery index %d batteryCapacityPercent is %ld\n",
            s_data.secondBatteryDynamicInfo.batteryIndex, s_data.secondBatteryDynamicInfo.batteryCapacityPercent);

        OsdkLinux_TaskSleepMs(m_waitTimeMs);
    }
}

void BatteryCollector::subscribe() {
    printf("subscribing to battery\n");
    bool enableSubscribeBatteryWholeInfo = true;
    p_vehicle->djiBattery->subscribeBatteryWholeInfo(enableSubscribeBatteryWholeInfo);
    m_collectBatteryDataThread = std::thread(&BatteryCollector::collectBatteryData, this);
    printf("subscribed to battery\n");
}

void BatteryCollector::unsubscribe() {
    m_terminateCollector = true;
    m_collectBatteryDataThread.join();
}
