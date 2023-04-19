#ifndef TimeCollector_HPP
#define TimeCollector_HPP

#include <memory>

#include "dji_linux_helpers.hpp"

#include "SharedData.hpp"

class TimeCollector : protected SharedData {
    DJI::OSDK::Vehicle* p_vehicle;

public:
    TimeCollector(const Constants& consts, DJI::OSDK::Vehicle* vehicle);
    TimeCollector() = delete;
    TimeCollector(const TimeCollector&) = delete;
    TimeCollector(TimeCollector&&) = delete;
    TimeCollector& operator=(const TimeCollector&) = delete;
    TimeCollector& operator=(TimeCollector&&) = delete;

    void subscribe();
    void unsubscribe();

private:
    static void nmeaCallback(Vehicle* vehiclePtr, RecvContainer recvFrame, UserData userData);
    static void utcTimeCallback(Vehicle* vehiclePtr, RecvContainer recvFrame, UserData userData);
    static void fcTimeInUTCCallback(Vehicle* vehiclePtr, RecvContainer recvFrame, UserData userData);
    static void ppsSourceCallback(Vehicle* vehiclePtr, RecvContainer recvFrame, UserData userData);
};

#endif  // TimeCollector_HPP
