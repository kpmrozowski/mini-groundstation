#include "TimeCollector.hpp"

TimeCollector::TimeCollector(const Constants& consts, DJI::OSDK::Vehicle* vehicle)
: p_vehicle(vehicle)
{
    if (not consts.subscribeToTimers()) {
        printf("subscribeToTimers is false, TimeCollector will not run.\n");
        return;
    }
    printf("Subscribing to Timers.\n");
    if (nullptr != p_vehicle) {
        subscribe();
    } else {
        printf("p_vehicle is nullptr, to continue change dryRun to false.\n");
    }
}

void TimeCollector::nmeaCallback(Vehicle* vehiclePtr,
                                 RecvContainer recvFrame,
                                 UserData userData)
{
    printf("nmea time callback.\n");
    int length = recvFrame.recvInfo.len-OpenProtocol::PackageMin-4;
    uint8_t rawBuf[length];
    memcpy(rawBuf, recvFrame.recvData.raw_ack_array, length);
    DSTATUS("%s\n", std::string((char*)rawBuf, length).c_str());
}

void TimeCollector::utcTimeCallback(Vehicle* vehiclePtr,
                     RecvContainer recvFrame,
                     UserData userData)
{
    printf("utc time callback.\n");
    int length = recvFrame.recvInfo.len-OpenProtocol::PackageMin-4;
    uint8_t rawBuf[length];
    memcpy(rawBuf, recvFrame.recvData.raw_ack_array, length);
    DSTATUS("The UTC time for the next PPS pulse (ard 500ms) is...");
    DSTATUS("%s\n", std::string((char*)rawBuf, length).c_str());
}

void TimeCollector::fcTimeInUTCCallback(Vehicle* vehiclePtr,
                       RecvContainer recvFrame,
                       UserData userData)
{
    printf("fc time callback.\n");
    DSTATUS("Received Flight controller time in UTC reference...");
    DSTATUS("FC: %u, UTC time: %u, UTC date: %u.\n",
            recvFrame.recvData.fcTimeInUTC.fc_timestamp_us,
            recvFrame.recvData.fcTimeInUTC.utc_hhmmss,
            recvFrame.recvData.fcTimeInUTC.utc_yymmdd);
}

void TimeCollector::ppsSourceCallback(Vehicle* vehiclePtr,
                         RecvContainer recvFrame,
                         UserData userData)
{
    printf("pps time callback\n");
    std::vector<std::string> stringVec = {"0", "INTERNAL_GPS", "EXTERNAL_GPS", "RTK"};
    DSTATUS("PPS pulse is coming from %s\n", stringVec[recvFrame.recvData.ppsSourceType].c_str());
}

void TimeCollector::subscribe() {
    printf("subscribing to time.\n");
    // Note that these CBs share the same thread with serial reading
    p_vehicle->hardSync->subscribeNMEAMsgs(nmeaCallback, nullptr);
    p_vehicle->hardSync->subscribeUTCTime(utcTimeCallback, nullptr);
    p_vehicle->hardSync->subscribeFCTimeInUTCRef(fcTimeInUTCCallback, nullptr);
    p_vehicle->hardSync->subscribePPSSource(ppsSourceCallback, nullptr);
    printf("subscribed to time.\n");
}

void TimeCollector::unsubscribe() {
    p_vehicle->hardSync->unsubscribeNMEAMsgs();
    p_vehicle->hardSync->unsubscribeUTCTime();
    p_vehicle->hardSync->unsubscribeFCTimeInUTCRef();
    p_vehicle->hardSync->unsubscribePPSSource();
}
