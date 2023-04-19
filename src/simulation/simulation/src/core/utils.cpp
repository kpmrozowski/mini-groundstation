#include "utils.hpp"

#include <cmath>
#include <iomanip>
#include <chrono>
#include <sstream>

namespace utils {

std::string time_in_YYYY_MM_DD_HH_MM_SS()
{
    using namespace std::chrono;
    // get current time
    auto now = system_clock::now();

    // convert to std::time_t in order to convert to std::tm (broken time)
    auto timer = system_clock::to_time_t(now);

    // convert to broken time
    std::tm bt = *std::localtime(&timer);
    std::ostringstream oss;
    oss << std::put_time(&bt, "%Y%m%d-%H%M%S"); // HH:MM:SS
    return oss.str();
}

std::string time_in_YYYYMMDDHHMMSSMMM(int precision)
{
    using namespace std::chrono;
    auto now = system_clock::now();

    using seconds_double = std::chrono::duration<double, std::chrono::seconds::period>;
    auto us = duration_cast<seconds_double>(now.time_since_epoch());
    uint32_t usd = static_cast<uint32_t>(
        std::pow(10, precision) * (us.count() - static_cast<int>(us.count())));

    auto timer = system_clock::to_time_t(now);
    std::tm bt = *std::localtime(&timer);
    std::ostringstream oss;
    oss << std::put_time(&bt, "%Y%m%d%H%M%S"); // HH:MM:SS
    oss << '.' << std::setfill('0') << std::setw(precision) << usd;
    return oss.str();
}

} // namespace utils
