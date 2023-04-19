#ifndef SharedData_HPP
#define SharedData_HPP

#include "CollectedData.hpp"
#include "Constants.hpp"

class SharedData {
  protected:
    // constants
    static Constants s_consts;
    // variables
    static CollectedData s_data;
    static std::array<bool, g_numTelemetryCallbacks> s_updatedTelemetry;
};

#endif // SharedData_HPP
