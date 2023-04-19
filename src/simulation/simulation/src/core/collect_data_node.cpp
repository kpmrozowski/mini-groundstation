#include "collect_data_node.hpp"

#include <memory>

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <fmt/format.h>

#include "dji_vehicle.hpp"
#include "dji_linux_helpers.hpp"
#include "osdkosal_linux.h"

#include "DjiCollector.hpp"

#include "PointCloudCollector.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int main(int argc, char **argv) {
    ros::init(argc, argv, "collect_data");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    std::string djiConfigPath{"/workspace/src/dji_data/configs/UserConfig.txt"};
    std::string configPath{"/workspace/src/dji_data/configs/config.yaml"};
    std::string logsPath{"/workspace/src/dji_data/logs"};
    bool dryRun = false;
    nhPrivate.getParam("djiConfigPath", djiConfigPath);
    nhPrivate.getParam("configPath", configPath);
    nhPrivate.getParam("logsPath", logsPath);
    nhPrivate.getParam("dryRun", dryRun);

    // auto cfg_ptr = std::make_unique<YAML::Node>(YAML::LoadFile(configPath));
    // Constants consts{cfg_ptr};
    Constants consts{configPath};

    // Setup OSDK.
    std::unique_ptr<LinuxSetup> linuxEnvironment = nullptr;
    Vehicle* vehicle = nullptr;
    if (not dryRun) {
        // const std::unique_ptr<char> binaryNameConstPtr = std::unique_ptr<char>(const_cast<char*>("collect_data"));
        static constexpr int dji_argc = 2;
        char* dji_argv[dji_argc] = {
            "collect_data",
            const_cast<char*>(djiConfigPath.c_str())
        };
        linuxEnvironment = std::make_unique<LinuxSetup>(dji_argc, dji_argv);
        vehicle = linuxEnvironment->getVehicle();

        if (nullptr == vehicle) {
            std::cout << "Vehicle not initialized, exiting.\n";
            return -1;
        }
        const char *acm_dev = linuxEnvironment->getEnvironment()->getDeviceAcm().c_str();
        vehicle->advancedSensing->setAcmDevicePath(acm_dev);

        // Obtain Control Authority
        const int functionTimeout = 1;
        vehicle->control->obtainCtrlAuthority(functionTimeout);
    }

    vehicle->control->releaseCtrlAuthority(1);

    PointCloudCollector pointCloudCollector{consts};

    DjiCollector djiCollector{std::move(consts), vehicle};
    printf("DjiCollector finished!\n");
    // while (true) {
    //     fmt::print("sleep\n");
    //     usleep(1000);std::unique_ptr<DjiCollector>
    // }

    ros::Rate rate(24.);
    while(ros::ok())
    {
        rate.sleep();
    }
    printf("IT'S MILLER TIME! :D");
    return 0;
}
