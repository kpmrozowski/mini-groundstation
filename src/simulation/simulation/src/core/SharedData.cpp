#include "SharedData.hpp"
#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

Constants SharedData::s_consts;
CollectedData SharedData::s_data;
std::array<bool, g_numTelemetryCallbacks> SharedData::s_updatedTelemetry;

void Constants::initWith(const std::string& configPath) {
    fmt::print("initWith(const std::string& configPath), {}\n", configPath.c_str());
    try {
        const YAML::Node cfg = YAML::LoadFile(configPath);
        fmt::print("after YAML::LoadFile\n");
        initWithYaml(cfg);
    } catch (const std::exception& e) {
        fmt::print("Exception while parsing yaml: {}\n", e.what());
    } catch (...) {
        fmt::print("Exception while parsing yaml.\n");
    }
}
 
void Constants::initWithYaml(const YAML::Node& config) {
    // m_safetyDistance = config["safetyDistance"].as<float>();
    // m_stabilizingDuration = std::chrono::duration<float, std::chrono::seconds::period>(config["stabilizingDuration"].as<float>());

    // general settings
    m_userConfigPath = config["userConfigPath"].as<std::string>();
    fmt::print("userConfigPath: {}\n", m_userConfigPath);

    m_dataPath = config["dataPath"].as<std::string>();
    fmt::print("dataPath: {}\n", m_dataPath);

    m_outputFolderName = config["outputFolderName"].as<std::string>();
    fmt::print("outputFolderName: {}\n", m_outputFolderName);

    m_inputFpvFolderName = config["inputFpvFolderName"].as<std::string>();
    fmt::print("inputFpvFolderName: {}\n", m_inputFpvFolderName);

    m_inputStereoFolderName = config["inputStereoFolderName"].as<std::string>();
    fmt::print("inputStereoFolderName: {}\n", m_inputStereoFolderName);

    m_framesToSkip = config["framesToSkip"].as<size_t>();
    fmt::print("framesToSkip: {}\n", m_framesToSkip);

    m_printDebug = config["printDebug"].as<bool>();
    fmt::print("printDebug: {}\n", m_printDebug);

    // CSV settings
    m_shallStartCsvWriter = config["CSV"]["startCsvWriter"].as<bool>();
    fmt::print("startCsvWriter: {}\n", m_shallStartCsvWriter);

    m_csvRefreshRate = config["CSV"]["csvRefreshRate"].as<uint16_t>();
    fmt::print("csvRefreshRate: {}\n", m_csvRefreshRate);

    m_csvSkipSameValues = config["CSV"]["csvSkipSameValues"].as<bool>();
    fmt::print("csvSkipSameValues: {}\n", m_csvSkipSameValues);

    m_subscribeToBattery = config["CSV"]["subscribeToBattery"].as<bool>();
    fmt::print("subscribeToBattery: {}\n", m_subscribeToBattery);

    m_subscribeToTelemetry = config["CSV"]["subscribeToTelemetry"].as<bool>();
    fmt::print("subscribeToTelemetry: {}\n", m_subscribeToTelemetry);

    m_subscribeToTimers = config["CSV"]["subscribeToTimers"].as<bool>();
    fmt::print("subscribeToTimers: {}\n", m_subscribeToTimers);

    // ImageStream settings
    m_subscribeToStereoParams = config["ImageStream"]["subscribeToStereoParams"].as<bool>();
    fmt::print("subscribeToStereoParams: {}\n", m_subscribeToStereoParams);

    m_subscribeToFpv = config["ImageStream"]["subscribeToFpv"].as<bool>();
    fmt::print("subscribeToFpv: {}\n", m_subscribeToFpv);

    m_subscribeToStereo = config["ImageStream"]["subscribeToStereo"].as<bool>();
    fmt::print("subscribeToStereo: {}\n", m_subscribeToStereo);

    m_computeDepth = config["ImageStream"]["computeDepth"].as<bool>();
    fmt::print("computeDepth: {}\n", m_computeDepth);

    m_computeDisparity = config["ImageStream"]["computeDisparity"].as<bool>();
    fmt::print("computeDisparity: {}\n", m_computeDisparity);

    m_computeColor = config["ImageStream"]["computeColor"].as<bool>();
    fmt::print("computeColor: {}\n", m_computeColor);

    m_computePointCloud = config["ImageStream"]["computePointCloud"].as<bool>();
    fmt::print("computePointCloud: {}\n", m_computePointCloud);

    m_computeSLAM = config["ImageStream"]["computeSLAM"].as<bool>();
    fmt::print("computeSLAM: {}\n", m_computeSLAM);

    // SGM settings
    m_dispSize = config["SGM"]["disparitySize"].as<int>();
    fmt::print("dispSize: {}\n", m_dispSize);

    m_p1 = config["SGM"]["p1"].as<int>();
    fmt::print("p1: {}\n", m_p1);

    m_p2 = config["SGM"]["p2"].as<int>();
    fmt::print("p2: {}\n", m_p2);

    m_uniqueness = config["SGM"]["uniqueness"].as<float>();
    fmt::print("uniqueness: {}\n", m_uniqueness);

    m_subpixel = config["SGM"]["subpixel"].as<bool>();
    fmt::print("subpixel: {}\n", m_subpixel);

    int numPaths = config["SGM"]["numPaths"].as<int>();
    fmt::print("numPaths: {}\n", numPaths);
    switch (numPaths) {
    case 4:
        m_pathType = sgm::PathType::SCAN_4PATH;
        break;
    case 8:
        m_pathType = sgm::PathType::SCAN_8PATH;
        break;
    default:
        throw std::invalid_argument("numPaths can be equal 4 or 8");
    }

    m_minDisp = config["SGM"]["minimalDisparity"].as<int>();
    fmt::print("minimalDisparity: {}\n", m_minDisp);

    m_lrMaxDiff = config["SGM"]["lrMaxDiff"].as<int>();
    fmt::print("lrMaxDiff: {}\n", m_lrMaxDiff);

    int censusTypeId = config["SGM"]["censusTypeId"].as<int>();
    fmt::print("censusTypeId: {}\n", censusTypeId);
    switch (censusTypeId) {
    case 0:
        m_censusType = sgm::CensusType::CENSUS_9x7;
        break;
    case 1:
        m_censusType = sgm::CensusType::SYMMETRIC_CENSUS_9x7;
        break;
    default:
        throw std::invalid_argument("censusType can be equal 0 or 1");
    }

    // ORB_SLAM3 settings
    m_useGui = config["ORB_SLAM3"]["useGui"].as<bool>();
    fmt::print("useGui: {}\n", m_useGui);


    m_initiated = true;
}
