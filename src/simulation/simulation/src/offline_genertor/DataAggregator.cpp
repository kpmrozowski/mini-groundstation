#include "DataAggregator.hpp"
#include "Constants.hpp"

#ifndef __has_include
static_assert(false, "__has_include not supported");
#else
#    if __cplusplus >= 201703L && __has_include(<filesystem>)
#        include <filesystem>
namespace fs = std::filesystem;
#    elif __has_include(<experimental/filesystem>)
#        include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#    endif
#endif

#include <fmt/format.h>

#include "CollectedData.hpp"

static const FolderName_t g_supported_flight = "_Radziwie7/1_flight_20230210-120154";
static constexpr size_t g_sec_to_microsec = 10e6;

struct CollectedData;

template <typename T>
static constexpr T absolute(T val) {
    if (val > 0) {
        return val;
    } else {
        return -val;
    }
    return {0};
}

static Timestamp_t getTimestamp(std::string ts_str) {
    // ts_str equals f.e.: "20230210120155.887"
    // printf("ts_str='%s'", ts_str.c_str());
    size_t ts = 0, pos = 0;
    ;
    if (ts_str.size() == 18) {
        pos = 8;
    }
    ts += 3600 * 1000 * std::stoul(ts_str.substr(pos, 2).c_str());
    ts += 60 * 1000 * std::stoul(ts_str.substr(pos + 2, 2).c_str());
    ts += 1000 * std::stoul(ts_str.substr(pos + 4, 2).c_str());
    ts += std::stoul(ts_str.substr(pos + 7, 3).c_str());
    // printf("\tts=%zu\n", ts);
    return ts;
}

std::map<FolderName_t, std::map<Timestamp_t, ImagePath_t>> DataAggregator::findFpvPaths(
    const std::unordered_map<FolderName_t, std::vector<ImagePath_t>>& flights_map, const Constants& consts) {
    std::map<FolderName_t, std::map<Timestamp_t, ImagePath_t>> fpv_paths_map;

    for (const auto& [folder_name, timestamp_img_pair_map] : flights_map) {
        std::string dataset_path = consts.dataPath() + "/" + folder_name;
        const FolderName_t fpv_folder_path = dataset_path + "/" + consts.inputFpvFolderName();
        if (not fs::exists(fpv_folder_path)) {
            throw std::runtime_error(fmt::format("directory '{}' does not exist", fpv_folder_path));
        } else {
            fmt::print("directory {} exists\n", fpv_folder_path);
        }
        for (const auto& entry : fs::directory_iterator(fpv_folder_path)) {
            // fmt::print("{}\n", entry.path().c_str());
            const std::string file_path = entry.path().string();
            const std::string parent_dir = entry.path().parent_path().parent_path();
            const std::string date_pattern = "flight_";
            const size_t date_begin = parent_dir.find(date_pattern);
            if (std::string::npos == date_begin) {
                throw std::runtime_error(fmt::format("'{}' has no 'flight_' substring", file_path));
            }
            const std::string date_str = parent_dir.substr(date_begin + date_pattern.size(), 8);
            // fmt::print("'{}'\n", date_str);

            const size_t stamp_begin = file_path.find(date_str, date_begin + date_pattern.size() + 8);
            if (std::string::npos == stamp_begin) {
                throw std::runtime_error(fmt::format("'{0}' has no '{1}' substring after {1}", file_path, date_str));
            }
            size_t stamp_end = file_path.find("_fpv.png", stamp_begin + date_str.size());
            if (std::string::npos == stamp_end) {
                stamp_end = file_path.find(".png", stamp_begin);
            }
            if (std::string::npos == stamp_end) {
                throw std::runtime_error(fmt::format("'{}' has no '_fpv.png' nor '.png' substrings", file_path));
            }
            const std::string stamp_str =
                file_path.substr(stamp_begin + date_str.size() + 1, stamp_end - (stamp_begin + date_str.size() + 1));
            // fmt::print("'{}'\n", stamp_str);
            size_t timestamp = getTimestamp(stamp_str);
            // fmt::print("'{}'\n", timestamp);
            fpv_paths_map[folder_name][timestamp] = file_path;
        }
        // fmt::print("\n");
    }
    return fpv_paths_map;
}

std::map<FolderName_t, std::map<Timestamp_t, std::vector<ImagePath_t>>> DataAggregator::updateStereoTimestamps(
    const std::map<FolderName_t, std::map<Timestamp_t, std::vector<ImagePath_t>>>& stereo_paths) {
    std::map<FolderName_t, std::map<Timestamp_t, std::vector<ImagePath_t>>> new_stereo_paths;

    for (const auto& [folder_name, stereo_timestamp_path_pairs] : stereo_paths) {
        if (folder_name != g_supported_flight) {
            continue;
        }
        new_stereo_paths[folder_name] = std::map<Timestamp_t, std::vector<ImagePath_t>>{};
        for (const auto& [stereo_timestamp, stereo_paths] : stereo_timestamp_path_pairs) {
            // fmt::print("stereo: {}, {}\n", stereo_timestamp, stereo_path[0]);
            size_t date_begin = stereo_paths[0].find_last_of("/");
            date_begin = stereo_paths[0].find("-", date_begin + 1);
            size_t date_end = stereo_paths[0].find("-idx_", date_begin + 1);
            if (std::string::npos == date_end) {
                printf("err: ");
            }
            std::string stamp_str = stereo_paths[0].substr(date_begin + 1, date_end - (date_begin + 1));
            // fmt::print("stamp_str={}\n", stamp_str);

            size_t stamp = getTimestamp(stamp_str);
            // fmt::print("{}, {}\n", stamp, stereo_paths[0]);
            new_stereo_paths[folder_name][stamp] = stereo_paths;
        }
    }
    return new_stereo_paths;
}

std::map<FolderName_t, std::map<Timestamp_t, FrameRecord>> DataAggregator::mergeFpvWithStereoPaths(
    const std::map<FolderName_t, std::map<Timestamp_t, ImagePath_t>>& fpv_paths_map,
    const std::map<FolderName_t, std::map<Timestamp_t, std::vector<ImagePath_t>>>& stereo_paths_map) {
    std::map<FolderName_t, std::map<Timestamp_t, FrameRecord>> fpv_stereo_paths_map;
    size_t pairs_found = 0;
    size_t pairs_not_found = 0;
    // (fpv_offeset, diff_tolerance): pairs_found
    // ( 0.006, 0.11): 28669 <-- +746
    // ( 0.006, 0.10): 27923 <-- +1671
    // ( 0.006, 0.09): 26252 <-- +2197 (chosen config)
    // ( 0.006, 0.08): 24055 <-- +2378
    // ( 0.006, 0.07): 21677 <-- +2732
    // ( 0.006, 0.06): 18945 <-- +2921
    // ( 0.006, 0.05): 16024 <-- +3138
    // ( 0.006, 0.04): 12886

    // ( 0.009, 0.03): 9664
    // ( 0.008, 0.03): 9679
    // ( 0.007, 0.03): 9704
    // ( 0.006, 0.03): 9709 <-- maximum
    // ( 0.005, 0.03): 9705
    // ( 0.004, 0.03): 9638
    // ( 0.003, 0.03): 9676
    // ( 0.002, 0.03): 9684
    // ( 0.001, 0.03): 9667
    // ( 0.000, 0.03): 9596
    // (-0.001, 0.03): 9565
    // (-0.002, 0.03): 9496
    // (-0.003, 0.03): 9454
    constexpr int fpv_offeset = static_cast<int>(0.006 * g_sec_to_microsec);
    constexpr int diff_tolerance = static_cast<int>(0.09 * g_sec_to_microsec);

    const auto& stereo_timestamp_path_pairs = stereo_paths_map.at(g_supported_flight);
    int ii = 0;
    for (const auto& [stereo_timestamp, stereo_paths] : stereo_timestamp_path_pairs) {
        fmt::print("stereoL: {}, {}\n", stereo_timestamp, stereo_paths[0]);
        fmt::print("stereoR: {}, {}\n", stereo_timestamp, stereo_paths[1]);
        if (++ii > 4) {
            break;
        }
    }
    const auto& fpv_timestamp_path_pairs = fpv_paths_map.at(g_supported_flight);
    ii = 0;
    for (const auto& [fpv_timestamp, fpv_path] : fpv_timestamp_path_pairs) {
        fmt::print("fpv: {}, {}\n", fpv_timestamp, fpv_path);
        if (++ii > 4) {
            break;
        }
    }

    auto it_fpv = fpv_timestamp_path_pairs.begin();
    auto it_stereo = stereo_timestamp_path_pairs.begin();
    while (true) {
        if (absolute((it_fpv->first - fpv_offeset) - it_stereo->first) < diff_tolerance) {
            pairs_found += 1;
            it_fpv++;
            it_stereo++;

            FrameRecord frame;
            frame.fpv = it_fpv->second;
            frame.stereo_l = it_stereo->second[0];
            frame.stereo_r = it_stereo->second[1];
            frame.imu_meas = std::vector<ImuMeas>{};

            fpv_stereo_paths_map[g_supported_flight][it_stereo->first] = std::move(frame);
        } else {
            pairs_not_found += 1;
            // fmt::print("not found: time_fpv: {}, time_stereo: {}\n", it_fpv->first, it_stereo->first);
            if ((it_fpv->first - fpv_offeset) < it_stereo->first) {
                it_fpv++;
            } else {
                it_stereo++;
            }
        }
        if (it_fpv == fpv_timestamp_path_pairs.end() or it_stereo == stereo_timestamp_path_pairs.end()) {
            break;
        }
    }
    fmt::print("pairs_found: {}\n", pairs_found);
    fmt::print("pairs_not_found: {}\n", pairs_not_found);
    return fpv_stereo_paths_map;
}

std::map<FolderName_t, std::map<Timestamp_t, FrameRecord>> DataAggregator::mergeFpvWithStereoPaths(
    const std::map<FolderName_t, std::map<Timestamp_t, std::vector<ImagePath_t>>>& stereo_paths_map) {
    std::map<FolderName_t, std::map<Timestamp_t, FrameRecord>> fpv_stereo_paths_map;

    const auto& stereo_timestamp_path_pairs = stereo_paths_map.at(g_supported_flight);

    for (auto it = stereo_timestamp_path_pairs.begin(); it != stereo_timestamp_path_pairs.end(); ++it) {
        FrameRecord frame;
        frame.fpv = "";
        frame.stereo_l = it->second[0];
        frame.stereo_r = it->second[1];
        frame.imu_meas = std::vector<ImuMeas>{};
        fpv_stereo_paths_map[g_supported_flight][it->first] = std::move(frame);
    }
    return fpv_stereo_paths_map;
}

// template <typename T>
// static T interpolate(T val1, T val2, Timestamp_t ts1, Timestamp_t ts2, Timestamp_t target_ts) {
//     T target_val = val1 + (val2 - val1) / static_cast<double>(ts2 - ts1) * target_ts;
//     fmt::print("dbg target_val: {}\n", target_val);
//     return target_val;
// }

// static CollectedData interpolate(const Timestamp_t stereo_ts, const CollectedData& data_ts1,
//                                  const CollectedData& data_ts2) {
//     CollectedData data_interpolated{};

//     Timestamp_t ts1 = getTimestamp(data_ts1.timestamp);
//     Timestamp_t ts2 = getTimestamp(data_ts2.timestamp);

//     data_interpolated.rtkPosition.HFSL =
//         interpolate(data_ts1.rtkPosition.HFSL, data_ts2.rtkPosition.HFSL, ts1, ts2, stereo_ts);

//     return data_interpolated;
// }

std::map<FolderName_t, std::map<Timestamp_t, FrameRecord>> DataAggregator::mergeTelemetryWithStereoPaths(
    const std::map<FolderName_t, std::map<Timestamp_t, std::vector<ImagePath_t>>>& stereo_paths_map,
    const std::map<int, CollectedData>& telemetry) {
    std::map<FolderName_t, std::map<Timestamp_t, FrameRecord>> stereo_telemetry_map;

    const auto& stereo_timestamp_path_pairs = stereo_paths_map.at(g_supported_flight);

    Timestamp_t stereo_ts = stereo_timestamp_path_pairs.begin()->first;
    Timestamp_t last_stereo_ts = stereo_ts;  // debug
    auto record_it1 = telemetry.begin();
    auto record_it2 = telemetry.begin();

    for (auto stereo_it = stereo_timestamp_path_pairs.begin(); stereo_it != stereo_timestamp_path_pairs.end();
         ++stereo_it) {
        stereo_ts = stereo_it->first;
        if (stereo_ts < getTimestamp(telemetry.begin()->second.timestamp)) {
            continue;
        }

        bool is_last_telemetry_record = true;
        for (auto it = record_it1; it != telemetry.end(); ++it) {
            record_it2 = it;
            Timestamp_t record_ts = getTimestamp(it->second.timestamp);
            if (record_ts < stereo_ts) {
                continue;
            }
            is_last_telemetry_record = false;
            break;
        }
        if (is_last_telemetry_record) {
            fmt::print("is_last_telemetry_record\n");
            break;
        }

        auto prev_record_it1 = record_it1;
        // for (int i = 0; i < 10; ++i) {
        //     if (prev_record_it1 != telemetry.begin()) {
        //         --prev_record_it1;
        //     }
        // }
        auto next_record_it2 = record_it2;
        // for (int i = 0; i < 10; ++i) {
        //     if (next_record_it2 != telemetry.end()) {
        //         ++next_record_it2;
        //     }
        // }

        std::vector<ImuMeas> imu_meas_vec;
        for (auto telemetry_it = prev_record_it1; telemetry_it != next_record_it2; ++telemetry_it) {
            ImuMeas imu_meas;
            imu_meas.acc = cv::Point3f{
                telemetry_it->second.accelerationRaw.x,
                telemetry_it->second.accelerationRaw.y,
                telemetry_it->second.accelerationRaw.z,
            };
            imu_meas.gyr = cv::Point3f{
                telemetry_it->second.angularRateFusioned.x,
                telemetry_it->second.angularRateFusioned.y,
                telemetry_it->second.angularRateFusioned.z,
            };
            imu_meas.timestamp = static_cast<double>(getTimestamp(telemetry_it->second.timestamp)) / 1000.;
            imu_meas_vec.emplace_back(std::move(imu_meas));
        }
        record_it1 = record_it2;

        if (imu_meas_vec.size() < 2) {
            // fmt::print("imu_meas_vec is empty\n");
            continue;
        }
        // fmt::print("imu_meas_vec[{}]=[", imu_meas_vec.size());
        // for (ImuMeas imu_meas : imu_meas_vec) {
        //     fmt::print("{}, ", static_cast<Timestamp_t>(imu_meas.timestamp * 1000));
        // }
        // fmt::print("], stereo_ts=[{}, {}]\n", last_stereo_ts, stereo_ts);
        last_stereo_ts = stereo_ts;

        FrameRecord frame;
        frame.fpv = "";
        frame.stereo_l = stereo_it->second[0];
        frame.stereo_r = stereo_it->second[1];
        frame.imu_meas = std::move(imu_meas_vec);

        stereo_telemetry_map[g_supported_flight][stereo_it->first] = std::move(frame);
    }
    return stereo_telemetry_map;
}
