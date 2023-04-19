#ifndef DataAggregator_HPP
#define DataAggregator_HPP

#include "Settings.hpp"

struct CollectedData;
struct Constants;

class DataAggregator {
public:
    static std::map<FolderName_t, std::map<Timestamp_t, ImagePath_t>> findFpvPaths(
        const std::unordered_map<FolderName_t, std::vector<ImagePath_t>>& flights_map, const Constants& consts);

    static std::map<FolderName_t, std::map<Timestamp_t, std::vector<ImagePath_t>>> updateStereoTimestamps(
        const std::map<FolderName_t, std::map<Timestamp_t, std::vector<ImagePath_t>>>& stereo_paths);

    static std::map<FolderName_t, std::map<Timestamp_t, FrameRecord>> mergeFpvWithStereoPaths(
        const std::map<FolderName_t, std::map<Timestamp_t, ImagePath_t>>& fpv_paths_map,
        const std::map<FolderName_t, std::map<Timestamp_t, std::vector<ImagePath_t>>>& stereo_paths_map);

    static std::map<FolderName_t, std::map<Timestamp_t, FrameRecord>> mergeFpvWithStereoPaths(
        const std::map<FolderName_t, std::map<Timestamp_t, std::vector<ImagePath_t>>>& stereo_paths_map);

    static std::map<FolderName_t, std::map<Timestamp_t, FrameRecord>> mergeTelemetryWithStereoPaths(
        const std::map<FolderName_t, std::map<Timestamp_t, std::vector<ImagePath_t>>>& stereo_paths_map,
        const std::map<int, CollectedData>& telemetry);
};

#endif  // DataAggregator_HPP
