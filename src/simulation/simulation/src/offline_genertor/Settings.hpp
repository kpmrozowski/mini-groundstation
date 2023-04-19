#ifndef Settings_HPP
#define Settings_HPP

#include <array>

#include "utils.hpp"

struct Settings {
    static constexpr int invalid_disparity_value = 65520;

    const std::string configPath = "/workspace/src/dji_data/configs/config.yaml";

    const std::array<int, 2> stereo_resolution{640, 480};
    static constexpr int stereo_bits = 8;
    static constexpr int disparity_bits = 16;

    // strings to search
    const FolderName_t output_disparity_folder_name = "o-disparity";
    const FolderName_t output_depth_folder_name = "o-depth";
    const FolderName_t output_color_folder_name = "o-color";
    const FolderName_t output_pc_folder_name = "o-point_cloud";
    const FolderName_t output_tracked_points_file_name = "o-points_tracked";
    const FolderName_t output_map_points_file_name = "o-points_all";
    const FolderName_t output_trajectory_file_name = "o-trajectory";
    const FolderName_t output_kp_trajectory_file_name = "o-trajectory_kp";
    const std::string timestamp_pattern = "-ts_";
    static constexpr size_t timestamp_length = 10;
    const std::string left_image_id = "id_58";
    const std::string right_image_id = "id_59";
    const std::string stereo_file_format = ".png";
    const std::string disparity_file_format = ".tiff";
    const std::string pc_file_format = ".ply";

    const std::vector<std::string> patterns_to_skip = {
        "disparity",
    };
};



#endif  // Settings_HPP
