#include "utils.hpp"
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
#include <fstream>

#include "fmt/format.h"
#include "spdlog/spdlog.h"

#include "Settings.hpp"

namespace utils {

static std::optional<std::array<std::string, 2>> findLeftRightPaths(const std::vector<std::string>& images_pair,
                                                                    const Settings& s) {
    ImagePath_t left_path{}, right_path{};
    size_t left_count{0}, right_count{0};
    for (const ImagePath_t& image_path : images_pair) {
        if (std::string::npos != image_path.find(s.left_image_id)) {
            left_path = image_path;
            ++left_count;
        }
        if (std::string::npos != image_path.find(s.right_image_id)) {
            right_path = image_path;
            ++right_count;
        }
    }
    if (left_count != right_count) {
        fmt::print("error: found {} left images and {} right images\n", left_count, right_count);
        return std::nullopt;
    }
    return {{left_path, right_path}};
}

std::map<FolderName_t, std::map<Timestamp_t, std::vector<ImagePath_t>>> findStereoPairsByTimestamp(
    std::unordered_map<FolderName_t, std::vector<ImagePath_t>>& image_paths_map, const Settings& s, const Constants& consts) {
    std::map<FolderName_t, std::map<Timestamp_t, std::vector<ImagePath_t>>> images;

    for (auto& [folder_name, image_paths] : image_paths_map) {
        std::string dataset_path = consts.dataPath() + "/" + folder_name;
        FolderName_t depth_folder_path = dataset_path + "/" + consts.inputStereoFolderName();
        for (const auto& entry : fs::directory_iterator(depth_folder_path)) {
            // skip directories
            if (entry.is_directory()) {
                fmt::print("skipping directory: '{}'\n", entry.path().c_str());
                continue;
            }

            // skip paths containing pattern
            bool skip_entry = false;
            for (std::string pattern : s.patterns_to_skip) {
                if (std::string::npos != entry.path().filename().string().find(pattern)) {
                    skip_entry = true;
                    break;
                }
            }
            if (skip_entry) {
                continue;
            }
            image_paths.push_back(entry.path().c_str());

            ImagePath_t image_path = entry.path().string();
            size_t timestamp_begin = image_path.find(s.timestamp_pattern);
            if (std::string::npos == timestamp_begin) {
                throw std::invalid_argument(
                    fmt::format("'{}' not found in the string: {}\n", s.timestamp_pattern, image_path).c_str());
            }
            timestamp_begin += s.timestamp_pattern.size();
            std::string timestamp_str = image_path.substr(timestamp_begin, s.timestamp_length);
            Timestamp_t timestamp{};
            try {
                timestamp = std::stoull(timestamp_str);
            } catch (const std::invalid_argument& e) {
                fmt::print("error: {}\n", e.what());
                fmt::print("str: {}\n", timestamp_str);
            }
            // fmt::print("timestamp: {}, {}\n", timestamp, timestamp_str);
            images[folder_name][timestamp].push_back(image_path);
        }
        fmt::print("images[{}].size = {}; ", folder_name, images[folder_name].size());
        // removing unpaired elements
        std::vector<size_t> timestamps_to_remove;
        for (auto& [timestamp, images_pair] : images[folder_name]) {
            if (2 != images_pair.size()) {
                timestamps_to_remove.push_back(timestamp);
            }
        }
        fmt::print("timestamps_to_remove.size = {}; ", timestamps_to_remove.size());
        for (size_t timestamp : timestamps_to_remove) {
            images[folder_name].erase(timestamp);
        }
        size_t swaps_count = 0;
        fmt::print("images[{}].size = {}; ", folder_name, images[folder_name].size());
        for (auto& [timestamp, images_pair] : images[folder_name]) {
            // find left and right image
            const auto ret = findLeftRightPaths(images_pair, s);
            if (not ret.has_value()) {
                throw std::runtime_error(
                    fmt::format("in '{}' two or more images of the same timestamp {}", folder_name, timestamp));
            }
            const auto [left_path, right_path] = ret.value();
            if (images_pair[0] != left_path) {
                ++swaps_count;
                std::swap(images_pair[0], images_pair[1]);
            }
        }
        fmt::print("swaps count = {}\n", swaps_count);
    }
    return images;
}

void removeDirectory(const std::string& path) {
    if (fs::exists(path)) {
        fs::remove_all(path);
    } else {
        fmt::print("directory '{}' does not exist\n", path);
    }
}

void createDirectory(const std::string& path) {
    if (not fs::exists(path)) {
        fs::create_directories(path);
    } else {
        fmt::print("directory '{}' exists\n", path);
    }
}
void createDirectories(std::vector<std::string>&& paths) {
    for (const std::string& path : paths) {
        createDirectory(path);
    }
}

nlohmann::json readJson(const std::string& path) {
    std::ifstream file(path);
    if (file.fail()) {
        spdlog::error("Failed to open (read) {}", path);
        std::exit(EXIT_FAILURE);
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    nlohmann::json data = nlohmann::json::parse(buffer.str());
    return data;
}

} // namespace utils
