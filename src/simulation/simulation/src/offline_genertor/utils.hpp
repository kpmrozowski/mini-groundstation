#ifndef utils_HPP
#define utils_HPP

#include <map>
#include <optional>
#include <unordered_map>
#include <vector>

#include "nlohmann/json.hpp"

#include "types.hpp"

struct Settings;
struct Constants;

namespace utils {

std::map<FolderName_t, std::map<Timestamp_t, std::vector<ImagePath_t>>> findStereoPairsByTimestamp(
    std::unordered_map<FolderName_t, std::vector<ImagePath_t>>& image_paths_map, const Settings& s, const Constants& consts);

void removeDirectory(const std::string& path);

void createDirectory(const std::string& path);

void createDirectories(std::vector<std::string>&& paths);

nlohmann::json readJson(const std::string& path);

} // namespace utils

#endif  // utils_HPP
