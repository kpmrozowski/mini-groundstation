#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>

namespace utils {

std::string time_in_YYYY_MM_DD_HH_MM_SS();
std::string time_in_YYYY_MM_DD_HH_MM_SS_MMM(int precision);
std::string time_in_YYYYMMDDHHMMSSMMM(int precision);

template <typename Key, typename Value, std::size_t Size>
struct Map {
  std::array<std::pair<Key, Value>, Size> data;

  [[nodiscard]] constexpr Value at(const Key &key) const {
    const auto itr =
        std::find_if(begin(data), end(data),
                     [&key](const auto &v) { return v.first == key; });
    if (itr != end(data)) {
      return itr->second;
    } else {
      throw std::range_error("no such key in Map");
    }
  }

};

} // namespace utils
