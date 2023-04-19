#ifndef CsvReader_HPP
#define CsvReader_HPP

#include <map>

#include "SharedData.hpp"

class CsvReader {
public:
    /**
     * @brief Simple reader of csv format
     * ID | values
     */
    static std::map<int, CollectedData> readCsvRaw(const std::string& path);
};

#endif  // CsvReader_HPP
