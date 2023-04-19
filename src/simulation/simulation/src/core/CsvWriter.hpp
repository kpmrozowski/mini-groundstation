#ifndef CsvWriter_HPP
#define CsvWriter_HPP

#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#include "SharedData.hpp"
#include "CollectedData.hpp"
#include "Constants.hpp"

class CsvWriter : public SharedData {
    std::mutex m_mut;
    std::thread m_csvSavingThread;
    std::ofstream m_csvFile;
    CollectedData m_lastData;
    std::chrono::duration<float, std::chrono::seconds::period> m_loopSleepTime;

    bool m_terminateLoop = false;

public:
    CsvWriter(const std::string& save_path, uint16_t csvRefreshRate);
    CsvWriter() = delete;
    CsvWriter(const CsvWriter&) = delete;
    CsvWriter(CsvWriter&&) = delete;
    CsvWriter& operator=(const CsvWriter&) = delete;
    CsvWriter& operator=(CsvWriter&&) = delete;
    ~CsvWriter();

    void initCsv();
    void append();
    void save_csv_in_a_loop();
    void start_loop();
};

#endif // CsvWriter_HPP
