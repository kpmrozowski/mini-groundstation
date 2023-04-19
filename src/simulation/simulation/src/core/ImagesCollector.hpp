#ifndef ImagesCollector_HPP
#define ImagesCollector_HPP

#include <condition_variable>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

#include <libsgm_wrapper.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "SharedData.hpp"
#include "utils.hpp"

enum class CameraDirection : uint8_t { FRONT, REAR, RIGHT, LEFT, UP, DOWN };

class ImagesCollector : protected SharedData {
public:
    struct StereoImagePacket {
        Perception::ImageInfoType info;
        uint8_t* imageRawBuffer;
        T_OsdkMutexHandle mutex;
        bool gotData;
    };

private:
    std::string m_folderNameFpv = "fpv";
    CameraDirection m_currentStereoDir = CameraDirection::FRONT;
    bool m_subscribedToFpv = false;
    bool m_subscribedToStereo = false;
    uint32_t m_currentImgIdx = -1;
    uint32_t m_lastProcessedImgIdx = -1;
    bool m_terminateSgm = false;
    mutable std::mutex m_sgmMut{};

    DJI::OSDK::Vehicle* p_vehicle;
    sgm::LibSGMWrapper m_sgm;
    std::string m_savePath;

    static std::map<CameraDirection, T_OsdkTaskHandle> m_stereoImageThreads;
    std::map<CameraDirection, StereoImagePacket> m_stereoImages;
    std::map<Perception::CamPositionType, cv::Mat> m_stereoCvImages;
    std::map<Perception::CamPositionType, uint32_t> m_camPosToLastImgIdx;
    std::thread m_sgmThread;
    std::condition_variable m_sgmCondVar;

public:
    ImagesCollector(const Constants& consts, DJI::OSDK::Vehicle* vehicle, const std::string& save_path);
    ImagesCollector() = delete;
    ImagesCollector(const ImagesCollector&) = delete;
    ImagesCollector(ImagesCollector&&) = delete;
    ImagesCollector& operator=(const ImagesCollector&) = delete;
    ImagesCollector& operator=(ImagesCollector&&) = delete;
    ~ImagesCollector();

    void subscribe(const Constants& consts);
    void unsubscribe();
    bool trySubscribeFpv();
    bool trySubscribeStereo(CameraDirection dir);
    bool unsubscribeStereo();
    bool trySubscribeStereoCamParams();
    void changeSubscription(CameraDirection dir);
    std::optional<uint32_t> getStereoImgIdxIfAreTheSame();
    void maybeNotifySgm();
    static void sgmWork(ImagesCollector* self);

    constexpr const auto& savePath() const {
        return m_savePath;
    }

    constexpr const auto& folderNameFpv() const {
        return m_folderNameFpv;
    }

    constexpr const auto& stereoImages() const {
        return m_stereoImages;
    }

    // const auto& stereoImages(CameraDirection dir) const {
    //     printf("const auto& stereoImages(CameraDirection dir).\n");
    //     return m_stereoImages.at(dir);
    // }

    auto& stereoImages(CameraDirection dir) {
        return m_stereoImages.at(dir);
    }

    constexpr auto& stereoCvImages() {
        return m_stereoCvImages;
    }

    constexpr const auto& stereoCvImages() const {
        return m_stereoCvImages;
    }

    auto& stereoCvImages(Perception::CamPositionType camPos) {
        return m_stereoCvImages.at(camPos);
    }

    // const auto& stereoCvImages(Perception::CamPositionType camPos) const {
    //     printf("const auto& stereoCvImages(Perception::CamPositionType camPos).\n");
    //     return m_stereoCvImages.at(camPos);
    // }

    const constexpr auto& camPosToLastImgIdx() const {
        return m_camPosToLastImgIdx;
    }

    constexpr auto& camPosToLastImgIdx() {
        return m_camPosToLastImgIdx;
    }

    const auto& camPosToLastImgIdx(Perception::CamPositionType camPos) const {
        return m_camPosToLastImgIdx.at(camPos);
    }

    constexpr auto& sgm() {
        return m_sgm;
    }
    constexpr auto& sgmCondVar() {
        return m_sgmCondVar;
    }
    constexpr auto& sgmMut() {
        return m_sgmMut;
    }
    constexpr auto& currentStereoDir() {
        return m_currentStereoDir;
    }
    constexpr auto& currentImgIdx() {
        return m_currentImgIdx;
    }
    constexpr auto& lastProcessedImgIdx() {
        return m_lastProcessedImgIdx;
    }
    constexpr auto& terminateSgm() {
        return m_terminateSgm;
    }

    static void saveFpvTask(CameraRGBImage img, void* p);

    // template <CameraDirection DIR>
    // static void saveStereo(CameraRGBImage img, void* p);

    template <CameraDirection DIR>
    static void* stereoImagesDisplayTask(void* arg);
};

#endif  // ImagesCollector_HPP
