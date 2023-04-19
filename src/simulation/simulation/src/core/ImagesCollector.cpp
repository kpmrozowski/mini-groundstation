// Now the direction [(DirectionType)0] images are updating.
// Only the SINGLE direction subscription is allowed in current OSDK version.
// Please do unsubscription firstly before do new subscribing

#include "ImagesCollector.hpp"

#include <libsgm.h>
#include <opencv2/core/cuda.hpp>
#include <dji_vehicle.hpp>
#include <fmt/format.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string_view>

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

using namespace std::literals::string_literals;

static constexpr std::array<std::pair<CameraDirection, std::string_view>, 6> g_stereoFolderNamesValues{
    {{CameraDirection::FRONT, "StereoFront"sv},
     {CameraDirection::REAR, "StereoRear"sv},
     {CameraDirection::RIGHT, "StereoRight"sv},
     {CameraDirection::LEFT, "StereoLeft"sv},
     {CameraDirection::UP, "StereoUp"sv},
     {CameraDirection::DOWN, "StereoDown"sv}}};
static constexpr auto g_stereoFolderNames =
    utils::Map<CameraDirection, std::string_view, g_stereoFolderNamesValues.size()>{{g_stereoFolderNamesValues}};

static constexpr std::array<std::pair<Perception::CamPositionType, std::string_view>, 12> g_camPosToPosNameValues{
    {{Perception::CamPositionType::RECTIFY_DOWN_LEFT, "DownLeft"sv},
     {Perception::CamPositionType::RECTIFY_DOWN_RIGHT, "DownRight"sv},
     {Perception::CamPositionType::RECTIFY_FRONT_LEFT, "FrontLeft"sv},
     {Perception::CamPositionType::RECTIFY_FRONT_RIGHT, "FrontRight"sv},
     {Perception::CamPositionType::RECTIFY_REAR_LEFT, "RearLeft"sv},
     {Perception::CamPositionType::RECTIFY_REAR_RIGHT, "RearRight"sv},
     {Perception::CamPositionType::RECTIFY_UP_LEFT, "UpLeft"sv},
     {Perception::CamPositionType::RECTIFY_UP_RIGHT, "UpRight"sv},
     {Perception::CamPositionType::RECTIFY_LEFT_LEFT, "LeftLeft"sv},
     {Perception::CamPositionType::RECTIFY_LEFT_RIGHT, "LeftRight"sv},
     {Perception::CamPositionType::RECTIFY_RIGHT_LEFT, "RightLeft"sv},
     {Perception::CamPositionType::RECTIFY_RIGHT_RIGHT, "RightRight"sv}}};
static constexpr auto g_camPosToPosName =
    utils::Map<Perception::CamPositionType, std::string_view, g_camPosToPosNameValues.size()>{
        {g_camPosToPosNameValues}};

static constexpr std::array<std::pair<CameraDirection, Perception::DirectionType>, 6> g_stereoDirToImgTypeValues{
    {{CameraDirection::FRONT, Perception::DirectionType::RECTIFY_FRONT},
     {CameraDirection::REAR, Perception::DirectionType::RECTIFY_REAR},
     {CameraDirection::RIGHT, Perception::DirectionType::RECTIFY_RIGHT},
     {CameraDirection::LEFT, Perception::DirectionType::RECTIFY_LEFT},
     {CameraDirection::UP, Perception::DirectionType::RECTIFY_UP},
     {CameraDirection::DOWN, Perception::DirectionType::RECTIFY_DOWN}}};
static constexpr auto g_stereoDirToImgType =
    utils::Map<CameraDirection, Perception::DirectionType, g_stereoDirToImgTypeValues.size()>{
        {g_stereoDirToImgTypeValues}};

static constexpr std::array<std::pair<Perception::DirectionType, CameraDirection>, 6> g_imgTypeToStereoDirValues{
    {{Perception::DirectionType::RECTIFY_FRONT, CameraDirection::FRONT},
     {Perception::DirectionType::RECTIFY_REAR, CameraDirection::REAR},
     {Perception::DirectionType::RECTIFY_RIGHT, CameraDirection::RIGHT},
     {Perception::DirectionType::RECTIFY_LEFT, CameraDirection::LEFT},
     {Perception::DirectionType::RECTIFY_UP, CameraDirection::UP},
     {Perception::DirectionType::RECTIFY_DOWN, CameraDirection::DOWN}}};
static constexpr auto g_imgTypeToStereoDir =
    utils::Map<Perception::DirectionType, CameraDirection, g_imgTypeToStereoDirValues.size()>{
        {g_imgTypeToStereoDirValues}};

static constexpr std::array<std::pair<CameraDirection, std::array<Perception::CamPositionType, 2>>, 6>
    g_stereoDirToCamPosPairValues{
        {{CameraDirection::FRONT,
          {Perception::CamPositionType::RECTIFY_FRONT_LEFT, Perception::CamPositionType::RECTIFY_FRONT_RIGHT}},
         {CameraDirection::REAR,
          {Perception::CamPositionType::RECTIFY_REAR_LEFT, Perception::CamPositionType::RECTIFY_REAR_RIGHT}},
         {CameraDirection::RIGHT,
          {Perception::CamPositionType::RECTIFY_RIGHT_LEFT, Perception::CamPositionType::RECTIFY_RIGHT_RIGHT}},
         {CameraDirection::LEFT,
          {Perception::CamPositionType::RECTIFY_LEFT_LEFT, Perception::CamPositionType::RECTIFY_LEFT_RIGHT}},
         {CameraDirection::UP,
          {Perception::CamPositionType::RECTIFY_UP_LEFT, Perception::CamPositionType::RECTIFY_UP_RIGHT}},
         {CameraDirection::DOWN,
          {Perception::CamPositionType::RECTIFY_DOWN_LEFT, Perception::CamPositionType::RECTIFY_DOWN_RIGHT}}}};
static constexpr auto g_stereoDirToCamPosPair =
    utils::Map<CameraDirection, std::array<Perception::CamPositionType, 2>, g_stereoDirToCamPosPairValues.size()>{
        {g_stereoDirToCamPosPairValues}};

std::map<CameraDirection, T_OsdkTaskHandle> ImagesCollector::m_stereoImageThreads{};

namespace {

template <CameraDirection DIR>
void perceptionImageCallback(Perception::ImageInfoType info, uint8_t* imageRawBuffer, int bufferLen, void* userData) {
    // DSTATUS("image info : dataId(%d) seq(%d) timestamp(%llu) datatype(%d) index(%d) h(%d) w(%d) dir(%d) "
    //         "bpp(%d) bufferlen(%d)", info.dataId, info.sequence, info.timeStamp, info.dataType,
    //         info.rawInfo.index, info.rawInfo.height, info.rawInfo.width, info.rawInfo.direction,
    //         info.rawInfo.bpp, bufferLen);
    ImagesCollector* self = (ImagesCollector*)userData;
    if (imageRawBuffer && userData) {
        OsdkOsal_MutexLock(self->stereoImages(DIR).mutex);
        self->stereoImages(DIR).info = info;
        if (self->stereoImages(DIR).imageRawBuffer) {
            OsdkOsal_Free(self->stereoImages(DIR).imageRawBuffer);
        }
        self->stereoImages(DIR).imageRawBuffer = (uint8_t*)OsdkOsal_Malloc(bufferLen);
        memcpy(self->stereoImages(DIR).imageRawBuffer, imageRawBuffer, bufferLen);
        self->stereoImages(DIR).gotData = true;
        OsdkOsal_MutexUnlock(self->stereoImages(DIR).mutex);
    }
}

void perceptionCamParamCB(Perception::CamParamPacketType pack, void* userData) {
    DSTATUS("stereo cam parameters : timestamp(%d) dirNum(%d)", pack.timeStamp, pack.directionNum);
    if ((pack.directionNum > 0) && (pack.directionNum <= IMAGE_MAX_DIRECTION_NUM)) {
        for (uint32_t i = 0; i < pack.directionNum; i++) {
            DSTATUS("dir[%d] parameters :", pack.cameraParam[i].direction);
            DSTATUS("\tleftIntrinsics \t= {%f, %f, %f, %f, %f, %f, %f, %f, %f }", pack.cameraParam[i].leftIntrinsics[0],
                    pack.cameraParam[i].leftIntrinsics[1], pack.cameraParam[i].leftIntrinsics[2],
                    pack.cameraParam[i].leftIntrinsics[3], pack.cameraParam[i].leftIntrinsics[4],
                    pack.cameraParam[i].leftIntrinsics[5], pack.cameraParam[i].leftIntrinsics[6],
                    pack.cameraParam[i].leftIntrinsics[7], pack.cameraParam[i].leftIntrinsics[8]);
            DSTATUS("\trightIntrinsics \t= {%f, %f, %f, %f, %f, %f, %f, %f, %f }",
                    pack.cameraParam[i].rightIntrinsics[0], pack.cameraParam[i].rightIntrinsics[1],
                    pack.cameraParam[i].rightIntrinsics[2], pack.cameraParam[i].rightIntrinsics[3],
                    pack.cameraParam[i].rightIntrinsics[4], pack.cameraParam[i].rightIntrinsics[5],
                    pack.cameraParam[i].rightIntrinsics[6], pack.cameraParam[i].rightIntrinsics[7],
                    pack.cameraParam[i].rightIntrinsics[8]);
            DSTATUS("\trotaionLeftInRight \t= {%f, %f, %f, %f, %f, %f, %f, %f, %f }",
                    pack.cameraParam[i].rotaionLeftInRight[0], pack.cameraParam[i].rotaionLeftInRight[1],
                    pack.cameraParam[i].rotaionLeftInRight[2], pack.cameraParam[i].rotaionLeftInRight[3],
                    pack.cameraParam[i].rotaionLeftInRight[4], pack.cameraParam[i].rotaionLeftInRight[5],
                    pack.cameraParam[i].rotaionLeftInRight[6], pack.cameraParam[i].rotaionLeftInRight[7],
                    pack.cameraParam[i].rotaionLeftInRight[8]);
            DSTATUS("\ttranslationLeftInRight \t= {%f, %f, %f }", pack.cameraParam[i].translationLeftInRight[0],
                    pack.cameraParam[i].translationLeftInRight[1], pack.cameraParam[i].translationLeftInRight[2]);
        }
    }
}

}  // namespace

ImagesCollector::ImagesCollector(const Constants& consts, DJI::OSDK::Vehicle* vehicle, const std::string& save_path)
        : p_vehicle(vehicle)
        , m_sgm(consts.dispSize(), consts.p1(), consts.p2(), consts.uniqueness(), consts.subpixel(), consts.pathType(),
                consts.minDisp(), consts.lrMaxDiff(), consts.censusType())
        , m_savePath(save_path) {
    if (consts.subscribeToFpv()) {
        fs::create_directories(m_savePath + "/" + m_folderNameFpv);
    }
    if (consts.subscribeToStereo()) {
        fs::create_directories(m_savePath + "/" + g_stereoFolderNames.at(m_currentStereoDir).data());
    }
    m_stereoImageThreads = {{CameraDirection::FRONT, {}}, {CameraDirection::REAR, {}}, {CameraDirection::RIGHT, {}},
                            {CameraDirection::LEFT, {}},  {CameraDirection::UP, {}},   {CameraDirection::DOWN, {}}};

    if (nullptr != p_vehicle) {
        subscribe(consts);
    } else {
        throw std::runtime_error("ImagesCollector: p_vehicle is nullptr, to continue change dryRun to false");
    }
}

ImagesCollector::~ImagesCollector() {
    unsubscribe();
}

void ImagesCollector::unsubscribe() {
    if (m_subscribedToFpv) {
        p_vehicle->advancedSensing->stopFPVCameraStream();
    }
    if (m_subscribedToStereo) {
        m_terminateSgm = true;
        if (unsubscribeStereo()) {
            throw std::runtime_error(
                fmt::format("ImagesCollector failed to unsubscribe {}", g_stereoFolderNames.at(m_currentStereoDir)));
        }
        if (s_consts.computeDepth()) {
            m_sgmThread.join();
        }
    }
}

void ImagesCollector::subscribe(const Constants& consts) {
    if (consts.subscribeToStereoParams()) {
        unsubscribe();
        if (trySubscribeStereoCamParams()) {
            unsubscribe();
            throw std::runtime_error("ImagesCollector could have not subscribe to stereo camera params.");
        }
        printf("Subscribed to stereo params. FPV and stereo images subscription will not run.\n");
        return;
    } else {
        printf("subscribeToStereoParams is false. Starting ImagesCollector in image collecting mode.\n");
    }
    if (consts.subscribeToFpv()) {
        if (trySubscribeFpv()) {
            unsubscribe();
            throw std::runtime_error("ImagesCollector could have not subscribe to FPV.");
        }
        m_subscribedToFpv = true;
    } else {
        printf("subscribeToFpv is false, Fpv images will not be saved.\n");
    }
    if (consts.subscribeToStereo()) {
        if (trySubscribeStereo(m_currentStereoDir)) {
            unsubscribe();
            throw std::runtime_error("ImagesCollector could have not subscribe to stereo images.");
        }
        if (consts.computeDepth()) {
            m_sgmThread = std::thread(&ImagesCollector::sgmWork, this);
        }
        m_subscribedToStereo = true;
    } else {
        printf("subscribeToStereo is false, stereo images will not be saved.\n");
    }
}

bool ImagesCollector::unsubscribeStereo() {
    Perception::PerceptionErrCode perceptionStat =
        p_vehicle->advancedSensing->unsubscribePerceptionImage(g_stereoDirToImgType.at(m_currentStereoDir));

    if (perceptionStat != Perception::OSDK_PERCEPTION_PASS) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

void ImagesCollector::changeSubscription(CameraDirection dir) {
    if (unsubscribeStereo()) {
        throw std::runtime_error("ImagesCollector could have not subscribe to stereo images.");
    }
    if (trySubscribeStereo(dir)) {
        throw std::runtime_error("ImagesCollector could have not subscribe to stereo images.");
    }
}

std::optional<uint32_t> ImagesCollector::getStereoImgIdxIfAreTheSame() {
    Perception::CamPositionType camPosLeft = g_stereoDirToCamPosPair.at(m_currentStereoDir)[0];
    if (not m_camPosToLastImgIdx.count(camPosLeft)) {
        printf("m_camPosToLastImgIdx.count(camPosLeft)=%zu.\n", m_camPosToLastImgIdx.count(camPosLeft));
        return std::nullopt;
    }
    Perception::CamPositionType camPosRight = g_stereoDirToCamPosPair.at(m_currentStereoDir)[1];
    if (not m_camPosToLastImgIdx.count(camPosRight)) {
        printf("m_camPosToLastImgIdx.count(camPosRight)=%zu.\n", m_camPosToLastImgIdx.count(camPosRight));
        return std::nullopt;
    }
    uint32_t idLeft = m_camPosToLastImgIdx.at(camPosLeft);
    uint32_t idRight = m_camPosToLastImgIdx.at(camPosRight);
    if (idLeft == idRight) {
        return {idLeft};
    }
    printf("idLeft=%d, idRight=%d.\n", idLeft, idRight);
    return std::nullopt;
}

void ImagesCollector::maybeNotifySgm() {
    std::unique_lock<std::mutex> lock{m_sgmMut};
    auto idx = getStereoImgIdxIfAreTheSame();
    if (not idx.has_value()) {
        printf("do not notify.\n");
        return;
    }
    m_currentImgIdx = idx.value();
    if (idx.value() != m_lastProcessedImgIdx) {
        m_sgmCondVar.notify_one();
        printf("do notify.\n");
    }
}

void ImagesCollector::sgmWork(ImagesCollector* self) {
    while (not self->terminateSgm()) {
        std::unique_lock<std::mutex> lock(self->sgmMut());
        self->sgmCondVar().wait(lock, [self] {
            auto idx = self->getStereoImgIdxIfAreTheSame();
            if (not idx.has_value()) {
                printf("do not work 1 ~sgm.\n");
                return false;
            }
            if (self->currentImgIdx() == self->lastProcessedImgIdx()) {
                printf("do not work 2 ~sgm.\n");
                return false;
            }
            printf("do work ~sgm.\n");
            return true;
        });
        Perception::CamPositionType camPosLeft = g_stereoDirToCamPosPair.at(self->currentStereoDir())[0];
        Perception::CamPositionType camPosRight = g_stereoDirToCamPosPair.at(self->currentStereoDir())[1];

        cv::Mat imgLeft{1, 1, CV_16U}, imgRight{1, 1, CV_16U}, disparity{1, 1, CV_16U};
        self->stereoCvImages(camPosLeft).convertTo(imgLeft, CV_16U);
        self->stereoCvImages(camPosRight).convertTo(imgRight, CV_16U);

        // clang-format off
        std::string imgPath = self->savePath() + "/" + g_stereoFolderNames.at(self->currentStereoDir()).data() + "/"
            + g_stereoFolderNames.at(self->currentStereoDir()).data()
            + "-idx_" + std::to_string(self->stereoImages(self->currentStereoDir()).info.rawInfo.index)
            + "-dir_" + g_stereoFolderNames.at(g_imgTypeToStereoDir.at(self->stereoImages(self->currentStereoDir()).info.rawInfo.direction)).data()
            + "-dt_" + g_camPosToPosName.at(self->stereoImages(self->currentStereoDir()).info.dataType).data()
            // + "-id_" + std::to_string(self->stereoImages(self->currentStereoDir()).info.dataId)
            // + "-seq_" + std::to_string(self->stereoImages(self->currentStereoDir()).info.sequence)
            // + "-ts_" + std::to_string(self->stereoImages(self->currentStereoDir()).info.timeStamp)
            // + "-t_" + utils::time_in_YYYY_MM_DD_HH_MM_SS_MMM(3)
            + "-disparity.png";
        // clang-format on

        try {
            cv::cuda::GpuMat d_imgLeft(imgLeft), d_imgRight(imgRight), d_disparity;
            self->sgm().execute(d_imgLeft, d_imgRight, d_disparity);
            d_disparity.download(disparity);
        } catch (const cv::Exception& e) {
            if (e.code == cv::Error::GpuNotSupported) {
                throw std::runtime_error("ImagesCollector, libSGM: GpuNotSupported");
            } else {
                throw std::runtime_error(e.what());
            }
        }

        cv::imwrite(imgPath, disparity);

        self->lastProcessedImgIdx() = self->currentImgIdx();
    }
}

bool ImagesCollector::trySubscribeStereoCamParams() {
    DSTATUS("Do stereo camera parameters subscription");
    p_vehicle->advancedSensing->setStereoCamParamsObserver(perceptionCamParamCB, nullptr);
    Perception::PerceptionErrCode perceptionStat = p_vehicle->advancedSensing->triggerStereoCamParamsPushing();

    if (perceptionStat != Perception::OSDK_PERCEPTION_PASS) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

bool ImagesCollector::trySubscribeFpv() {
    bool camResult = p_vehicle->advancedSensing->startFPVCameraStream(&ImagesCollector::saveFpvTask, this);

    if (not camResult) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

static inline void createOsdkMutex(std::map<CameraDirection, ImagesCollector::StereoImagePacket>& stereoImages,
                                   CameraDirection dir) {
    stereoImages[dir] = {.info = {{0}}, .imageRawBuffer = nullptr, .mutex = nullptr, .gotData = false};
    OsdkOsal_MutexCreate(&stereoImages[dir].mutex);
}

template <CameraDirection DIR>
static inline bool createOsdkTask(std::map<CameraDirection, T_OsdkTaskHandle>& stereoImageThreads,
                                  DJI::OSDK::Vehicle* vehicle, ImagesCollector* self) {
    E_OsdkStat osdkStat = E_OsdkStat::OSDK_STAT_ERR;

    osdkStat = OsdkOsal_TaskCreate(&stereoImageThreads[DIR], ImagesCollector::stereoImagesDisplayTask<DIR>,
                                   OSDK_TASK_STACK_SIZE_DEFAULT, self);
    if (osdkStat != OSDK_STAT_OK) {
        return EXIT_FAILURE;
    }

    Perception::PerceptionErrCode perceptionStat = vehicle->advancedSensing->subscribePerceptionImage(
        g_stereoDirToImgType.at(DIR), perceptionImageCallback<DIR>, self);

    if (perceptionStat != Perception::OSDK_PERCEPTION_PASS) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

bool ImagesCollector::trySubscribeStereo(CameraDirection dir) {
    DSTATUS("Do stereo camera imagines subscription");
    // p_vehicle->advancedSensing->unsubscribePerceptionImage(m_stereoDirToImgType[CameraDirection::FRONT]);
    bool isFailure = false;
    switch (dir) {
        case CameraDirection::FRONT: {
            createOsdkMutex(m_stereoImages, CameraDirection::FRONT);
            isFailure = createOsdkTask<CameraDirection::FRONT>(m_stereoImageThreads, p_vehicle, this);
            break;
        }
        case CameraDirection::REAR: {
            createOsdkMutex(m_stereoImages, CameraDirection::REAR);
            isFailure = createOsdkTask<CameraDirection::REAR>(m_stereoImageThreads, p_vehicle, this);
            break;
        }
        case CameraDirection::RIGHT: {
            createOsdkMutex(m_stereoImages, CameraDirection::RIGHT);
            isFailure = createOsdkTask<CameraDirection::RIGHT>(m_stereoImageThreads, p_vehicle, this);
            break;
        }
        case CameraDirection::LEFT: {
            createOsdkMutex(m_stereoImages, CameraDirection::LEFT);
            isFailure = createOsdkTask<CameraDirection::LEFT>(m_stereoImageThreads, p_vehicle, this);
            break;
        }
        case CameraDirection::UP: {
            createOsdkMutex(m_stereoImages, CameraDirection::UP);
            isFailure = createOsdkTask<CameraDirection::UP>(m_stereoImageThreads, p_vehicle, this);
            break;
        }
        case CameraDirection::DOWN: {
            createOsdkMutex(m_stereoImages, CameraDirection::DOWN);
            isFailure = createOsdkTask<CameraDirection::DOWN>(m_stereoImageThreads, p_vehicle, this);
            break;
        }
    }
    return isFailure;
}

void ImagesCollector::saveFpvTask(CameraRGBImage img, void* p) {
    ImagesCollector* self = (ImagesCollector*)p;
#ifdef OPEN_CV_INSTALLED
    auto img_path = self->savePath() + "/" + self->folderNameFpv() + "/" + utils::time_in_YYYYMMDDHHMMSSMMM(6) +
                    "_" + self->folderNameFpv() + ".png";
    cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width * 3);
    cvtColor(mat, mat, cv::COLOR_RGB2BGR);
    cv::imwrite(img_path, mat);
    // std::cout << "#### Got image from:" << self->folderNameFpv() << ", size=(" << img.height << "," << img.width <<
    // "), path=" << img_path << "\n";
#endif
}

// template <CameraDirection DIR>
// void ImagesCollector::saveStereo(CameraRGBImage img, void* p) {
//     ImagesCollector* self = (ImagesCollector*)p;
// #ifdef OPEN_CV_INSTALLED
//     cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width * 3);
//     cvtColor(mat, mat, cv::COLOR_RGB2BGR);
//     auto img_path = self->savePath() + "/" + g_stereoFolderNames.at(DIR).data() + "/" +
//                     g_stereoFolderNames.at(DIR).data() + "_" + utils::time_in_YYYY_MM_DD_HH_MM_SS_MMM(3) + ".png";
//     cv::imwrite(img_path, mat);
//     // std::cout << "#### Got image from:" << self->stereoFolderNames(DIR) << ", size=(" << img.height << "," <<
//     // img.width << "), path=" << img_path << "\n";
// #endif
// }

template <CameraDirection DIR>
void* ImagesCollector::stereoImagesDisplayTask(void* arg) {
    while (true) {
        ImagesCollector* self = (ImagesCollector*)arg;
        OsdkOsal_TaskSleepMs(1);
        if (not arg) {
            continue;
        }

#ifdef OPEN_CV_INSTALLED
        /*! Get data here */
        // std::unique_lock<std::mutex> lock(self->sgmMut());
        OsdkOsal_MutexLock(self->stereoImages(DIR).mutex);
        if (!self->stereoImages(DIR).gotData) {
            OsdkOsal_MutexUnlock(self->stereoImages(DIR).mutex);
            // lock.unlock();
            // std::cout << "continue" << std::endl;
            continue;
        }
        // fmt::print("stereoImagesDisplayTask: {}", self->stereoFolderNames(DIR));
        // cv::Mat cvImgStereo = cv::Mat(
        //     self->stereoImages(DIR).info.rawInfo.height,
        //     self->stereoImages(DIR).info.rawInfo.width,
        //     CV_8U);
        Perception::CamPositionType cameraId =
            static_cast<Perception::CamPositionType>(self->stereoImages(DIR).info.dataType);
        if (not self->stereoCvImages().count(cameraId)) {
            printf("Adding new image to map: %s.\n", g_camPosToPosName.at(cameraId).data());
            self->stereoCvImages().insert({cameraId, cv::Mat(self->stereoImages(DIR).info.rawInfo.height,
                                                             self->stereoImages(DIR).info.rawInfo.width, CV_8U,
                                                             self->stereoImages(DIR).info.rawInfo.width)});
        }

        int copySize = self->stereoImages(DIR).info.rawInfo.height * self->stereoImages(DIR).info.rawInfo.width;
        if (self->stereoImages(DIR).imageRawBuffer) {
            memcpy(self->stereoCvImages(cameraId).data, self->stereoImages(DIR).imageRawBuffer, copySize);
            self->camPosToLastImgIdx()[cameraId] = self->stereoImages(DIR).info.rawInfo.index;
            self->maybeNotifySgm();
            OsdkOsal_Free(self->stereoImages(DIR).imageRawBuffer);
            self->stereoImages(DIR).imageRawBuffer = nullptr;
        }
        // char name[20] = {0};
        // sprintf(name, "Image_dataType : %d", self->stereoImages(DIR).info.dataType);
        // fmt::print("{}\n", name);
        self->stereoImages(DIR).gotData = false;

        // clang-format off
        auto imgPath = self->savePath() + "/" + g_stereoFolderNames.at(DIR).data() + "/"
            + utils::time_in_YYYYMMDDHHMMSSMMM(6)
            + "-idx_" + std::to_string(self->stereoImages(DIR).info.rawInfo.index)
            + "-dir_" + g_stereoFolderNames.at(g_imgTypeToStereoDir.at(self->stereoImages(DIR).info.rawInfo.direction)).data()
            + "-dt_" + g_camPosToPosName.at(self->stereoImages(DIR).info.dataType).data()
            + "-id_" + std::to_string(self->stereoImages(DIR).info.dataId)
            + "-seq_" + std::to_string(self->stereoImages(DIR).info.sequence)
            + "-ts_" + std::to_string(self->stereoImages(DIR).info.timeStamp)
            + "_stereo.png";
        // clang-format on
        OsdkOsal_MutexUnlock(self->stereoImages(DIR).mutex);
        // lock.unlock();

        cv::imwrite(imgPath, self->stereoCvImages(cameraId));
        // fmt::print("#### Got image from: {}, size=({},{}), path={}\n",
        //     self->stereoFolderNames(DIR),
        //     self->stereoImages(DIR).info.rawInfo.height,
        //     self->stereoImages(DIR).info.rawInfo.width,
        //     img_path.c_str()
        // );
#else
        std::cout << "else" << std::endl;
        OsdkOsal_MutexLock(self->stereoImages(DIR).mutex);
        if (!self->stereoImages(DIR).gotData) {
            OsdkOsal_MutexUnlock(self->stereoImages(DIR).mutex);
            continue;
        }
        int copySize = self->stereoImages(DIR).info.rawInfo.height * self->stereoImages(DIR).info.rawInfo.width;
        uint8_t* imageRawBuffer = (uint8_t*)OsdkOsal_Malloc(copySize);
        memcpy(imageRawBuffer, self->stereoImages(DIR).imageRawBuffer, copySize);
        OsdkOsal_MutexUnlock(self->stereoImages(DIR).mutex);

        DSTATUS("Save images at local path.");
        writePictureData(imageRawBuffer, copySize);
        OsdkOsal_Free(imageRawBuffer);
#endif
    }
}
