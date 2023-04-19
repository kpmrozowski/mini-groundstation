#include "Constants.hpp"

#include <fstream>
#include <future>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <fmt/format.h>
#include <libsgm.h>
#include <spdlog/spdlog.h>
#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "CsvReader.hpp"
#include "DataAggregator.hpp"
#include "Settings.hpp"

static constexpr float g_invalid_depth = 10000.f;
static const cv::Size g_rgb_resol{1280, 960};
static const cv::Size g_stereo_resol{640, 480};
static constexpr bool g_fit_images_to_stereo = true;
static constexpr bool g_fit_left_stereo_to_rgb = true;

struct MonoCalibration {
    cv::Mat_<double> intrinsics;
    cv::Mat_<double> distortion;
    cv::Mat_<double> rotation;
    cv::Mat_<double> translation;
};

struct DJICalibration {
    MonoCalibration rgb;
    MonoCalibration stereoL_rect;
    MonoCalibration stereoR_rect;
    cv::Mat StereoL_stereoR_transform;
};

struct Rectification {
    cv::Mat rgb_map_1;
    cv::Mat rgb_map_2;
    cv::Mat gray_map_1;
    cv::Mat gray_map_2;
    cv::Mat new_intrinsics;
    cv::Rect rgb_valid_pix_roi;
    cv::Rect gray_valid_pix_roi;
};

static sgm::StereoSGM createStereoSGM(const Settings& s, const Constants& consts);

static cv::Mat computeDisparity(sgm::StereoSGM& ssgm, const cv::Mat& stereoL_raw, const cv::Mat& stereoR_raw,
                                const Settings& s);

static void saveImageAsDisparityPath(cv::InputArray image, const ImagePath_t& stereo_imag_path,
                                     const FolderName_t output_folder_path, std::string file_format, const Settings& s);

static void saveDepthAsDisparityPath(cv::InputArray pc_to_save, const ImagePath_t& stereo_imag_path,
                                     const FolderName_t output_folder_path, std::string file_format, const Settings& s);

static void savePointCloudAsDisparityPath(cv::InputArray pc_to_save, cv::InputArray image,
                                          const ImagePath_t& stereo_imag_path, const FolderName_t output_folder_path,
                                          std::string file_format, const Settings& s);

static void saveTrackedPoints(const std::vector<ORB_SLAM3::MapPoint*>& pc_to_save,
                              const FolderName_t output_folder_path, const Settings& s);

static Rectification performRectification(const DJICalibration& calib);

static DJICalibration loadDJICalibration(const std::string& path);

static bool shouldSave(size_t current_index) {
    return current_index % 1000 == 0;
}

int main() {
    spdlog::info("Hello from generate_depth!");

    // ========================================================
    // ==================== LOADING CONFIG ====================
    spdlog::info("LOADING CONFIG\n");
    Settings s{};

    spdlog::info("Loading Constants");
    Constants consts{s.configPath};
    try {
        spdlog::info("consts.dataPath={}\n", consts.dataPath());
    } catch (const std::exception& e) {
        spdlog::error("not consts.dataPath: {}\n", e.what());
    }

    std::unordered_map<FolderName_t, std::vector<ImagePath_t>> flights_map{
        {"_Radziwie7/1_flight_20230210-120154", {}},
        // {"_Targowa26/1_flight_20230125-104622", {}},
        // {"_Targowa26/2_flight_20230125-122203", {}},
    };

    const std::map<int, CollectedData> telemetry =
        CsvReader::readCsvRaw(consts.dataPath() + "/_Radziwie7/1_flight_20230210-120154/telemetry_fixed.csv");
    if (telemetry.empty()) {
        fmt::print("telemetry is empty\n");
    }

    const auto fpv_paths_map = DataAggregator::findFpvPaths(flights_map, consts);
    auto stereo_paths_map = utils::findStereoPairsByTimestamp(flights_map, s, consts);
    stereo_paths_map = DataAggregator::updateStereoTimestamps(stereo_paths_map);

    std::map<FolderName_t, std::map<Timestamp_t, FrameRecord>> fpv_stereo_paths_map;
    if (consts.computeColor()) {
        fpv_stereo_paths_map = DataAggregator::mergeFpvWithStereoPaths(fpv_paths_map, stereo_paths_map);
    } else {
        fpv_stereo_paths_map = DataAggregator::mergeFpvWithStereoPaths(stereo_paths_map);
        // fpv_stereo_paths_map = DataAggregator::mergeTelemetryWithStereoPaths(stereo_paths_map, telemetry);
    }

    spdlog::info("Creating StereoSGM");
    sgm::StereoSGM ssgm = createStereoSGM(s, consts);

    [[maybe_unused]] bool init_slam_status = init_slam_future.get();

    for (const auto& [folder_name, timestamp_imgs_pair_map] : fpv_stereo_paths_map) {
        point_cloud_collector.slam().Reset();

        const std::string dataset_path = consts.dataPath() + "/" + folder_name;
        fmt::print("\nComputing depth for '{}'\n", dataset_path);
        const std::string output_path = dataset_path + "/" + consts.outputFolderName();

        const FolderName_t output_disparity_path =  output_path + "/" + s.output_disparity_folder_name;
        const FolderName_t output_depth_path = output_path + "/" + s.output_depth_folder_name;
        const FolderName_t output_color_path = output_path + "/" + s.output_color_folder_name;
        const FolderName_t output_pc_path = output_path + "/" + s.output_pc_folder_name;

        const FolderName_t output_fpv_undis_path = output_path + "/o-undisFpv";
        const FolderName_t output_stereoL_undis_path = output_path + "/o-undisL";
        const FolderName_t output_stereoR_undis_path = output_path + "/o-undisR";
        const FolderName_t output_fpv_rect_path = output_path + "/o-rectFpv";
        const FolderName_t output_stereoL_rect_path = output_path + "/o-rectL";
        const FolderName_t output_stereoR_rect_path = output_path + "/o-rectR";

        auto save_orb_slam3_lambda = [&s, &point_cloud_collector, &output_path](){
            saveTrackedPoints(point_cloud_collector.slam().GetTrackedMapPoints(), output_path, s);
            saveMapPointsAndTrajectories(point_cloud_collector.slam(), output_path, s);
            point_cloud_collector.slam().SaveKeyFrameTrajectoryEuRoC(output_path + "/o-KeyFrameTrajectoryEuRoC.csv");
            point_cloud_collector.slam().SaveKeyFrameTrajectoryTUM(output_path + "/o-KeyFrameTrajectoryTUM.csv");
            point_cloud_collector.slam().SaveTrajectoryEuRoC(output_path + "/o-TrajectoryEuRoC.csv");
            // point_cloud_collector.slam().SaveAtlas(0);
            // point_cloud_collector.slam().SaveTrajectoryTUM(output_path + "/o-TrajectoryTUM.csv");
            // point_cloud_collector.slam().SaveTrajectoryKITTI(output_path + "/o-TrajectoryKITTI.csv");
        };

        utils::createDirectories({
            output_path,
        });
        if (consts.computeDisparity()) {
            utils::createDirectories({
                output_disparity_path,
            });
        }
        if (consts.computeDepth()) {
            utils::createDirectories({
                output_depth_path,
            });
        }
        if (consts.computeColor()) {
            utils::createDirectories({
                output_fpv_undis_path,
                output_fpv_rect_path,
            });
        }
        /// TODO: save disparity point cloud without color
        if (consts.computePointCloud() and consts.computeColor()) {
            utils::createDirectories({
                output_pc_path,
            });
        }
        utils::createDirectories({
            output_stereoL_undis_path,
            output_stereoR_undis_path,
            output_stereoL_rect_path,
            output_stereoR_rect_path,
        });

        spdlog::info("Loading DJICalibration");
        DJICalibration calib =
            loadDJICalibration("/workspace/src/dji_data/configs/camera_calibration/calibration_2.json");

        spdlog::info("Performing rectification");
        Rectification rectification = performRectification(calib);
        // double baseline = 79.401115; // baseline
        double Tx = calib.StereoL_stereoR_transform.at<double>(0, 3);
        // [1, 0, 0,     -cx1,
        //  0, 1, 0,     -cy,
        //  0, 0, 0,      f,
        //  0, 0, -1/Tx,  (cx1-cx2)/Tx ]
        cv::Mat mapping(4, 4, CV_64F);
        mapping.at<double>(0, 0) = 1;
        mapping.at<double>(1, 1) = 1;
        mapping.at<double>(3, 2) = -1 / Tx;
        mapping.at<double>(0, 3) = -calib.stereoL_rect.intrinsics.at<double>(0, 2);
        mapping.at<double>(1, 3) = -calib.stereoL_rect.intrinsics.at<double>(1, 2);
        mapping.at<double>(2, 3) = -calib.stereoL_rect.intrinsics.at<double>(0, 0);
        mapping.at<double>(3, 3) =
            (calib.stereoL_rect.intrinsics.at<double>(0, 2) - calib.rgb.intrinsics.at<double>(0, 2)) / Tx;
        std::cout << "my_mapping:\n" << mapping;

        size_t skipped_count = 0;
        size_t current_index = 0;
        for (const auto& [timestamp, fpv_stereoL_stereoR_paths] : timestamp_imgs_pair_map) {
            if (skipped_count < consts.framesToSkip()) {
                ++skipped_count;
                ++current_index;
                continue;
            } else {
                ++current_index;
            }
            // fmt::print("\n");
            spdlog::info("processing ts={}, {}/{}: {}%", timestamp, current_index, timestamp_imgs_pair_map.size(),
                         static_cast<float>(100 * current_index) / static_cast<float>(timestamp_imgs_pair_map.size()));

            cv::Mat fpv_img_raw, stereoL_raw, stereoR_raw, color_img, fpv_rect, stereoL_rect, stereoR_rect, fpv_undis,
                stereoL_undis, stereoR_undis;
            if (consts.computeColor()) {
                fpv_img_raw = cv::imread(fpv_stereoL_stereoR_paths.fpv, cv::IMREAD_UNCHANGED);
                cv::undistort(fpv_img_raw, fpv_undis, calib.rgb.intrinsics, calib.rgb.distortion);
                cv::remap(fpv_undis, fpv_rect, rectification.rgb_map_1, rectification.rgb_map_2, cv::INTER_LINEAR);
            }
            stereoL_raw = cv::imread(fpv_stereoL_stereoR_paths.stereo_l, cv::IMREAD_UNCHANGED);
            cv::undistort(stereoL_raw, stereoL_undis, calib.stereoL_rect.intrinsics, calib.stereoL_rect.distortion);
            cv::remap(stereoL_undis, stereoL_rect, rectification.gray_map_1, rectification.gray_map_2,
                      cv::INTER_LINEAR);

            stereoR_raw = cv::imread(fpv_stereoL_stereoR_paths.stereo_r, cv::IMREAD_UNCHANGED);
            cv::undistort(stereoR_raw, stereoR_undis, calib.stereoR_rect.intrinsics, calib.stereoR_rect.distortion);

            cv::remap(stereoR_undis, stereoR_rect, rectification.gray_map_1, rectification.gray_map_2,
                      cv::INTER_LINEAR);

            // compute disparity
            cv::Mat disparity;
            if (consts.computeDisparity() or consts.computeDepth() or consts.computePointCloud()) {
                disparity = computeDisparity(ssgm, stereoL_rect, stereoR_rect, s);
            }
            if (consts.computeDisparity()) {
                // save disparity
                std::thread saving_thread{saveImageAsDisparityPath, disparity, fpv_stereoL_stereoR_paths.stereo_l,
                                          output_disparity_path,    ".tiff",   s};
                saving_thread.detach();
                // saveImageAsDisparityPath(disparity, fpv_stereoL_stereoR_paths.stereo_l, output_disparity_path, ".tiff",
                //                          s);
            }

            // stereoL_rect.convertTo(stereoL_rect, fpv_rect.type());
            // cv::addWeighted(fpv_rect, 0.5, stereoL_rect, 0.5, 0, color_img);
            // fmt::print("c color_img.size=({}, {}),\n", color_img.size[0], color_img.size[1]);

            if (consts.computeColor()) {
                // std::thread saving_thread1{saveImageAsDisparityPath, fpv_undis, fpv_stereoL_stereoR_paths[1],
                //                            output_fpv_undis_path, ".png", s};
                saveImageAsDisparityPath(fpv_undis, fpv_stereoL_stereoR_paths.stereo_l, output_fpv_undis_path, ".png",
                                         s);
                saveImageAsDisparityPath(stereoL_undis, fpv_stereoL_stereoR_paths.stereo_l, output_stereoL_undis_path,
                                         ".png", s);
                saveImageAsDisparityPath(stereoR_undis, fpv_stereoL_stereoR_paths.stereo_r, output_stereoR_undis_path,
                                         ".png", s);

                saveImageAsDisparityPath(fpv_rect, fpv_stereoL_stereoR_paths.stereo_l, output_fpv_rect_path, ".png", s);
                saveImageAsDisparityPath(stereoL_rect, fpv_stereoL_stereoR_paths.stereo_l, output_stereoL_rect_path,
                                         ".png", s);
                saveImageAsDisparityPath(stereoR_rect, fpv_stereoL_stereoR_paths.stereo_r, output_stereoR_rect_path,
                                         ".png", s);
            }

            // compute depth
            cv::Mat img2d_to_coords3d_map{};
            bool handle_missing_values = true;
            if (consts.computeDepth() or consts.computePointCloud()) {
                disparity.assignTo(disparity, CV_32SC1);
                try {
                    cv::reprojectImageTo3D(disparity, img2d_to_coords3d_map, mapping, handle_missing_values);
                } catch (const cv::Exception& e) {
                    std::cout << "\nError in generate_depth.cpp in reprojectImageTo3D: " << e.what()
                              << "\ndisparity.type: " << cv::typeToString(disparity.type())
                              << "\nmapping.type: " << cv::typeToString(mapping.type())
                              << ", types: " << cv::typeToString(CV_8UC1) << ", " << cv::typeToString(CV_16SC1) << ", "
                              << cv::typeToString(CV_32SC1) << ", " << cv::typeToString(CV_32FC1) << "\n";
                }
                // std::cout << "img2d_to_coords3d_map.type: " << cv::typeToString(img2d_to_coords3d_map.type())
                //         << ", img2d_to_coords3d_map.dims: " << img2d_to_coords3d_map.dims
                //         << ", img2d_to_coords3d_map.size: (" << img2d_to_coords3d_map.size[0] << ", "
                //         << img2d_to_coords3d_map.size[1] << ")\n";
            }
            // if (consts.computeDepth()) {
            //     std::thread saving_thread{saveDepthAsDisparityPath,
            //                               img2d_to_coords3d_map,
            //                               fpv_stereoL_stereoR_paths.stereo_l,
            //                               output_depth_path,
            //                               ".tiff",
            //                               s};
            //     saving_thread.detach();
            // }
            if (consts.computeDepth()) {
                saveDepthAsDisparityPath(img2d_to_coords3d_map, fpv_stereoL_stereoR_paths.stereo_l, output_depth_path,
                                         ".tiff", s);
            }
            // if (consts.computePointCloud() and consts.computeColor()) {
            //     std::thread saving_thread{savePointCloudAsDisparityPath,
            //                               img2d_to_coords3d_map,
            //                               fpv_rect,
            //                               fpv_stereoL_stereoR_paths.stereo_l,
            //                               output_pc_path,
            //                               ".ply",
            //                               s};
            //     saving_thread.detach();
            // }
            if (consts.computePointCloud() and consts.computeColor()) {
                // save point_cloud
                savePointCloudAsDisparityPath(img2d_to_coords3d_map, fpv_rect, fpv_stereoL_stereoR_paths.stereo_l,
                                              output_pc_path, ".ply", s);
            }
            if (consts.printDebug()) {
                std::vector<ORB_SLAM3::Map*> maps_vec = point_cloud_collector.slam().GetAllMaps();
                for (ORB_SLAM3::Map* pMap : maps_vec) {
                    std::cout << "  Map " << std::to_string(pMap->GetId()) << " has "
                              << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs"
                              << "and is" << (pMap->IsBad() ? "Bad" : "Good") << std::endl;
                }
            }

            // compute SLAM
            const double timestamp_d = static_cast<double>(timestamp) * 1e-3;
            // fmt::print("timestamp_d={}, imu_ts1={}, imu_ts2={}, imu_meas.size: {}\n",
            //            timestamp_d, fpv_stereoL_stereoR_paths.imu_meas.front().timestamp, fpv_stereoL_stereoR_paths.imu_meas.back().timestamp, fpv_stereoL_stereoR_paths.imu_meas.size());
            if (consts.computeSLAM()) {
                // point_cloud_collector.slam().TrackStereo(stereoL_undis, stereoR_undis, timestamp_d);
                point_cloud_collector.slam().TrackStereo(stereoL_raw, stereoR_raw, timestamp_d,
                                                        fpv_stereoL_stereoR_paths.imu_meas);
            }

            if (shouldSave(current_index) and consts.computeSLAM() and current_index > 100 + consts.framesToSkip()) {
                spdlog::info("SAVING SLAM DATA");
                save_orb_slam3_lambda();
                // std::thread saving_thread{save_orb_slam3_lambda};
                // saving_thread.detach();
            }
        }
        save_orb_slam3_lambda();
        point_cloud_collector.slam().Shutdown();
    }
}

// =============================================================
// ========================= FUNCTIONS =========================

sgm::StereoSGM createStereoSGM(const Settings& s, const Constants& consts) {
    // clang-format off
    return {
        s.stereo_resolution[0],
        s.stereo_resolution[1],
        consts.dispSize(),
        s.stereo_bits,
        s.disparity_bits,
        sgm::EXECUTE_INOUT_HOST2HOST,
        {
            consts.p1(),
            consts.p2(),
            consts.uniqueness(),
            consts.subpixel(),
            consts.pathType(),
            consts.minDisp(),
            consts.lrMaxDiff(),
            consts.censusType()
        }
    };
    // clang-format on
}

static cv::Mat computeDisparity(sgm::StereoSGM& ssgm, const cv::Mat& stereoL_raw, const cv::Mat& stereoR_raw,
                                const Settings& s) {
    cv::Mat stereoL_rect{1, 1, CV_8U}, stereoR_rect{1, 1, CV_8U};
    stereoL_raw.convertTo(stereoL_rect, CV_8U);
    stereoR_raw.convertTo(stereoR_rect, CV_8U);
    cv::Mat disparity(stereoL_rect.size(), CV_16U);
    try {
        // cv::cuda::GpuMat d_stereoL(stereoL_rect), d_stereoR(stereoR_rect), d_disparity;
        // ssgm.execute(d_stereoL.data, d_stereoR.data, d_disparity);
        ssgm.execute(stereoL_rect.data, stereoR_rect.data, disparity.data);
        // d_disparity.download(disparity);
    } catch (const cv::Exception& e) {
        if (e.code == cv::Error::GpuNotSupported) {
            throw std::runtime_error("generate_depth, libSGM: GpuNotSupported");
        } else {
            throw std::runtime_error(e.what());
        }
    }
    // create mask for invalid disp
    const cv::Mat mask1 = disparity == ssgm.get_invalid_disparity();
    const cv::Mat mask2 = disparity == s.invalid_disparity_value;
    disparity.setTo(0, mask1);
    disparity.setTo(0, mask2);
    return disparity;
}

static std::string parentFolderName(std::string path) {
    const size_t last_pos = path.find_last_of("/");
    const std::string parent_path = path.substr(0, last_pos);
    const size_t prelast_pos = parent_path.find_last_of("/");
    const std::string parent_folder = parent_path.substr(prelast_pos + 1, last_pos - prelast_pos);

    return parent_folder;
}

static ImagePath_t prepareFilePath(const ImagePath_t& stereo_imag_path, const FolderName_t output_folder_path,
                                   std::string file_format, const Settings& s) {
    static constexpr size_t stereo_str_size = 7;
    size_t folder_path_len = stereo_imag_path.find_last_of("/");
    if (std::string::npos == folder_path_len) {
        throw std::runtime_error(fmt::format("no '/' in {}", stereo_imag_path));
    }
    ImagePath_t file_name =
        stereo_imag_path.substr(folder_path_len, stereo_imag_path.size() - folder_path_len -
                                                     s.stereo_file_format.size() - stereo_str_size) +
        "-" + parentFolderName(output_folder_path) + file_format;

    return output_folder_path + file_name;
}

static void saveImageAsDisparityPath(cv::InputArray image, const ImagePath_t& stereo_imag_path,
                                     const FolderName_t output_folder_path, std::string file_format,
                                     const Settings& s) {
    ImagePath_t file_path = prepareFilePath(stereo_imag_path, output_folder_path, file_format, s);
    cv::imwrite(file_path, image);
    // fmt::print("'{}' saved\n", file_path);
}

static void saveDepthAsDisparityPath(cv::InputArray pc_to_save, const ImagePath_t& stereo_imag_path,
                                     const FolderName_t output_folder_path, std::string file_format,
                                     const Settings& s) {
    cv::Mat pc_mat = pc_to_save.getMat();
    cv::Mat depth(pc_mat.rows, pc_mat.cols, CV_64F);
    std::vector<cv::Mat> channels_xyz(3);

    cv::split(pc_mat, channels_xyz);

    channels_xyz[2].setTo(0, channels_xyz[2] == g_invalid_depth);

    ImagePath_t file_path = prepareFilePath(stereo_imag_path, output_folder_path, file_format, s);
    cv::imwrite(file_path, channels_xyz[2]);
    // fmt::print("'{}' saved\n", file_path);
}

static void savePointCloudAsDisparityPath(cv::InputArray pc_to_save, cv::InputArray image,
                                          const ImagePath_t& stereo_imag_path, const FolderName_t output_folder_path,
                                          std::string file_format, const Settings& s) {
    cv::Mat pc_mat = pc_to_save.getMat();
    cv::Mat image_mat = image.getMat();

    size_t pc_size = 0;
    for (int row = 0; row < pc_mat.rows; ++row) {
        for (int col = 0; col < pc_mat.cols; ++col) {
            if (std::isinf(pc_mat.at<cv::Vec3f>(row, col)[0]) or std::isinf(pc_mat.at<cv::Vec3f>(row, col)[1]) or
                std::isinf(pc_mat.at<cv::Vec3f>(row, col)[2]) or pc_mat.at<cv::Vec3f>(row, col)[2] == g_invalid_depth) {
                continue;
            }
            ++pc_size;
        }
    }

    ImagePath_t file_path = prepareFilePath(stereo_imag_path, output_folder_path, file_format, s);
    std::ofstream outfile(file_path);
    outfile << "ply\n"
            << "format ascii 1.0\n"
            << "comment VTK generated PLY File\n";
    outfile << "element vertex " << pc_size << "\n";
    outfile << "property float x\n"
            << "property float y\n"
            << "property float z\n";
    outfile << "property uchar red\n"
            << "property uchar green\n"
            << "property uchar blue\n"
            << "property uchar alpha\n";
    outfile << "element face 0\n";
    outfile << "property list uchar int vertex_indices\n"
            << "end_header";

    for (int row = 0; row < pc_mat.rows; ++row) {
        for (int col = 0; col < pc_mat.cols; ++col) {
            if (std::isinf(pc_mat.at<cv::Vec3f>(row, col)[0]) or std::isinf(pc_mat.at<cv::Vec3f>(row, col)[1]) or
                std::isinf(pc_mat.at<cv::Vec3f>(row, col)[2]) or pc_mat.at<cv::Vec3f>(row, col)[2] == g_invalid_depth) {
                continue;
            }
            outfile << "\n";
            outfile << -pc_mat.at<cv::Vec3f>(row, col)[0] << " ";
            outfile << -pc_mat.at<cv::Vec3f>(row, col)[1] << " ";
            outfile << -pc_mat.at<cv::Vec3f>(row, col)[2] << " ";
            outfile << static_cast<int>(image_mat.at<cv::Vec3b>(row, col)[0]) << " ";
            outfile << static_cast<int>(image_mat.at<cv::Vec3b>(row, col)[1]) << " ";
            outfile << static_cast<int>(image_mat.at<cv::Vec3b>(row, col)[2]) << " 255";
        }
    }

    outfile.close();
    // fmt::print("'{}' saved\n", file_path);
}

static void saveTrackedPoints(const std::vector<ORB_SLAM3::MapPoint*>& pc_to_save,
                              const FolderName_t output_folder_path, const Settings& s) {
    std::map<unsigned long, std::vector<Eigen::Vector3f>> maps;

    for (ORB_SLAM3::MapPoint* point : pc_to_save) {
        if (point == nullptr) {
            continue;
        }
        if (point->isBad()) {
            continue;
        }
        if (point->GetMap()->IsBad()) {
            continue;
        }
        maps[point->GetMap()->GetId()].push_back(point->GetWorldPos());
    }

    for (const auto& [mapId, map_points] : maps) {
        ImagePath_t file_path = output_folder_path + "/" + s.output_tracked_points_file_name + "-mapId_" +
                                std::to_string(mapId) + s.pc_file_format;
        std::ofstream outfile(file_path);
        outfile << "ply\n"
                << "format ascii 1.0\n"
                << "comment VTK generated PLY File\n";
        outfile << "element vertex " << map_points.size() << "\n";
        outfile << "property float x\n"
                << "property float y\n"
                << "property float z\n";
        outfile << "element face 0\n";
        outfile << "property list uchar int vertex_indices\n"
                << "end_header";
        for (const Eigen::Vector3f& point : map_points) {
            outfile << "\n";
            outfile << point(0) << " ";
            outfile << point(1) << " ";
            outfile << point(2);
        }
        fmt::print("'{}' saved\n", file_path);
    }
}

static Rectification performRectification(const DJICalibration& calib) {
    std::cout << "Calibration rgb:\n"
              << calib.rgb.intrinsics << "\n"
              << calib.rgb.distortion << "\n"
              << calib.rgb.rotation << "\n"
              << calib.rgb.translation << "\n"
              << "Calibration stereoL_rect:\n"
              << calib.stereoL_rect.intrinsics << "\n"
              << calib.stereoL_rect.distortion << "\n"
              << calib.stereoL_rect.rotation << "\n"
              << calib.stereoL_rect.translation << "\n"
              << "Calibration stereoR_rect:\n"
              << calib.stereoR_rect.intrinsics << "\n"
              << calib.stereoR_rect.distortion << "\n"
              << calib.stereoR_rect.rotation << "\n"
              << calib.stereoR_rect.translation << "\n";

    // =========================================================
    // ==================== COMPUTING DEPTH ====================

    MonoCalibration gray_calib{};
    if constexpr (g_fit_left_stereo_to_rgb) {
        gray_calib.intrinsics = calib.stereoL_rect.intrinsics;
        gray_calib.distortion = calib.stereoL_rect.distortion;
        gray_calib.rotation = calib.stereoL_rect.rotation;
        gray_calib.translation = calib.stereoL_rect.translation;
        // gray_calib.translation = cv::Mat_<double>{7.5456632900812878e-02, 0, 0};
    } else {
        gray_calib.intrinsics = calib.stereoR_rect.intrinsics;
        gray_calib.distortion = calib.stereoR_rect.distortion;
        gray_calib.rotation = calib.stereoR_rect.rotation;
        gray_calib.translation = calib.stereoR_rect.translation;
        // gray_calib.translation = cv::Mat_<double>{-2.4666640199322742e-02, 0, 0};
    }
    cv::invert(gray_calib.rotation, gray_calib.rotation);
    gray_calib.translation = -gray_calib.translation;
    gray_calib.translation(1) = 0;
    gray_calib.translation(2) = 0;
    cv::Size image_size;
    if constexpr (g_fit_images_to_stereo) {
        image_size = g_stereo_resol;
    } else {
        image_size = g_rgb_resol;
    }

    cv::Mat rgb_rectification(3, 3, CV_64F);
    cv::Mat gray_rectification(3, 3, CV_64F);
    cv::Mat rgb_projectionMat(3, 4, CV_64F);
    cv::Mat gray_projectionMat(3, 4, CV_64F);
    cv::Mat mapping(4, 4, CV_64F);
    cv::Rect rgb_valid_pix_roi_1;
    cv::Rect gray_valid_pix_roi_1;

    spdlog::info("stereoRectify");
    std::cout << "calib.rgb.intrinsics:\n"
              << calib.rgb.intrinsics << "\n"
              << "calib.rgb.distortion:\n"
              << calib.rgb.distortion << "\n"
              << "gray_calib.intrinsics:\n"
              << gray_calib.intrinsics << "\n"
              << "gray_calib.distortion:\n"
              << gray_calib.distortion << "\n"
              << "image_size:\n"
              << image_size << "\n"
              << "gray_calib.rotation:\n"
              << gray_calib.rotation << "\n"
              << "gray_calib.translation:\n"
              << gray_calib.translation << "\n";
    spdlog::info("stereoRectify start");
    cv::Mat_<double> zero_distorions{0, 0, 0, 0, 0};
    cv::stereoRectify(gray_calib.intrinsics.clone(),   // Input
                      zero_distorions,                 // gray_calib.distortion.clone(),   // Input
                      calib.rgb.intrinsics.clone(),    // Input
                      zero_distorions,                 // calib.rgb.distortion.clone(),    // Input
                      image_size,                      // Input
                      gray_calib.rotation.clone(),     // Input
                      gray_calib.translation.clone(),  // Input
                      gray_rectification,              // Output
                      rgb_rectification,               // Output
                      gray_projectionMat,              // Output
                      rgb_projectionMat,               // Output
                      mapping,                         // Output
                      cv::CALIB_ZERO_DISPARITY,        // Input
                      1.0,                             // Input
                      g_stereo_resol,                  // Input
                      &gray_valid_pix_roi_1,           // Output
                      &rgb_valid_pix_roi_1             // Output
    );
    spdlog::info("getOptimalNewCameraMatrix gray\n");
    cv::Rect gray_valid_pix_roi_2;
    cv::Mat new_gray_intrinsics =
        cv::getOptimalNewCameraMatrix(gray_calib.intrinsics, zero_distorions,  // gray_calib.distortion,
                                      g_stereo_resol, 1.0, image_size, &gray_valid_pix_roi_2);
    spdlog::info("getOptimalNewCameraMatrix rgb\n");
    cv::Rect rgb_valid_pix_roi_2;
    cv::Mat new_rgb_intrinsics =
        cv::getOptimalNewCameraMatrix(gray_calib.intrinsics, zero_distorions,  // gray_calib.distortion,
                                      g_stereo_resol, 1.0, image_size, &rgb_valid_pix_roi_2);

    cv::Mat new_intrinsics;
    if constexpr (g_fit_images_to_stereo) {
        new_intrinsics = new_gray_intrinsics;
        // new_intrinsics = gray_projectionMat;
    } else {
        new_intrinsics = new_rgb_intrinsics;
        // new_intrinsics = rgb_projectionMat;
    }

    spdlog::info("initUndistortRectifyMap rgb\n");
    cv::Mat rgb_map_1, rgb_map_2;
    cv::initUndistortRectifyMap(calib.rgb.intrinsics, zero_distorions, /*calib.rgb.distortion*/ rgb_rectification,
                                new_intrinsics, image_size, CV_32FC2, rgb_map_1, rgb_map_2);

    spdlog::info("initUndistortRectifyMap gray\n");
    cv::Mat gray_map_1, gray_map_2;
    cv::initUndistortRectifyMap(gray_calib.intrinsics, zero_distorions, /*gray_calib.distortion*/ gray_rectification,
                                new_intrinsics, image_size, CV_32FC2, gray_map_1, gray_map_2);

    mapping.at<double>(4, 3) *= -1;
    mapping.at<double>(4, 4) *= -1;
    mapping.at<double>(3, 4) *= -1;
    std::cout << "new_intrinsics:\n"
              << new_intrinsics << "\n"
              << "new_gray_calib.intrinsics:\n"
              << new_gray_intrinsics << "\n\n"
              << "rgb_rectification:\n"
              << rgb_rectification << "\n"
              << "gray_rectification:\n"
              << gray_rectification << "\n"
              << "rgb_projectionMat:\n"
              << rgb_projectionMat << "\n"
              << "gray_projectionMat:\n"
              << gray_projectionMat << "\n"
              << "mapping:\n"
              << mapping << "\n\n"
              << "rgb_valid_pix_roi_2:\n"
              << rgb_valid_pix_roi_2 << "\n"
              << "gray_valid_pix_roi_2:\n"
              << gray_valid_pix_roi_2 << "\n"
              << "\n"
              << "rgb_map_1(" << rgb_map_1.size.dims() << ").size=(" << rgb_map_1.size[0] << ", " << rgb_map_1.size[1]
              << ")\n"
              << "rgb_map_2(" << rgb_map_2.size.dims() << ").size=(" << rgb_map_2.size[0] << ", " << rgb_map_2.size[1]
              << ")\n"
              << "gray_map_1(" << gray_map_1.size.dims() << ").size=(" << gray_map_1.size[0] << ", "
              << gray_map_1.size[1] << ")\n"
              << "gray_map_2(" << gray_map_2.size.dims() << ").size=(" << gray_map_2.size[0] << ", "
              << gray_map_2.size[1] << ")\n";

    Rectification r;
    r.rgb_map_1 = rgb_map_1;
    r.rgb_map_2 = rgb_map_2;
    r.gray_map_1 = gray_map_1;
    r.gray_map_2 = gray_map_2;
    r.new_intrinsics = new_intrinsics;
    r.rgb_valid_pix_roi = rgb_valid_pix_roi_2;
    r.gray_valid_pix_roi = gray_valid_pix_roi_2;
    return r;
}

static DJICalibration loadDJICalibration(const std::string& path) {
    DJICalibration calib{};
    nlohmann::json data = utils::readJson(path);
    calib.rgb.intrinsics = cv::Mat(data["intrinsics_calib_rgb"]["data"].get<std::vector<double>>(), true).reshape(0, 3);
    calib.rgb.distortion = cv::Mat(data["distortion_calib_rgb"]["data"].get<std::vector<double>>(), true);
    calib.rgb.rotation = cv::Mat::eye(3, 3, CV_64F);
    calib.rgb.translation = cv::Mat::zeros(3, 1, CV_64F);

    calib.stereoL_rect.intrinsics =
        cv::Mat(data["intrinsics_calib_ir1"]["data"].get<std::vector<double>>(), true).reshape(0, 3);
    calib.stereoL_rect.distortion = cv::Mat(data["distortion_calib_ir1"]["data"].get<std::vector<double>>(), true);
    calib.stereoL_rect.rotation =
        cv::Mat(data["rotation_calib_rgb_calib_ir1"]["data"].get<std::vector<double>>(), true).reshape(0, 3);
    calib.stereoL_rect.translation =
        cv::Mat(data["translation_calib_rgb_calib_ir1"]["data"].get<std::vector<double>>(), true);

    calib.stereoR_rect.intrinsics =
        cv::Mat(data["intrinsics_calib_ir2"]["data"].get<std::vector<double>>(), true).reshape(0, 3);
    calib.stereoR_rect.distortion = cv::Mat(data["distortion_calib_ir2"]["data"].get<std::vector<double>>(), true);
    calib.stereoR_rect.rotation =
        cv::Mat(data["rotation_calib_rgb_calib_ir2"]["data"].get<std::vector<double>>(), true).reshape(0, 3);
    calib.stereoR_rect.translation =
        cv::Mat(data["translation_calib_rgb_calib_ir2"]["data"].get<std::vector<double>>(), true);

    calib.StereoL_stereoR_transform =
        cv::Mat(data["transformation_calib_ir1_calib_ir2"]["data"].get<std::vector<double>>(), true);

    return calib;
}
