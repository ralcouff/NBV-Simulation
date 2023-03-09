#ifndef NBV_SIMULATION_SHARE_DATA_H
#define NBV_SIMULATION_SHARE_DATA_H

#pragma once

#include <filesystem>

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>

#include "utils.cpp"
#include "View.h"

namespace fs = std::filesystem;

#define OursIG 0
#define OA 1
#define UV 2
#define RSE 3
#define APORA 4
#define Kr 5
#define NBVNET 6
#define PCNBV 7
#define NewOurs 8
#define Test_one 101

class Share_Data {
public:
    std::string yaml_config_file_path;
    std::string object_folder_path;
    std::string name_of_object;
    std::string object_file_path;
    std::string view_space_file_path;
    std::string quality_file_path;
    std::string save_path;

    // Point cloud data
    std::atomic<int> valid_clouds{};
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud; // The input cloud from the reconstruction algorithm
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr working_cloud; // The cloud used for computations

    // Octomap parameters
    double octomap_resolution{}; // The resolution to use for the octomap
    octomap::ColorOcTree *octo_model; // The Octomap used for computation
    double p_unknown_upper_bound{}; // Upper bound for voxels to still be considered uncertain. Default: 0.97.
    double p_unknown_lower_bound{}; // Lower bound for voxels to still be considered uncertain. Default: 0.12.
    int init_voxels; // Number of initial voxels in the octo_model

    bool move_wait{};
    std::atomic<bool> over{}; // Checks if the process is over
    std::atomic<bool> current_view_space_processed{}; // Is the View Space processed
    std::atomic<bool> current_views_information_processed{}; // Is the Views Information processed
    std::atomic<bool> move_on{}; // Did we continue

    // Position matrices
    Eigen::Matrix4d current_camera_pose_world; // The current camera pose considered as the NBV
    Eigen::Vector3d object_center_world; // The barycenter of the object, used as the center of the world
    double bbox_radius{}; // The estimated radius of the object
    double map_size{}; // The radius of the octomap
    double init_entropy; // Octomap entropy
    double voxels_in_BBX; // Number of voxels in the octomap

    // Pipeline parameters
    int method_of_IG; // Method of Information Gain computation
    int num_of_max_iteration; // Number of maximum iteration
    int num_of_views; // Number of views sampled around the object
    double cost_weight{}; // Gamma in Information Gain computation
    double skip_coefficient{}; // TODO: Jsp
    double sum_local_information{}; // The sum of all local information
    double sum_global_information{}; // The sum of all global information
    std::vector<View> best_views{}; // A list containing the successive best views

    /* Camera parameters */
    rs2_intrinsics color_intrinsics{}; // Intrinsic parameters for the camera

    /**
     * Constructor of the Share_Data object
     * It is initialized with a file
     * @param _config_file_path The initialization file
     */
    explicit Share_Data(std::string _config_file_path);

    /**
     * Destructor of the object Share_Data
     */
    ~Share_Data();

    /**
     * Check for the existence of folders in multi-level directories and create them if they do not exist
     * @param cd The folder path
     */
    static void access_directory(const std::string &cd);

};

#endif //NBV_SIMULATION_SHARE_DATA_H