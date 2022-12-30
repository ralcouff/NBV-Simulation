#ifndef NBV_SIMULATION_SHARE_DATA_H
#define NBV_SIMULATION_SHARE_DATA_H

#pragma once

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <filesystem>

#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <utility>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmData/View.hpp>
#include <aliceVision/sfmDataIO/jsonIO.hpp>

#include "utils.cpp"
#include "View.h"
#include <aliceVision/types.hpp>

namespace fs = std::filesystem;
namespace bpt = boost::property_tree;

/// Define a collection of View
using Views = aliceVision::HashMap<aliceVision::IndexT, std::shared_ptr<View> >;

#define OursIG 0
#define OA 1
#define UV 2
#define RSE 3
#define APORA 4
#define Kr 5
#define NBVNET 6
#define PCNBV 7
#define NewOurs 8
#define MyIG 9

/**
 * The class that contains all the data needed for the NBV algorithm.
 */
class Share_Data {
public:
    // Variable input parameters
    std::string pcd_file_path;
    std::string yaml_file_path;
    std::string name_of_pcd;
    std::string nbv_net_path;

    // Number of viewpoints sampled at one time
    int num_of_views{};
    double cost_weight{};
    rs2_intrinsics color_intrinsics{};
    double depth_scale{};

    // Operating parameters
    // Process number
    int process_cnt;
    // System clock
    std::atomic<double> pre_clock{};
    // Is the process finished
    std::atomic<bool> over{};
    bool show{};
    int num_of_max_iteration{};

    // Point cloud data
    std::atomic<int> valid_clouds{};
    // Point cloud group
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcd;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ground_truth;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final;
    bool move_wait{};

    // Eight Forks Map
    octomap::ColorOcTree *octo_model;
    // octomap::ColorOcTree* cloud_model;
    octomap::ColorOcTree *ground_truth_model;
    octomap::ColorOcTree *GT_sample;
    double octomap_resolution{};
    double ground_truth_resolution{};
    double map_size{};
    double p_unknown_upper_bound{}; //! Upper bound for voxels to still be considered uncertain. Default: 0.97.
    double p_unknown_lower_bound{}; //! Lower bound for voxels to still be considered uncertain. Default: 0.12.

    // Workspace and viewpoint space
    std::atomic<bool> now_view_space_processed{};
    std::atomic<bool> now_views_information_processed{};
    std::atomic<bool> move_on{};

    Eigen::Matrix4d now_camera_pose_world;
    // Object centre
    Eigen::Vector3d object_center_world;
    // Object BBX radius
    double predicted_size{};

    int method_of_IG{};
    pcl::visualization::PCLVisualizer::Ptr viewer;

    double stop_thresh_map{};
    double stop_thresh_view{};

    double skip_coefficient{};

    double sum_local_information{};
    double sum_global_information{};

    double sum_robot_cost{};
    double camera_to_object_dis{};
    bool robot_cost_negative{};

    int num_of_max_flow_node;
    double interesting_threshold{};

    // Number of point cloud voxels
    int init_voxels{};
    // Number of voxels on the map
    int voxels_in_BBX{};
    // Map information entropy
    double init_entropy{};

    std::string save_path;

    std::vector<View> best_views{};

    aliceVision::sfmData::SfMData sfm_data{};

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
     * Return the time used and update the clock
     * @return elapsed_time : the time elapsed since the latest update
     */
    [[maybe_unused]] double out_clock();

    /**
     * Check for the existence of folders in multi-level directories and create them if they do not exist
     * @param cd The folder path
     */
    static void access_directory(const std::string &cd);

    /**
     * Storage of rotation matrix data to disk
     * @param T The rotation matrix to save
     * @param cd The path to save the file
     * @param name The name of the file
     * @param frames_cnt The frame number
     */
    [[maybe_unused]] void
    save_posetrans_to_disk(Eigen::Matrix4d &T, const std::string &cd, const std::string &name, int frames_cnt) const;

    /**
     * Save the octomap logs to the disk
     * @param voxels
     * @param entropy
     * @param cd
     * @param name
     * @param iterations
     */
    [[maybe_unused]] void
    save_octomap_log_to_disk(int voxels, double entropy, const std::string &cd, const std::string &name,
                             int iterations) const;

    /**
     * Store point cloud data to disk, very slow, rarely used
     *
     * @param cloud The point cloud to save
     * @param cd The path to save the point cloud
     * @param name The name of the point cloud to save
     */
    void save_cloud_to_disk(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::string &cd,
                            const std::string &name) const;

    /**
     * Store point cloud data to disk, very slow, rarely used
     * @param cloud The point cloud to save
     * @param cd The path to save the point cloud
     * @param name The name of the point cloud to save
     * @param frames_cnt The number of frames
     */
    [[maybe_unused]] void
    save_cloud_to_disk(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::string &cd,
                       const std::string &name,
                       int frames_cnt) const;

    /**
     * Store point cloud data to disk, very slow, rarely used
     * @param _octo_model The octree to save
     * @param cd The path to save the octree
     * @param name The name of the octree to save
     */
    [[maybe_unused]] void
    save_octomap_to_disk(octomap::ColorOcTree *_octo_model, const std::string &cd, const std::string &name) const;

};

#endif //NBV_SIMULATION_SHARE_DATA_H
