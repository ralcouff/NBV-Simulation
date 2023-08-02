#ifndef NBV_SIMULATION_SHARE_DATA_H
#define NBV_SIMULATION_SHARE_DATA_H

#pragma once

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <filesystem>
#include <string>
#include <utility>

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

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmData/View.hpp>
#include <aliceVision/sfmDataIO/jsonIO.hpp>
#include <aliceVision/types.hpp>

#include "utils.cpp"
#include "View.h"

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
#define Test_local 10
#define Test_flow 11
#define Test_qlt 101
#define Test_two 102

/**
 * The class that contains all the data needed for the NBV algorithm.
 */
class Share_Data {
public:

    // Variable input parameters
    std::string yamlConfigFilePath;
    std::string objectFolderPath;
    std::string nameOfObject;
    std::string nbvNetPath;
    std::string objectFilePath;
    std::string viewSpaceFilePath;
    std::string qualityFilePath;
    std::string savePath;
    std::string pythonMetricsPath;
    std::string pythonSavesPath;

    // Point cloud data
    std::atomic<int> valid_clouds{};
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds; // A group containing all the clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcd; // The input cloud from the reconstruction algorithm
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ground_truth; // The Ground Truth cloud containing the rescaled input cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final; // The final cloud, output of the algorithm
    int init_voxels{}; // Number of point cloud voxels

    // Octomaps
    octomap::ColorOcTree *octo_model; //TODO
    // octomap::ColorOcTree* cloud_model; //TODO
    octomap::ColorOcTree *ground_truth_model; //TODO
    octomap::ColorOcTree *GT_sample; //TODO
    double octomap_resolution{}; // The resolution of the current octomap model
    double ground_truth_resolution{}; // The resolution of the Ground Truth octomap
    double p_unknown_upper_bound{}; //! Upper bound for voxels to still be considered uncertain. Default: 0.97.
    double p_unknown_lower_bound{}; //! Lower bound for voxels to still be considered uncertain. Default: 0.12.
    int voxels_in_BBX{}; // Number of voxels in the map
    double init_entropy{}; // Map information entropy

    // Process flags and parameters
    std::atomic<bool> over{}; // Checks if the process is over
    std::atomic<bool> now_view_space_processed{}; // Is the View Space processed
    std::atomic<bool> now_views_information_processed{};  // Are the Views Information processed
    std::atomic<bool> move_on = false; // Did we continue
    int process_cnt; // Process number
    std::atomic<double> pre_clock{}; // System clock

    // Quality set
    std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *quality_weight; // A quality map for each voxel
    std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *gt_quality_weight{}; // A quality map for each voxel
    std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *gt_sample_quality_weight{}; // A quality map for each voxel

    // Position matrices
    Eigen::Matrix4d now_camera_pose_world; // The current camera pose considered as the NBV
    Eigen::Vector3d object_center_world; // The barycenter of the object, used as the center of the world
    double predicted_size{}; // The estimated radius of the object
    double map_size{}; // The radius of the octomap

    //Pipeline parameters
    std::atomic<short> method_of_IG{}; // Method of Information Gain computation
    int alt_method_of_IG; // Method of Information Gain computation
    int num_of_max_iteration{}; // Number of maximum iteration
    int num_of_views{}; // Number of views sampled around the object
    double cost_weight{}; // Gamma in Information Gain computation
    double skip_coefficient{}; // A coefficient to skip some pixels in the projected image
    double sum_local_information{}; // The sum of all local information
    double sum_global_information{}; // The sum of all global information
    double sum_robot_cost{}; // The sum of robot cost
    std::vector<View> best_views{}; // A list containing the successive best views
    bool show{}; // To show the interface or not
    bool move_wait{}; // User input to say if we continue between each iteration or not
    bool robot_cost_negative{}; //TODO
    int num_of_max_flow_node; // The maximum number of nodes for the max flow algorithm.
    double interesting_threshold{}; // The threshold to determine if it's interesting or not in the MCMF


    // Camera parameters
    rs2_intrinsics color_intrinsics{};
    double depth_scale{};

    // A visualizer for the octomaps and clouds
    pcl::visualization::PCLVisualizer::Ptr viewer;

    // A sfm Data to store the results
    aliceVision::sfmData::SfMData sfm_data{};

    // Unused variables
    double stop_thresh_map{};
    double stop_thresh_view{};
    double camera_to_object_dis{};

    // Test Pipeline parameters.
    std::string folder_name;
    std::string current_input_filename;
    std::string test_base_filename;
    std::string string_test_time;
    int n_model;
    int n_size;
    int n_test;
    int reconstructionIterations;


    /**
     * Constructor of the Share_Data object
     * It is initialized with a file
     * @param _config_file_path The configuration initialization file
     * @param _model_path
     * @param _model_qlt_path
     * @param _method
     * @param _n_iter
     * @param _save_folder
     * @param _string_test_time
     */
    explicit Share_Data(const std::string& _config_file_path, const std::string& _model_path, const std::string& _model_qlt_path, short _method, int _n_iter, const std::string& _save_folder, const std::string& _string_test_time);


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
    void save_cloud_to_disk(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
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

/**
 * Get the method corresponding to the int passed in parameters
 * @param n_test The int of the method
 */
short get_method(int n_test);

#endif //NBV_SIMULATION_SHARE_DATA_H
