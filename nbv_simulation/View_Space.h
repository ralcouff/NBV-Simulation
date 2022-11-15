#ifndef NBV_SIMULATION_VIEW_SPACE_H
#define NBV_SIMULATION_VIEW_SPACE_H

#pragma once

#include <Eigen/Core>
#include <unordered_set>

#include "View.h"
#include "Voxel_Information.h"
#include "Share_Data.h"

class View_Space {
public:
    /* Number of ViewPoints. */
    int num_of_views;
    /* Sampled viewpoints in space. */
    std::vector<View> views;
    /* Object center. */
    Eigen::Vector3d object_center_world;
    // Object BBX radius
    double predicted_size{};
    // The first few nbv iterations
    int id;
    // Camera positions for this nbv iteration
    Eigen::Matrix4d now_camera_pose_world;
    int occupied_voxels{};
    double map_entropy{};
    bool object_changed;
    double octomap_resolution;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    double height_of_ground{};
    double cut_size{};
    std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> *views_key_set;
    octomap::ColorOcTree *octo_model;
    Voxel_Information *voxel_information;
    double camera_to_object_dis{};
    Share_Data *share_data;

    /**
     * Creates a set of potential viewpoints for the algorithm
     * @param _id The id of the View Space
     * @param _share_data The data shared by the project
     * @param _voxel_information The voxel information associated to the octree
     * @param cloud The original point cloud
     */
    View_Space(int _id,
               Share_Data *_share_data,
               Voxel_Information *_voxel_information,
               const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    /**
     * Checking if the generated view is valid ie. not within a two times expansion of BBOX, adn in a 4 time radius
     * @param view The candidate view
     * @return TRUE if valid FALSE if not
     */
    bool valid_view(View &view);

    double check_size(double predicted_size, std::vector<Eigen::Vector3d> &points);

    /**
     * Generate a set of viewpoints in the octomap (views_key_set)
     * @param points The 3D points of the point cloud
     */
    void get_view_space(std::vector<Eigen::Vector3d> &points);

    void update(int _id,
                Share_Data *_share_data,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr update_cloud);

    void add_bbx_to_cloud(pcl::visualization::PCLVisualizer::Ptr viewer);

};

void add_trajectory_to_cloud(Eigen::Matrix4d now_camera_pose_world,
                             vector<Eigen::Vector3d> &points,
                             pcl::visualization::PCLVisualizer::Ptr viewer);

void delete_trajectory_in_cloud(int num, pcl::visualization::PCLVisualizer::Ptr viewer);

#endif //NBV_SIMULATION_VIEW_SPACE_H
