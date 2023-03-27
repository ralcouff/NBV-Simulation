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
    int id; // The id of the View Space
    Share_Data *share_data; // The shared data among the whole project
    Voxel_Information *voxel_information;

    int num_of_views; //Number of viewpoints
    std::vector<View> views; // Sampled viewpoints in space

    Eigen::Vector3d object_center_world; // Object center of mass
    Eigen::Matrix4d now_camera_pose_world; // The current camera pose
    double predicted_size{}; // The estimated BBOX radius of the object

    double octomap_resolution; // The current octomap resolution
    octomap::ColorOcTree *octo_model; // The current octomap

    std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> *views_key_set; // A set containing the octree keys of the views

    int occupied_voxels{}; //TODO
    double map_entropy{}; //TODO
    bool object_changed; //TODO

    pcl::visualization::PCLVisualizer::Ptr viewer; //TODO
    double height_of_ground{}; //TODO : MAYBE UNUSED
    double cut_size{}; //TODO : MAYBE UNUSED
    double camera_to_object_dis{}; //TODO : MAYBE UNUSED


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
     * Generate a set of viewpoints in the octomap (views_key_set)
     * @param points The 3D points of the point cloud
     */
    void get_view_space(std::vector<Eigen::Vector3d> &points);

    /**
     * Checks if the points are within a ball of radius predicted size around the center of mass of the object.
     * @param _predicted_size The radius of the ball to determine
     * @param points A vector of points to check
     * @return The percentage of points inside that ball
     */
    double check_size(double _predicted_size, std::vector<Eigen::Vector3d> &points);

    /**
     * Checking if the generated view is valid ie. not within a two times expansion of BBOX, adn in a 4 time radius
     * @param view The candidate view
     * @return TRUE if valid FALSE if not
     */
    bool valid_view(View &view);

    /**
     * Save the current View Space with the number of views, the object center and the predicted size of the model,
     * as well as the position of each view.
     */
    void save_view_space();

    /**
     * Updating View Space
     * Updating robot cost for each view. Filling the octo_map with update_cloud. Compute the occupancy and the map entropy.
     * Saving the octomaps.
     * @param _id The id
     * @param _share_data The data shared through the whole files
     * @param cloud FIXME : Unused
     * @param update_cloud The cloud used to update the octomap
     */
    void update(int _id,
                Share_Data *_share_data,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &update_cloud);

    /**
     * Adding the BBOx of the object to a PCLVisualizer in green
     * @param visualizer The PCLVisualizer onto which we want to display the bbox
     */
    void add_bbx_to_cloud(const pcl::visualization::PCLVisualizer::Ptr &visualizer);

};

[[maybe_unused]] void add_trajectory_to_cloud(Eigen::Matrix4d now_camera_pose_world,
                             vector<Eigen::Vector3d> &points,
                             pcl::visualization::PCLVisualizer::Ptr viewer);

[[maybe_unused]] void delete_trajectory_in_cloud(int num, pcl::visualization::PCLVisualizer::Ptr viewer);

#endif //NBV_SIMULATION_VIEW_SPACE_H
