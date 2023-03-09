#ifndef NBV_SIMULATION_VIEW_SPACE_H
#define NBV_SIMULATION_VIEW_SPACE_H

#pragma once

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
    Eigen::Matrix4d current_camera_pose_world; // The current camera pose
    double predicted_size{}; // The estimated BBOX radius of the object

    double octomap_resolution; // The current octomap resolution
    octomap::ColorOcTree *octo_model; // The current octomap

    std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> *views_key_set; // A set containing the octree keys of the views


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
     * Checking if the generated view is valid ie. not within a two times radius expansion of BBOX,
     * and in a 4 times radius
     * @param view The candidate view
     * @return TRUE if valid FALSE if not
     */
    bool valid_view(View &view);

    /**
     * Save the current View Space with the number of views, the object center and the predicted size of the model,
     * as well as the position of each view.
     */
    void save_view_space();

};

#endif //NBV_SIMULATION_VIEW_SPACE_H
