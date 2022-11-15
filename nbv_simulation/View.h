#ifndef NBV_SIMULATION_VIEW_H
#define NBV_SIMULATION_VIEW_H

#pragma once

#include <bitset>

#include <Eigen/Core>

#include <pcl/visualization/pcl_visualizer.h>

class View {

public:
    int space_id;
    int id;
    /* Initial position */
    Eigen::Vector3d init_pos;
    // view_i to view_i+1 rotation matrix
    Eigen::Matrix4d pose;
    double information_gain;
    int voxel_num;
    double robot_cost;
    double dis_to_object;
    double final_utility;
    std::atomic<bool> robot_moved;
    int path_num;
    int vis;
    bool can_move;
    std::bitset<64> in_coverage;

    explicit View(Eigen::Vector3d _init_pos);

    View(const View &other);

    View &operator=(const View &other);

    static double global_function(int x);

    double get_global_information();

    void get_next_camera_pos(Eigen::Matrix4d now_camera_pose_world, Eigen::Vector3d object_center_world);

    void add_view_coordinates_to_cloud(Eigen::Matrix4d now_camera_pose_world,
                                       pcl::visualization::PCLVisualizer::Ptr viewer,
                                       int space_id);
};

/**
 * Compares the IDs of two views
 * @param a First view
 * @param b Second View
 * @return True if a.id < b.id
 */
bool view_id_compare(View &a, View &b);

/**
 * Compares the utility of two views.
 * @param a First view
 * @param b Second View
 * @return True if a.final_utility > b.final_utility
 */
bool view_utility_compare(View &a, View &b);

#endif //NBV_SIMULATION_VIEW_H
