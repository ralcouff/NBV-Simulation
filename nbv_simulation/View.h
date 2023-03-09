#ifndef NBV_SIMULATION_VIEW_H
#define NBV_SIMULATION_VIEW_H

#pragma once

#include <bitset>
#include <Eigen/Core>
#include <Eigen/Geometry>

class View {
public:
    int space_id{}; // The View Space id containing the view
    int id{}; // The id of the view
    Eigen::Vector3d init_pos; // Position of the view (x,y,z)
    Eigen::Matrix4d pose; // View_i to View_i+1 rotation matrix
    int vis; // The number of times the view has been visited
    double information_gain; // The information gain brought by this view
    int voxel_num;
    std::bitset<64> in_coverage;
    double final_utility; // The global final utility


    /**
     * Constructor of the view from a Vector3d
     * @param _init_pos Vector3d containing the position of the view
     */
    explicit View(Eigen::Vector3d _init_pos);

    /**
     * Constructor of a view from an other view
     * @param other The other view to build the new view
     */
    View(const View &other);

    /**
     * Get to the next camera position and set the pose parameter
     * @param now_camera_pose_world The next camera pose in the current reference model
     * @param object_center_world The current center of the scene
     */
    void get_next_camera_pos(const Eigen::Matrix4d &now_camera_pose_world, Eigen::Vector3d object_center_world);

    /**
     * Computing his(v,k) (eq. (4))
     * @param x The difference (it-k)
     * @return the value of his(v,k)
     */
    static double global_function(int x);

    /**
     * Computes the I_flow function (eq. (5))
     * @return The value of I_flow for the considered view
     */
    double get_global_information();
};

/**
 * Comparison function for ids of the views
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
