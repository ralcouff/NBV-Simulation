#ifndef NBV_SIMULATION_VIEWS_INFORMATION_H
#define NBV_SIMULATION_VIEWS_INFORMATION_H

#pragma once

#include "View_Space.h"
#include "Ray.h"
#include "Ray_Hash.h"
#include "Information.h"
#include "Ray_Information.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <unordered_map>
#include <vector>
#include <thread>


class Views_Information {
public:
    double cost_weight; // The cost used for the final mixing in the information gain
    rs2_intrinsics color_intrinsics{}; // The intrinsic parameters of the camera
    int method; // The method used for the computation of Global Information
    octomap::ColorOcTree *octo_model; // The current octomap
    double octomap_resolution; // The resolution of the current octomap
    double alpha; // TODO:
    Voxel_Information *voxel_information; // Voxel Information object
    int pre_edge_cnt;
    std::size_t edge_cnt;
    int K = 6; // The number of nearest neighbors
    long long max_num_of_rays; // Maximum number of rays
    Ray_Information **rays_info; // Object containing the Information on rays
    int ray_num; // The current ray number


    std::unordered_map<int, std::vector<int>> *views_to_rays_map; // Map containing for each view its rays
    std::unordered_map<int, std::vector<int>> *rays_to_views_map; // Map containing for each ray, its views
    std::unordered_map<Ray, int, Ray_Hash> *rays_map; // Map containing for each ray_id the hash in the octomap
    std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *occupancy_map; // Map containing the occupancy of each voxel in the octomap.
    std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *object_weight; // P(obj_x) (eq. (2)
    std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *quality_weight; // A quality map for each voxel

    /**
     * Compute the information gain for each view of the View Space [eq. (1)]
     * @param share_data The data shared through the whole project.
     * @param _voxel_information The information contained in each voxel
     * @param view_space The considered View Space
     * @param iterations FIXME : unused
     */
    Views_Information(Share_Data *share_data, Voxel_Information *_voxel_information, View_Space *view_space,
                      int iterations);
};

#endif //NBV_SIMULATION_VIEWS_INFORMATION_H
