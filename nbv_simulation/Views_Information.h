#ifndef NBV_SIMULATION_VIEWS_INFORMATION_H
#define NBV_SIMULATION_VIEWS_INFORMATION_H

#pragma once

#include "Ray_Hash.h"
#include "Ray_Information.h"
#include "Share_Data.h"
#include "View_Space.h"
#include "Information.h"

#include <unordered_map>
#include <vector>
#include <thread>

class Views_Information {

public:
    double cost_weight;
    Ray_Information **rays_info;
    std::unordered_map<int, std::vector<int>> *views_to_rays_map;
    std::unordered_map<int, std::vector<int>> *rays_to_views_map;
    std::unordered_map<Ray, int, Ray_Hash> *rays_map;
    std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *occupancy_map;
    std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *object_weight; // P(obj_x) (eq. (2)
    std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *quality_weight; // My quality
    long long max_num_of_rays;
    int ray_num;
    double alpha;
    int K = 6;
    rs2_intrinsics color_intrinsics{};
    Voxel_Information *voxel_information;
    octomap::ColorOcTree *octo_model;
    double octomap_resolution;
    int method;
    int pre_edge_cnt;
    std::size_t edge_cnt;

    /**
     * Compute the information gain for each view of the View Space [eq. (1)]
     * @param share_data The data shared through the whole project.
     * @param _voxel_information The information contained in each voxel
     * @param view_space The considered View Space
     * @param iterations FIXME : unused
     */
    Views_Information(Share_Data *share_data, Voxel_Information *_voxel_information, View_Space *view_space,
                      int iterations);

    /**
     * Updating the information gain for each view of the View Space
     * @param share_data The data shared through the whole project
     * @param view_space The considered View Space
     * @param iterations FIXME : unused
     */
    void update(Share_Data *share_data, View_Space *view_space, int iterations);
};

#endif //NBV_SIMULATION_VIEWS_INFORMATION_H
