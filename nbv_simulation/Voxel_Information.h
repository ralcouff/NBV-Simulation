#ifndef NBV_SIMULATION_VOXEL_INFORMATION_H
#define NBV_SIMULATION_VOXEL_INFORMATION_H

#pragma once

#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>

#include <Eigen/Core>

#include <mutex>

class Voxel_Information {
public:
    double p_unknown_upper_bound;
    double p_unknown_lower_bound;
    double k_vis;
    double b_vis;
    double quality;
    std::mutex mutex_rays;
    std::vector<std::mutex*> mutex_voxels;
    std::vector<Eigen::Vector4d> convex;
    double skip_coefficient;
    double octomap_resolution;

    Voxel_Information(double _p_unknown_lower_bound, double _p_unknown_upper_bound);
    void init_mutex_voxels(int init_voxels);
    double entropy(double& occupancy);
    bool is_known(double& occupancy);
    bool is_unknown(double& occupancy);
    bool is_free(double& occupancy);
    bool is_occupied(double& occupancy);
    bool voxel_unknown(octomap::ColorOcTreeNode* traversed_voxel);
    bool voxel_free(octomap::ColorOcTreeNode* traversed_voxel);
    bool voxel_occupied(octomap::ColorOcTreeNode* traversed_voxel);
    double get_voxel_visible(double occupancy);
    double get_voxel_visible(octomap::ColorOcTreeNode* traversed_voxel);
    double get_voxel_information(octomap::ColorOcTreeNode* traversed_voxel);
    double voxel_object(octomap::OcTreeKey& voxel_key,
                        std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>* object_weight);
    double get_voxel_object_visible(
            octomap::OcTreeKey& voxel_key,
            std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>* object_weight);
};

#endif //NBV_SIMULATION_VOXEL_INFORMATION_H
