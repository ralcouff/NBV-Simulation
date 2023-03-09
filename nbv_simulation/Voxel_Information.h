#ifndef NBV_SIMULATION_VOXEL_INFORMATION_H
#define NBV_SIMULATION_VOXEL_INFORMATION_H

#pragma once

#include <Eigen/Core>
#include <mutex>
#include <octomap/ColorOcTree.h>

class Voxel_Information {
public:
    double p_unknown_upper_bound;
    double p_unknown_lower_bound;
    double k_vis;
    double b_vis;
    std::mutex mutex_rays;
    std::vector<std::mutex *> mutex_voxels;
    std::vector<Eigen::Vector4d> convex;
    double skip_coefficient{};
    double octomap_resolution{};

    /**
     * Constructor of the Voxel_Information object
     * @param _p_unknown_lower_bound The lower boundary to determine if the voxel is unknown
     * @param _p_unknown_upper_bound The upper boundary to determine if the voxel is unknown
     */
    Voxel_Information(double _p_unknown_lower_bound, double _p_unknown_upper_bound);

    /**
     * Computes the Shannon entropy of the voxel, based on its occupancy
     * @param occupancy Occupancy probability of a voxel
     * @return The Shannon entropy of the voxel
     */
    static double entropy(double &occupancy);

    /**
     * Initialize the vector mutex_voxels with empty mutex
     * @param init_voxels The number of initial voxels
     */
    void init_mutex_voxels(int init_voxels);

    /**
     * Return true if the voxel is considered unknown i.e. Not occupied nor empty
     * @param occupancy The probability of occupancy
     * @return true if it is unknown, false otherwise
     */
    bool is_unknown(double &occupancy) const;

    /**
     * Checks if voxel is free according to its occupancy
     * @param occupancy The probability of occupancy of the voxel
     * @return TRUE if the voxel is free
     */
    bool is_free(double &occupancy) const;

    /**
     * Checks if the voxel is occupied according to its occupancy
     * @param occupancy The probability of occupancy of the voxel
     * @return TRUE if the voxel is occupied
     */
    bool is_occupied(double &occupancy) const;

    /**
     * Checks if the voxel is free
     * @param traversed_voxel The considered voxel
     * @return TRUE if the voxel is free.
     */
    bool voxel_free(octomap::ColorOcTreeNode *traversed_voxel) const;

    /**
     * Checks if the voxel is occupied
     * @param traversed_voxel The considered voxel
     * @return TRUE if the voxel is occupied.
     */
    bool voxel_occupied(octomap::ColorOcTreeNode *traversed_voxel) const;

    /**
     * Search the voxel in object_weight map according to its key.
     * @param voxel_key The key of the voxel in the unordered map
     * @param object_weight An unordered map containing the id of the voxel and the corresponding object weight p_obj
     * @return The object weight of the voxel corresponding to voxel_key if not found, return 0
     */
    static double voxel_object(octomap::OcTreeKey &voxel_key,
                               std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *object_weight);

    /**
     * Search the voxel in quality_weight map according to its key.
     * @param voxel_key The key of the voxel in the unordered map
     * @param quality_weight An unordered map containing the id of the voxel and the corresponding quality weight
     * @return The quality weight of the voxel corresponding to voxel_key if not found return 1
     */
    static double voxel_quality(octomap::OcTreeKey &voxel_key,
                                std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *quality_weight);

    /**
     * Remapping the upper/lower bound to [0,1] range and inverting it
     * @param occupancy The occupancy of the voxel
     * @return 0.0 if voxel occupied, 1.0 if free and a value in [0,1] if unknown
     */
    double get_voxel_visible(double occupancy) const;
};

#endif //NBV_SIMULATION_VOXEL_INFORMATION_H
