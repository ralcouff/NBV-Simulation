#include "Voxel_Information.h"

Voxel_Information::Voxel_Information(double _p_unknown_lower_bound, double _p_unknown_upper_bound) {
    p_unknown_upper_bound = _p_unknown_upper_bound;
    p_unknown_lower_bound = _p_unknown_lower_bound;
    k_vis = (0.0 - 1.0) / (p_unknown_upper_bound - p_unknown_lower_bound);
    b_vis = -k_vis * p_unknown_upper_bound;
}

double Voxel_Information::entropy(double &occupancy) {
    double p_free = 1 - occupancy;
    double vox_ig = 0;
    if (occupancy != 0 && p_free != 0) {
        vox_ig = -occupancy * log(occupancy) - p_free * log(p_free);
    }
    return vox_ig;
}

void Voxel_Information::init_mutex_voxels(int init_voxels) {
    mutex_voxels.resize(init_voxels);
    for (auto &mutex_voxel: mutex_voxels)
        mutex_voxel = new std::mutex;
}

bool Voxel_Information::is_unknown(double &occupancy) const {
    return occupancy < p_unknown_upper_bound && occupancy > p_unknown_lower_bound;
}

bool Voxel_Information::is_free(double &occupancy) const {
    return occupancy < p_unknown_lower_bound;
}

bool Voxel_Information::is_occupied(double &occupancy) const {
    return occupancy > p_unknown_upper_bound;
}

bool Voxel_Information::voxel_free(octomap::ColorOcTreeNode *traversed_voxel) const {
    double occupancy = traversed_voxel->getOccupancy();
    return is_free(occupancy);
}

bool Voxel_Information::voxel_occupied(octomap::ColorOcTreeNode *traversed_voxel) const {
    double occupancy = traversed_voxel->getOccupancy();
    return is_occupied(occupancy);
}

double Voxel_Information::voxel_object(octomap::OcTreeKey &voxel_key,
                                       std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *object_weight) {
    auto key = object_weight->find(voxel_key);
    if (key == object_weight->end())
        return 0;
    return key->second;
}

double Voxel_Information::voxel_quality(octomap::OcTreeKey &voxel_key,
                                        std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *quality_weight) {
    auto key = quality_weight->find(voxel_key);
    if (key == quality_weight->end())
        return 1;
//    std::cout << "JSP JPP JENAIMARRE" << std::endl;
    return key->second;
}

double Voxel_Information::get_voxel_visible(double occupancy) const {
    if (occupancy > p_unknown_upper_bound)
        return 0.0;
    if (occupancy < p_unknown_lower_bound)
        return 1.0;
    return k_vis * occupancy + b_vis;
}