#ifndef NBV_SIMULATION_VIEWS_VOXELS_MF_H
#define NBV_SIMULATION_VIEWS_VOXELS_MF_H

#pragma once

#include <thread>

#include "Views_Information.h"
#include "Voxel_Information.h"
#include "Share_Data.h"
#include "MCMF.h"
#include "View_Space.h"

class views_voxels_MF {
public:
    // Number of points on three sides, number of viewpoints nx, number of rays ny,number of voxels nz
    int nx, ny, nz;
    // Adjacency table
    std::vector<std::vector<std::pair<int, double>>>* bipartite_list;
    View_Space* view_space;
    Views_Information* views_information;
    Voxel_Information* voxel_information;
    Share_Data* share_data;
    // Voxel subscript
    std::unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>* voxel_id_map;
    MCMF* mcmf;
    std::vector<int> view_id_set;

    views_voxels_MF(int _nx, View_Space* _view_space, Views_Information* _views_information,
                    Voxel_Information* _voxel_information, Share_Data* _share_data);
    ~views_voxels_MF();
    void solve();
    vector<int> get_view_id_set();

};

void adjacency_list_thread_process(int ray_id,
                                   int* nz,
                                   int ray_index_shift,
                                   int voxel_index_shift,
                                   unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>* voxel_id_map,
                                   vector<vector<pair<int, double>>>* bipartite_list,
                                   View_Space* view_space,
                                   Views_Information* views_information,
                                   Voxel_Information* voxel_information,
                                   Share_Data* share_data);

#endif //NBV_SIMULATION_VIEWS_VOXELS_MF_H
