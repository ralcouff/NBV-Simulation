#ifndef NBV_SIMULATION_VIEWS_VOXELS_MF_H
#define NBV_SIMULATION_VIEWS_VOXELS_MF_H

#pragma once

#include <thread>

#include "MCMF.h"
#include "Share_Data.h"
#include "View_Space.h"
#include "Views_Information.h"
#include "Voxel_Information.h"

/**
 * The class implementing the Max Flow Graph
 */
class views_voxels_MF {
public:
    /* Number of points on three sides, number of viewpoints nx, number of rays ny, number of voxels nz */
    int nx, ny, nz;
    // Adjacency table
    std::vector<std::vector<std::pair<int, double>>> *bipartite_list;
    View_Space *view_space;
    Views_Information *views_information;
    Voxel_Information *voxel_information;
    Share_Data *share_data;
    // Voxel subscript
    std::unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash> *voxel_id_map;
    MCMF *mcmf;
    std::vector<int> view_id_set;

    /**
     * The constructor of the views_voxel_MF
     * @param _nx The number of viewpoints
     * @param _view_space The considered View Space
     * @param _views_information The information contained by each view (I_local : eq. 1)
     * @param _voxel_information TODO
     * @param _share_data The data shared through the whole project
     */
    views_voxels_MF(int _nx, View_Space *_view_space, Views_Information *_views_information,
                    Voxel_Information *_voxel_information, Share_Data *_share_data);

    /**
     * Destructor of the views_voxels_MF object
     */
    ~views_voxels_MF();

    /**
     * Solve the graph by Max Flow
     */
    void solve();

    /**
     * Getter to the view_id_set parameter.
     * @return The ids of the views satisfying the set cover problem.
     */
    [[nodiscard]] vector<int> get_view_id_set() const;

};

/**
 * Fill the bipartite adjacency list [Views -> Rays -> Voxels]
 * For each category, it has a pair containing
 * (origin, weight)
 * @param ray_id The id of the ray
 * @param nz The number of voxels in the MF
 * @param ray_index_shift The number of viewpoints
 * @param voxel_index_shift The number of viewpoints + the number of rays
 * @param voxel_id_map TODO
 * @param bipartite_list TODO
 * @param view_space The considered View Space
 * @param views_information  The views information
 * @param voxel_information The voxels information
 * @param share_data The data shared through the whole project
 */
void adjacency_list_thread_process(int ray_id,
                                   int *nz,
                                   int ray_index_shift,
                                   int voxel_index_shift,
                                   unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash> *voxel_id_map,
                                   vector<vector<pair<int, double>>> *bipartite_list,
                                   View_Space *view_space,
                                   Views_Information *views_information,
                                   Voxel_Information *voxel_information,
                                   Share_Data *share_data);

#endif //NBV_SIMULATION_VIEWS_VOXELS_MF_H
