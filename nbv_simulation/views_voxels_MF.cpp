#include "views_voxels_MF.h"

views_voxels_MF::views_voxels_MF(int _nx, View_Space *_view_space, Views_Information *_views_information,
                                 Voxel_Information *_voxel_information, Share_Data *_share_data) {
    auto now_time = clock();
    view_space = _view_space;
    views_information = _views_information;
    voxel_information = _voxel_information;
    share_data = _share_data;
    /* Viewpoints are sorted by id and a trilateration adjacency table is created. */
    sort(view_space->views.begin(), view_space->views.end(), view_id_compare);
    nx = _nx;
    ny = views_information->ray_num;
    bipartite_list = new std::vector<std::vector<std::pair<int, double>>>;
    bipartite_list->resize(nx + ny + share_data->voxels_in_BBX);
    // Create a table of ids for voxels
    voxel_id_map = new std::unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>;
    // Parallel traversal of the voxels on each ray to the corresponding viewpoint
    nz = 0;
    auto **adjacency_list_process = new std::thread *[views_information->ray_num];
    for (int i = 0; i < views_information->ray_num; i++) {
        adjacency_list_process[i] = new std::thread(adjacency_list_thread_process,
                                                    i,
                                                    &nz,
                                                    nx,
                                                    nx + ny,
                                                    voxel_id_map,
                                                    bipartite_list,
                                                    view_space,
                                                    views_information,
                                                    voxel_information,
                                                    share_data);
//        adjacency_list_thread_process(i, &nz, nx, nx + ny, voxel_id_map, bipartite_list, view_space, views_information,
//                                      voxel_information, share_data);
    }
    for (int i = 0; i < views_information->ray_num; i++) {
        (*adjacency_list_process[i]).join();
    }
    // Output the exact figure size
    if (nz != voxel_id_map->size())
        cout << "node_z wrong." << endl;
    std::size_t num_of_all_edge = 0;
    std::size_t num_of_view_edge = 0;
    for (int i = 0; i < bipartite_list->size(); i++) {
        num_of_all_edge += (*bipartite_list)[i].size();
        if (i > nx && i < nx + ny)
            num_of_view_edge += (*bipartite_list)[i].size();
    }
    cout << "Full edge is " << num_of_all_edge << ". View edge(in) is " << num_of_view_edge
         << ". Voxel edge(out) is " << num_of_all_edge - num_of_view_edge << "." << endl;
    cout << "adjacency list with interested voxels num " << ny << " got with executed time "
         << clock() - now_time << " ms." << endl;
    mcmf = new MCMF();
}

views_voxels_MF::~views_voxels_MF() {
    delete bipartite_list;
    delete voxel_id_map;
    delete mcmf;
}

void views_voxels_MF::solve() {
    auto now_time = clock();
    view_id_set = mcmf->work(*bipartite_list);
    auto cost_time = clock() - now_time;
    cout << "Flow network solved in : " << cost_time << " ms." << endl;
    cout << view_id_set.size() << " views got by max flow." << endl;
    Share_Data::access_directory(share_data->savePath + "/run_time");
    std::ofstream fout(share_data->savePath + "/run_time/MF" + std::to_string(view_space->id) + ".txt");
    fout << cost_time << '\t' << view_id_set.size() << endl;
}

vector<int> views_voxels_MF::get_view_id_set() const {
    return view_id_set;
}

void adjacency_list_thread_process(int ray_id, int *nz, int ray_index_shift, int voxel_index_shift,
                                   unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash> *voxel_id_map,
                                   vector<vector<pair<int, double>>> *bipartite_list, View_Space *view_space,
                                   Views_Information *views_information, Voxel_Information *voxel_information,
                                   Share_Data *share_data) {
    // Which viewpoints the ray is seen by, added to the diagram
    vector<int> views_id = (*views_information->rays_to_views_map)[ray_id];
    for (int &i: views_id)
        (*bipartite_list)[ray_id + ray_index_shift].emplace_back(i, 0.0);
    // Retain only voxels of interest
    double visible = 1.0;
//    auto first = views_information->rays_info[ray_id]->ray->start;
//    auto last = views_information->rays_info[ray_id]->ray->stop;
    for (auto it = views_information->rays_info[ray_id]->ray->start;
         it != views_information->rays_info[ray_id]->ray->stop;
         ++it) {
        // Look up the key from the hash table
        auto hash_this_key = (*views_information->occupancy_map).find(*it);
        // Next if you can't find a node
        if (hash_this_key == (*views_information->occupancy_map).end())
            continue;
        // Read node probability values
        double occupancy = hash_this_key->second;
        // Read the node for the surface rate of the object
        double on_object = Voxel_Information::voxel_object(*it, views_information->object_weight);
        /* Statistical information entropy
        Definition 1 in Global Optimality*/
        double information_gain = on_object * visible * Voxel_Information::entropy(occupancy);
        visible *= voxel_information->get_voxel_visible(occupancy);
        if (information_gain > share_data->interesting_threshold) {
            octomap::OcTreeKey node_y = *it;
            int voxel_id;
            voxel_information->mutex_rays.lock();
            auto hash_this_node = voxel_id_map->find(node_y);
            // If not recorded, it is considered a new voxel
            if (hash_this_node == voxel_id_map->end()) {
                voxel_id = (*nz) + voxel_index_shift;
                (*voxel_id_map)[node_y] = voxel_id;
                (*nz)++;
            } else {
                voxel_id = hash_this_node->second;
            }
            voxel_information->mutex_rays.unlock();
            // For each viewpoint, count the id and value of the voxel
            for (int i = 0; i < views_id.size(); i++) {
                (*voxel_information->mutex_voxels[voxel_id - voxel_index_shift]).lock();
                (*bipartite_list)[voxel_id].emplace_back(ray_id + ray_index_shift, information_gain);
                (*voxel_information->mutex_voxels[voxel_id - voxel_index_shift]).unlock();
            }
        }
    }
}
