#include "Views_Information.h"
#include "Information.h"
#include "octomap/OcTreeKey.h"
#include <unordered_map>

Views_Information::Views_Information(Share_Data *share_data, Voxel_Information *_voxel_information,
                                     View_Space *view_space, int iterations) {
    /* Initializing the parameters */
    cost_weight = share_data->cost_weight;
    color_intrinsics = share_data->color_intrinsics;
    method = share_data->method_of_IG;
    octo_model = share_data->octo_model;
    octomap_resolution = share_data->octomap_resolution;
    voxel_information = _voxel_information;
    voxel_information->octomap_resolution = octomap_resolution;
    voxel_information->skip_coefficient = share_data->skip_coefficient;
    alpha = 0.1 / octomap_resolution;
    auto now_time = clock();

    /* Initializing all the maps and list needed */
    // Sorting the views by id to create the mapping
    sort(view_space->views.begin(), view_space->views.end(), view_id_compare);
    views_to_rays_map = new std::unordered_map<int, std::vector<int>>();
    rays_to_views_map = new std::unordered_map<int, std::vector<int>>();
    rays_map = new std::unordered_map<Ray, int, Ray_Hash>();
    object_weight = new std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>();
    occupancy_map = new std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>();
    quality_weight = new std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>();
    quality_weight = share_data->quality_weight;

    /* Computing the frontier voxels */
    std::vector<octomap::point3d> points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge(new pcl::PointCloud<pcl::PointXYZ>);
    double map_size = view_space->predicted_size;
    // Find edges in the map
    for (octomap::ColorOcTree::leaf_iterator it = octo_model->begin_leafs(), end = octo_model->end_leafs();
         it != end; ++it) {
        double occupancy = (*it).getOccupancy();
        // Record the mapping of key to occ rate in bbx for repeated queries
        (*occupancy_map)[it.getKey()] = occupancy;
        if (voxel_information->is_unknown(occupancy)) {
            auto coordinate = it.getCoordinate();
            if (coordinate.x() >= view_space->object_center_world(0) - map_size &&
                coordinate.x() <= view_space->object_center_world(0) + map_size &&
                coordinate.y() >= view_space->object_center_world(1) - map_size &&
                coordinate.y() <= view_space->object_center_world(1) + map_size &&
                coordinate.z() >= view_space->object_center_world(2) - map_size &&
                coordinate.z() <= view_space->object_center_world(2) + map_size) {
                points.push_back(coordinate);
                if (frontier_check(coordinate, octo_model, voxel_information, octomap_resolution) == 2)
                    edge->points.emplace_back(coordinate.x(), coordinate.y(), coordinate.z());
            }
        }
    }
    pre_edge_cnt = 0x3f3f3f3f;
    edge_cnt = edge->points.size();
    // Calculate the probability that the point in the map is the surface of an object based on the nearest frontier
    if (!edge->points.empty()) {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(edge);
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        for (auto &point: points) {
            octomap::OcTreeKey key;
            bool key_have = octo_model->coordToKeyChecked(point, key);
            if (key_have) {
                pcl::PointXYZ searchPoint(point.x(), point.y(), point.z());
                int num = kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
                if (num > 0) {
                    double p_obj = 1;
                    for (int j = 0; j < pointIdxNKNSearch.size(); j++) {
                        p_obj *= distance_function(pointNKNSquaredDistance[j], alpha);
                    }
                    (*object_weight)[key] = p_obj;
                    octo_model->integrateNodeColor(key, 0, 0, 255);
                }
            }
        }
    }
    cout << "Size of occupancy_map: " << occupancy_map->size() << endl;
    cout << "Size of edge: " << edge->points.size() << endl;
    cout << "Size of object_weight: " << object_weight->size() << endl;
    // Calculate the maximum number of rays according to BBX, the maximum number of rays is the size of the
    //  surface area * volume, used to allocate pointer memory
    double pre_line_point = 2.0 * map_size / octomap_resolution;
    long long superficial = ceil(5.0 * pre_line_point * pre_line_point);
    long long volume = ceil(pre_line_point * pre_line_point * pre_line_point);
    max_num_of_rays = superficial * volume;
    rays_info = new Ray_Information *[max_num_of_rays];
    cout << "There will be at most: " << max_num_of_rays << " rays" << endl;
    // Calculate the eight vertices of BBX for delineating the ray range
    std::vector<Eigen::Vector4d> convex_3d;
    double x1 = view_space->object_center_world(0) - map_size;
    double x2 = view_space->object_center_world(0) + map_size;
    double y1 = view_space->object_center_world(1) - map_size;
    double y2 = view_space->object_center_world(1) + map_size;
    double z1 = view_space->object_center_world(2) - map_size;
    double z2 = view_space->object_center_world(2) + map_size;
    convex_3d.emplace_back(x1, y1, z1, 1);
    convex_3d.emplace_back(x1, y2, z1, 1);
    convex_3d.emplace_back(x2, y1, z1, 1);
    convex_3d.emplace_back(x2, y2, z1, 1);
    convex_3d.emplace_back(x1, y1, z2, 1);
    convex_3d.emplace_back(x1, y2, z2, 1);
    convex_3d.emplace_back(x2, y1, z2, 1);
    convex_3d.emplace_back(x2, y2, z2, 1);
    voxel_information->convex = convex_3d;
    // Ray generators for assigning viewpoints
//    auto **ray_caster = new std::thread *[view_space->views.size()];
    // Initial subscripts for rays start from 0
    ray_num = 0;
//#pragma omp parallel default(shared)
    #pragma omp parallel for default(shared)
    for (int i = 0; i < view_space->views.size(); i++) {
        ray_cast_thread_process(&ray_num,
                                rays_info,
                                rays_map,
                                views_to_rays_map,
                                rays_to_views_map,
                                octo_model,
                                voxel_information,
                                view_space,
                                &color_intrinsics,
                                i);
        // Threads that divide into rays for this viewpoint
//        ray_caster[i] = new std::thread(ray_cast_thread_process,
//                                        &ray_num,
//                                        rays_info,
//                                        rays_map,
//                                        views_to_rays_map,
//                                        rays_to_views_map,
//                                        octo_model,
//                                        voxel_information,
//                                        view_space,
//                                        &color_intrinsics,
//                                        i);
    }
    /* Wait for each viewpoint ray generator to complete its calculation. */
//    for (int i = 0; i < view_space->views.size(); i++) {
//        (*ray_caster[i]).join();
//    }
    cout << "Number of rays (ray_num) : " << ray_num << endl;
    cout << "All views' rays generated in " << clock() - now_time << " ms. Starting computation of information."
         << endl;

    /* Allocate a thread to each ray. */
    now_time = clock();
//    auto **rays_process = new std::thread *[ray_num];
    #pragma omp parallel for default(shared)
    for (int i = 0; i < ray_num; i++) {
        ray_information_thread_process(i,
                                       rays_info,
                                       rays_map,
                                       occupancy_map,
                                       object_weight,
                                       quality_weight,
                                       octo_model,
                                       voxel_information,
                                       view_space,
                                       method);
//        rays_process[i] = new std::thread(ray_information_thread_process,
//                                          i,
//                                          rays_info,
//                                          rays_map,
//                                          occupancy_map,
//                                          object_weight,
//                                          quality_weight,
//                                          octo_model,
//                                          voxel_information,
//                                          view_space,
//                                          method);
    }
//    cout << "Finished that shit !" << endl;
//    cout << "----------------------------------------------------------------------------------------" << endl;
    /* Waiting for the ray calculation to be completed. */
//    for (int i = 0; i < ray_num; i++) {
//        (*rays_process[i]).join();
//    }
    auto cost_time = clock() - now_time;
    cout << "All rays' threads over with executed time " << cost_time << " ms." << endl;
    Share_Data::access_directory(share_data->savePath + "/run_time");
    std::ofstream fout(share_data->savePath + "/run_time/IG" + std::to_string(view_space->id) + ".txt");
    fout << cost_time << endl;
    // Information counters for assigning viewpoints
    now_time = clock();
    auto **view_gain = new std::thread *[view_space->views.size()];
    for (int i = 0; i < view_space->views.size(); i++) {
        // Threads that distribute information statistics for this viewpoint
        view_gain[i] = new std::thread(information_gain_thread_process, rays_info, views_to_rays_map, view_space, i);
    }
    // Wait for the information on each viewpoint to be counted
    for (int i = 0; i < view_space->views.size(); i++) {
        (*view_gain[i]).join();
    }
    cout << "All views' gain threads over with executed time " << clock() - now_time << " ms." << endl;
    // Displaying the repartition of quality in the octomap
    save_quality_map(share_data, quality_weight);
//    std::map<double, int> qlt_map{};
//    for (octomap::ColorOcTree::leaf_iterator it = share_data->octo_model->begin_leafs(), end = share_data->octo_model->end_leafs();
//         it != end; ++it) {
//        double qltt = Voxel_Information::voxel_quality(const_cast<octomap::OcTreeKey &>(it.getKey()), quality_weight);
//        qlt_map[qltt]++;
//    }
//    cout << "Quality in octo_model" << endl;
//    for (auto &it: qlt_map) {
//        std::cout << it.first << " - " << it.second << endl;
//    }
}

void Views_Information::update(Share_Data *share_data, View_Space *view_space, int iterations) {
    /* Re-setting the parameters */
    octo_model = share_data->octo_model;
    octomap_resolution = share_data->octomap_resolution;
    voxel_information->octomap_resolution = octomap_resolution;
    voxel_information->skip_coefficient = share_data->skip_coefficient;
    alpha = 0.1 / octomap_resolution;
    auto now_time = clock();

    /* Initializing all the maps and lists needed */
    // Sorting the views by id to create the mapping
    sort(view_space->views.begin(), view_space->views.end(), view_id_compare);
    // Clear viewpoint information
    for (auto &view: view_space->views) {
        view.information_gain = 0;
        view.voxel_num = 0;
    }
    // Avoid duplicate searches
    delete occupancy_map;
    occupancy_map = new std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>();
    delete object_weight;
    object_weight = new std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>();
//    delete quality_weight;
//    quality_weight = new std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>();
    quality_weight = share_data->quality_weight;
    /* Computing the frontier voxels */
    std::vector<octomap::point3d> points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge(new pcl::PointCloud<pcl::PointXYZ>);
    double map_size = view_space->predicted_size;
    for (octomap::ColorOcTree::leaf_iterator it = octo_model->begin_leafs(), end = octo_model->end_leafs();
         it != end;
         ++it) {
        double occupancy = (*it).getOccupancy();
        (*occupancy_map)[it.getKey()] = occupancy;
        if (voxel_information->is_unknown(occupancy)) {
            auto coordinate = it.getCoordinate();
            if (coordinate.x() >= view_space->object_center_world(0) - map_size &&
                coordinate.x() <= view_space->object_center_world(0) + map_size &&
                coordinate.y() >= view_space->object_center_world(1) - map_size &&
                coordinate.y() <= view_space->object_center_world(1) + map_size &&
                coordinate.z() >= view_space->object_center_world(2) - map_size &&
                coordinate.z() <= view_space->object_center_world(2) + map_size) {
                points.push_back(coordinate);
                if (frontier_check(coordinate, octo_model, voxel_information, octomap_resolution) == 2)
                    edge->points.emplace_back(coordinate.x(), coordinate.y(), coordinate.z());
            }
        }
    }
    edge_cnt = edge->points.size();
    if (edge_cnt > pre_edge_cnt)
        pre_edge_cnt = 0x3f3f3f3f;
    if (!edge->points.empty()) {
        // Calculating frontier
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(edge);
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        for (auto &point: points) {
            octomap::OcTreeKey key;
            bool key_have = octo_model->coordToKeyChecked(point, key);
            if (key_have) {
                pcl::PointXYZ searchPoint(point.x(), point.y(), point.z());
                int num = kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
                if (num > 0) {
                    double p_obj = 1;
                    for (int j = 0; j < pointIdxNKNSearch.size(); j++) {
                        p_obj *= distance_function(pointNKNSquaredDistance[j], alpha);
                    }
                    (*object_weight)[key] = p_obj;
                    octo_model->integrateNodeColor(key, (1-p_obj)*255, 0, p_obj*255);
                }
            }
        }
    }
    cout << "edge is " << edge->points.size() << endl;
    cout << "object_map is " << object_weight->size() << endl;
    cout << "occupancy_map is " << occupancy_map->size() << endl;
    cout << "frontier updated with executed time " << clock() - now_time << " ms." << endl;
    // Detects if re-generation
    now_time = clock();
    bool regenerate = false;
    if (view_space->object_changed) {
        regenerate = true;
    }
    // Update data structure if regenerated
    if (regenerate) {
        // Recalculation of the maximum number of rays, starting from 0
        double pre_line_point = 2.0 * map_size / octomap_resolution;
        long long superficial = ceil(5.0 * pre_line_point * pre_line_point);
        long long volume = ceil(pre_line_point * pre_line_point * pre_line_point);
        max_num_of_rays = superficial * volume;
        delete[] rays_info;
        rays_info = new Ray_Information *[max_num_of_rays];
        cout << "full rays num is " << max_num_of_rays << endl;
        ray_num = 0;
        delete views_to_rays_map;
        views_to_rays_map = new std::unordered_map<int, std::vector<int>>();
        delete rays_to_views_map;
        rays_to_views_map = new std::unordered_map<int, std::vector<int>>();
        delete rays_map;
        rays_map = new std::unordered_map<Ray, int, Ray_Hash>();

        // Calculate the eight vertices of BBX for delineating the ray range
        std::vector<Eigen::Vector4d> convex_3d;
        double x1 = view_space->object_center_world(0) - map_size;
        double x2 = view_space->object_center_world(0) + map_size;
        double y1 = view_space->object_center_world(1) - map_size;
        double y2 = view_space->object_center_world(1) + map_size;
        double z1 = view_space->object_center_world(2) - map_size;
        double z2 = view_space->object_center_world(2) + map_size;
        convex_3d.emplace_back(x1, y1, z1, 1);
        convex_3d.emplace_back(x1, y2, z1, 1);
        convex_3d.emplace_back(x2, y1, z1, 1);
        convex_3d.emplace_back(x2, y2, z1, 1);
        convex_3d.emplace_back(x1, y1, z2, 1);
        convex_3d.emplace_back(x1, y2, z2, 1);
        convex_3d.emplace_back(x2, y1, z2, 1);
        convex_3d.emplace_back(x2, y2, z2, 1);
        voxel_information->convex = convex_3d;
        // Ray generators for assigning viewpoints
//        auto **ray_caster = new std::thread *[view_space->views.size()];
        #pragma omp parallel for default(shared)
        for (int i = 0; i < view_space->views.size(); i++) {
            // Threads that divide into rays for this viewpoint
            ray_cast_thread_process(&ray_num,
                                    rays_info,
                                    rays_map,
                                    views_to_rays_map,
                                    rays_to_views_map,
                                    octo_model,
                                    voxel_information,
                                    view_space,
                                    &color_intrinsics,
                                    i);
//            ray_caster[i] = new std::thread(ray_cast_thread_process,
//                                            &ray_num,
//                                            rays_info,
//                                            rays_map,
//                                            views_to_rays_map,
//                                            rays_to_views_map,
//                                            octo_model,
//                                            voxel_information,
//                                            view_space,
//                                            &color_intrinsics,
//                                            i);
        }
        // Wait for each viewpoint ray generator to complete its calculation
//        for (int i = 0; i < view_space->views.size(); i++) {
//            (*ray_caster[i]).join();
//        }
        cout << "ray_num is " << ray_num << endl;
        cout << "All views' rays generated with executed time " << clock() - now_time << " ms. Starting computation."
             << endl;
    }
    // Allocate a thread to each ray
    now_time = clock();
//    auto **rays_process = new std::thread *[ray_num];
    #pragma omp parallel for default(shared)
    for (int i = 0; i < ray_num; i++) {
        rays_info[i]->clear();
//        rays_process[i] = new std::thread(ray_information_thread_process,
//                                          i,
//                                          rays_info,
//                                          rays_map,
//                                          occupancy_map,
//                                          object_weight,
//                                          quality_weight,
//                                          octo_model,
//                                          voxel_information,
//                                          view_space,
//                                          method);
        ray_information_thread_process(i,
                                       rays_info,
                                       rays_map,
                                       occupancy_map,
                                       object_weight,
                                       quality_weight,
                                       octo_model,
                                       voxel_information,
                                       view_space,
                                       method);
    }
    // Waiting for the ray calculation to be completed
//    for (int i = 0; i < ray_num; i++) {
//        (*rays_process[i]).join();
//    }
    auto cost_time = clock() - now_time;
    cout << "All rays' threads over with executed time " << cost_time << " ms." << endl;
    Share_Data::access_directory(share_data->savePath + "/run_time");
    std::ofstream fout(share_data->savePath + "/run_time/IG" + std::to_string(view_space->id) + ".txt");
    fout << cost_time << endl;
    now_time = clock();
    auto **view_gain = new std::thread *[view_space->views.size()];
    for (int i = 0; i < view_space->views.size(); i++) {
        // Threads that distribute information statistics for this viewpoint
        view_gain[i] = new std::thread(information_gain_thread_process, rays_info, views_to_rays_map, view_space, i);
    }
    // Wait for the information on each viewpoint to be counted
    for (int i = 0; i < view_space->views.size(); i++) {
        (*view_gain[i]).join();
    }
    cout << "All views' gain threads over with executed time " << clock() - now_time << " ms." << endl;

    save_quality_map(share_data, quality_weight);
//    std::map<double, int> qlt_map{};
//    for (octomap::ColorOcTree::leaf_iterator it = share_data->octo_model->begin_leafs(), end = share_data->octo_model->end_leafs();
//         it != end; ++it) {
//        double qltt = Voxel_Information::voxel_quality(const_cast<octomap::OcTreeKey &>(it.getKey()), quality_weight);
//        qlt_map[qltt]++;
//    }
//    cout << "Quality in octo_model" << endl;
//    for (auto &it: qlt_map) {
//        std::cout << it.first << " - " << it.second << endl;
//    }

}

void save_quality_map(Share_Data *share_data, std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *quality_weight) {
    std::map<double, int> qlt_map{};
    for (octomap::ColorOcTree::leaf_iterator it = share_data->octo_model->begin_leafs(), end = share_data->octo_model->end_leafs();
         it != end; ++it) {
        double qltt = Voxel_Information::voxel_quality(const_cast<octomap::OcTreeKey &>(it.getKey()), quality_weight);
        qlt_map[qltt]++;
    }
    cout << "Quality in octo_model" << endl;
    for (auto &it: qlt_map) {
        std::cout << it.first << " - " << it.second << endl;
    }
}
