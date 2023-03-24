#include "Views_Information.h"

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


    /* Computing the frontier voxels */
    std::vector<octomap::point3d> points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge(new pcl::PointCloud<pcl::PointXYZ>);
    double map_size = view_space->predicted_size;
    for (octomap::ColorOcTree::leaf_iterator it = octo_model->begin_leafs(), end = octo_model->end_leafs();
         it != end; ++it) {
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
    pre_edge_cnt = 0x3f3f3f3f;
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
                }
            }
        }
    }

    ifstream quality_file(share_data->quality_file_path);
    if (quality_file.is_open()) {
        for (auto &pt: share_data->working_cloud->points) {
            std::string line;
            if (getline(quality_file, line)) {
                octomap::OcTreeKey key;
                bool key_have = octo_model->coordToKeyChecked(pt.x, pt.y, pt.z, key);
                if (key_have) {
                    if ((*quality_weight).find(key) == (*quality_weight).end())
                        (*quality_weight)[key] = std::stod(line);
                    else
                        (*quality_weight)[key] = std::min((*quality_weight)[key],std::stod(line));
                }
            }
        }
        quality_file.close();
        cout << "Quality file has been read" << endl;
    } else {
        cout << "No Quality file has been found" << endl;
    }

    // Displaying the repartition of quality in the octomap
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
    cout << "There are: " << max_num_of_rays << " rays" << endl;
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
    auto **ray_caster = new std::thread *[view_space->views.size()];
    // Initial subscripts for rays start from 0
    ray_num = 0;
    //TODO: Multi-thread it
//    for (int i = 0; i < view_space->views.size(); i++) {
//        ray_cast_thread_process(&ray_num, rays_info, rays_map, views_to_rays_map, rays_to_views_map, octo_model,
//                                voxel_information, view_space, &color_intrinsics, i);
//    }
    for (int i = 0; i < view_space->views.size(); i++) {
        // Threads that divide into rays for this viewpoint
        ray_caster[i] = new std::thread(ray_cast_thread_process,
                                        &ray_num,
                                        rays_info,
                                        rays_map,
                                        views_to_rays_map,
                                        rays_to_views_map,
                                        octo_model,
                                        voxel_information,
                                        view_space,
                                        &color_intrinsics,
                                        i);
    }
//     Wait for each viewpoint ray generator to complete its calculation.
    for (int i = 0; i < view_space->views.size(); i++) {
        (*ray_caster[i]).join();
    }
    cout << "Number of rays (ray_num) : " << ray_num << endl;
    cout << "All views' rays generated in " << clock() - now_time << " ms. Starting computation of information."
         << endl;

    /* Allocate a thread to each ray. */
    now_time = clock();
    //TODO: Multi-thread it
    for (int i = 0; i < ray_num; i++) {
        ray_information_thread_process(i, rays_info, rays_map, occupancy_map, object_weight, quality_weight,
                                       octo_model, voxel_information, view_space, method);
    }
//    auto **rays_process = new std::thread *[ray_num];
//    for (int i = 0; i < ray_num; i++) {
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
//    }
//    /* Waiting for the ray calculation to be completed. */
//    for (int i = 0; i < ray_num; i++) {
//        (*rays_process[i]).join();
//    }
    auto cost_time = clock() - now_time;
    cout << "All rays' threads over with executed time " << cost_time << " ms." << endl;
    Share_Data::access_directory(share_data->save_path + "/run_time");
    std::ofstream fout(share_data->save_path + "/run_time/IG" + std::to_string(view_space->id) + ".txt");
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
}
