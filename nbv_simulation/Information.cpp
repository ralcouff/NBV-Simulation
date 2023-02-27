#include "Information.h"

void information_gain_thread_process(Ray_Information **rays_info,
                                     unordered_map<int, vector<int>> *views_to_rays_map,
                                     View_Space *view_space,
                                     int pos) {
    // Information about each relevant ray of the viewpoint is added to the viewpoint
    for (auto it = (*views_to_rays_map)[pos].begin(); it != (*views_to_rays_map)[pos].end(); it++) {
        view_space->views[pos].information_gain += rays_info[*it]->information_gain;
        view_space->views[pos].voxel_num += rays_info[*it]->voxel_num;
    }
}

void ray_cast_thread_process(int *ray_num,
                             Ray_Information **rays_info,
                             unordered_map<Ray, int, Ray_Hash> *rays_map,
                             unordered_map<int, vector<int>> *views_to_rays_map,
                             unordered_map<int, vector<int>> *rays_to_views_map,
                             octomap::ColorOcTree *octo_model,
                             Voxel_Information *voxel_information,
                             View_Space *view_space,
                             rs2_intrinsics *color_intrinsics,
                             int pos) {
    // Get a viewpoint pose
    view_space->views[pos].get_next_camera_pos(view_space->now_camera_pose_world, view_space->object_center_world);
    Eigen::Matrix4d view_pose_world =
            (view_space->now_camera_pose_world * view_space->views[pos].pose.inverse()).eval();
    // Projection of the 3D object BBX onto the convex pack area of the picture according to the viewpoint pose
    double skip_coefficient = voxel_information->skip_coefficient;
    // Control the ray traversal according to the accessible voxels, noting the interval jump parameter
    int pixel_interval = color_intrinsics->width;
    double max_range = 6.0 * view_space->predicted_size;
    vector<cv::Point2f> hull;
    hull = get_convex_on_image(voxel_information->convex,
                               view_pose_world,
                               *color_intrinsics,
                               pixel_interval,
                               max_range,
                               voxel_information->octomap_resolution);
    // if (hull.size() != 4 && hull.size() != 5 && hull.size() != 6) cout << "hull wrong with size " << hull.size() <<
    // endl; Calculating the enclosing box of a convex pack
    vector<int> boundary;
    boundary = get_xmax_xmin_ymax_ymin_in_hull(hull, *color_intrinsics);
    int xmax = boundary[0];
    int xmin = boundary[1];
    int ymax = boundary[2];
    int ymin = boundary[3];
    // cout << xmax << " " << xmin << " " << ymax << " " << ymin << " ," << pixel_interval <<endl;
    // Intermediate data structures
    vector<Ray *> rays;
    // int num = 0;
    // Check the key of the viewpoint
    octomap::OcTreeKey key_origin;
    bool key_origin_have = octo_model->coordToKeyChecked(view_space->views[pos].init_pos(0),
                                                         view_space->views[pos].init_pos(1),
                                                         view_space->views[pos].init_pos(2),
                                                         key_origin);
    if (key_origin_have) {
        octomap::point3d origin = octo_model->keyToCoord(key_origin);
        // Traversing the wrap-around box
        // srand(pos);
        // int rr = rand() % 256, gg = rand() % 256, bb = rand() % 256;
        for (int x = xmin; x <= xmax; x += (int) (pixel_interval * skip_coefficient))
            for (int y = ymin; y <= ymax; y += (int) (pixel_interval * skip_coefficient)) {
                // num++;
                cv::Point2f pixel(x, y);
                // Check if it is inside the convex bale area
                if (!is_pixel_in_convex(hull, pixel))
                    continue;
                // Reverse projection to find the end point
                octomap::point3d end = project_pixel_to_ray_end(x, y, *color_intrinsics, view_pose_world, max_range);
                // Show it
                // view_space->viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(origin(0), origin(1), origin(2)),
                // pcl::PointXYZ(end(0), end(1), end(2)), rr, gg, bb, "line" + to_string(pos) + "-" + to_string(x) + "-"
                // + to_string(y));
                octomap::OcTreeKey key_end;
                octomap::point3d direction = end - origin;
                octomap::point3d end_point;
                // Crossing unknown areas and finding the end
                bool found_end_point = octo_model->castRay(origin, direction, end_point, true, max_range);
                if (!found_end_point) {
                    // End point not found, set end point as maximum distance
                    end_point =
                            origin +
                            direction.normalized() *
                            max_range; // use max range instead of stopping at the unknown       found_endpoint = true;
                }
                // Check that the end is within the map limits and hits BBX
                bool key_end_have = octo_model->coordToKeyChecked(end_point, key_end);
                if (key_end_have) {
                    // Generation of rays
                    auto *ray_set = new octomap::KeyRay();
                    // Get the ray array, without the end node
                    bool point_on_ray_getted = octo_model->computeRayKeys(origin, end_point, *ray_set);
                    if (!point_on_ray_getted)
                        cout << "Warning. ray cast with wrong max_range." << endl;
                    if (ray_set->size() > 950)
                        cout << ray_set->size() << " rewrite the vector size in Octreekey.h." << endl;
                    // Putting the end point into the ray group
                    ray_set->addKey(key_end);
                    // The first non-empty node is used as the start of the ray and the last non-empty element from the
                    // tail is used as the end of the ray
                    auto last = ray_set->end();
                    last--;
                    while (last != ray_set->begin() && (octo_model->search(*last) == nullptr))
                        last--;
                    // Dichotomous first non-empty element
                    auto l = ray_set->begin();
                    auto r = last;
                    auto mid = l + (r - l) / 2;
                    while (mid != r) {
                        if (octo_model->search(*mid) != nullptr)
                            r = mid;
                        else
                            l = mid + 1;
                        mid = l + (r - l) / 2;
                    }
                    auto first = mid;
                    while (first != ray_set->end() &&
                           (octo_model->keyToCoord(*first).x() <
                            view_space->object_center_world(0) - view_space->predicted_size ||
                            octo_model->keyToCoord(*first).x() >
                            view_space->object_center_world(0) + view_space->predicted_size ||
                            octo_model->keyToCoord(*first).y() <
                            view_space->object_center_world(1) - view_space->predicted_size ||
                            octo_model->keyToCoord(*first).y() >
                            view_space->object_center_world(1) + view_space->predicted_size ||
                            octo_model->keyToCoord(*first).z() <
                            view_space->object_center_world(2) - view_space->predicted_size ||
                            octo_model->keyToCoord(*first).z() >
                            view_space->object_center_world(2) + view_space->predicted_size))
                        first++;
                    // If there are no non-empty elements, just discard the ray
                    if (last - first < 0) {
                        delete ray_set;
                        continue;
                    }
                    auto stop = last;
                    stop++;
                    // Show it
                    // while (octo_model->keyToCoord(*first).x() < view_space->object_center_world(0) -
                    // view_space->predicted_size || octo_model->keyToCoord(*first).x() >
                    // view_space->object_center_world(0) + view_space->predicted_size
                    //	|| octo_model->keyToCoord(*first).y() < view_space->object_center_world(1) -
                    //view_space->predicted_size || octo_model->keyToCoord(*first).y() >
                    //view_space->object_center_world(1) + view_space->predicted_size
                    //	|| octo_model->keyToCoord(*first).z() < min(view_space->height_of_ground,
                    //view_space->object_center_world(2) - view_space->predicted_size) ||
                    //octo_model->keyToCoord(*first).z() > view_space->object_center_world(2) +
                    //view_space->predicted_size) first++; octomap::point3d ss = octo_model->keyToCoord(*first);
                    // octomap::point3d ee = octo_model->keyToCoord(*last);
                    // view_space->viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(ss(0), ss(1), ss(2)),
                    // pcl::PointXYZ(ee(0), ee(1), ee(2)), rr, gg, bb, "line" + to_string(pos) + "-" + to_string(x) +
                    // "-" + to_string(y)); Add the ray to the set of viewpoints, the first element with the last
                    // element key+array+head+tail
                    Ray *ray = new Ray(*first, *last, ray_set, first, stop);
                    rays.push_back(ray);
                }
            }
    } else {
        cout << pos << "th view out of map.check." << endl;
    }
    // cout << "rays " << rays.size() <<" num "<<num<< endl;
    // Array of subscripts for this viewpoint ray
    vector<int> ray_ids;
    ray_ids.resize(rays.size());
    // Note that common data structures should be locked
    voxel_information->mutex_rays.lock();
    // Get the position of the current ray
    int ray_id = (*ray_num);
    for (int i = 0; i < rays.size(); i++) {
        // For these rays, hash queries to see if there are duplicates
        auto hash_this_ray = rays_map->find(*rays[i]);
        // If there are no duplicates, save the ray
        if (hash_this_ray == rays_map->end()) {
            (*rays_map)[*rays[i]] = ray_id;
            ray_ids[i] = ray_id;
            // Creation ray calculation class
            rays_info[ray_id] = new Ray_Information(rays[i]);
            vector<int> view_ids;
            view_ids.push_back(pos);
            (*rays_to_views_map)[ray_id] = view_ids;
            ray_id++;
        }
            // If there is a duplicate, it means that other viewpoints are also counted to that ray, so put the
            // corresponding id into the subscript array
        else {
            ray_ids[i] = hash_this_ray->second;
            delete rays[i]->ray_set;
            // Rays already recorded in other viewpoints, put in the record of this viewpoint
            vector<int> view_ids = (*rays_to_views_map)[ray_ids[i]];
            view_ids.push_back(pos);
            (*rays_to_views_map)[ray_ids[i]] = view_ids;
        }
    }
    // Updating the number of rays
    (*ray_num) = ray_id;
    // Update the ray array for the viewpoint mapping
    (*views_to_rays_map)[pos] = ray_ids;
    // Release the lock
    voxel_information->mutex_rays.unlock();
}

void ray_information_thread_process(
        int ray_id,
        Ray_Information **rays_info,
        unordered_map<Ray, int, Ray_Hash> *rays_map,
        unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *occupancy_map,
        unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *object_weight,
        unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *quality_weight,
        octomap::ColorOcTree *octo_model,
        Voxel_Information *voxel_information,
        View_Space *view_space,
        short method) {
    // Since it is checked, the first node is a non-empty node
    auto first = rays_info[ray_id]->ray->start;
    auto last = rays_info[ray_id]->ray->stop;
    last--;
    for (auto it = rays_info[ray_id]->ray->start; it != rays_info[ray_id]->ray->stop; ++it) {
        // Look up the key from the hash table
        auto hash_this_key = (*occupancy_map).find(*it);
        // Next if you can't find a node
        if (hash_this_key == (*occupancy_map).end()) {
            if (method == RSE && it == last)
                rays_info[ray_id]->information_gain = 0;
            continue;
        }
        // Read node probability values
        double occupancy = hash_this_key->second;
        /* Look up the key from the hash table */
        auto hash_this_key_quality = (*quality_weight).find(*it);
        /* Recover the quality of the node */
        double quality = hash_this_key_quality->second;
//        if (quality >= 0.5) {
//            cout << "Quality : " << quality << endl;
//        }
        // Check to see if the current node is occupied
        bool voxel_occupied = voxel_information->is_occupied(occupancy);
        // Check to see if the node is unknown
        bool voxel_unknown = voxel_information->is_unknown(occupancy);
        // Read the node for the surface rate of the object
        double on_object = Voxel_Information::voxel_object(*it, object_weight);
        // If it is occupied, it is the last node
        if (voxel_occupied)
            last = it;
        // If free, the initial node is to be updated
        if (it == first && (!voxel_unknown && !voxel_occupied))
            first = it;
        // Determine if it is the last node
        bool is_end = (it == last);
        // Statistical information entropy
        rays_info[ray_id]->information_gain = information_function(method,
                                                                   rays_info[ray_id]->information_gain,
                                                                   Voxel_Information::entropy(occupancy),
                                                                   quality,
                                                                   rays_info[ray_id]->visible,
                                                                   voxel_unknown,
                                                                   rays_info[ray_id]->previous_voxel_unknown,
                                                                   is_end,
                                                                   voxel_occupied,
                                                                   on_object,
                                                                   rays_info[ray_id]->object_visible);
        rays_info[ray_id]->object_visible *= (1 - on_object);
        if (method == OursIG or method == MyIG or method == Test_one or method == Test_two or method == Test_three or
            method == Test_four or method == Test_five or method == Test_six or method == Test_seven or
            method == Test_eight or method == Test_nine)
            rays_info[ray_id]->visible *= voxel_information->get_voxel_visible(occupancy);
        else
            rays_info[ray_id]->visible *= occupancy;
        rays_info[ray_id]->voxel_num++;
        // Exit if it's the end
        if (is_end)
            break;
    }
    while (last - first < -1)
        first--;
    last++;
    // Update stop to one iterator after the last node
    rays_info[ray_id]->ray->stop = last;
    // Update start to the first iterator
    rays_info[ray_id]->ray->start = first;
}

inline double information_function(short &method,
                                   double &ray_information,
                                   double voxel_information,
                                   double voxel_quality,
                                   double &visible,
                                   bool &is_unknown,
                                   bool &previous_voxel_unknown,
                                   bool &is_endpoint,
                                   bool &is_occupied,
                                   double &object,
                                   double &object_visible) {
    double final_information = 0;
    switch (method) {
        case Test_one:
            final_information = (1 - voxel_quality);
            break;
        case Test_two:
            final_information = 0;
            break;
        case Test_three:
            final_information = (1 - voxel_quality);
            break;
        case Test_four:
            if (is_unknown) {
                final_information = ray_information + object * visible * (1 - voxel_quality);
            } else {
                final_information = ray_information;
            }
            break;
        case Test_five:
            final_information = 0;
            break;
        case Test_six:
            if (is_unknown) {
                final_information = ray_information + object * visible * (1 - voxel_quality);
            } else {
                final_information = ray_information;
            }
            break;
        case Test_seven:
            if (is_unknown) {
                final_information = ray_information + object * visible;
            } else {
                final_information = ray_information;
            }
            break;
        case Test_eight:
            final_information = 0;
            break;
        case Test_nine:
            if (is_unknown) {
                final_information = ray_information + object * visible * voxel_information * (1-voxel_quality);
            } else {
                final_information = ray_information;
            }
            break;
        case OursIG:
            if (is_unknown) {
                final_information = ray_information + object * visible * voxel_information;
            } else {
                final_information = ray_information;
            }
            break;
        case MyIG:
            if (is_unknown) {
                final_information = ray_information + object * visible * voxel_information * (1 - voxel_quality);
            } else {
                final_information = ray_information;
            }
            break;
        case OA:
            final_information = ray_information + visible * voxel_information;
            break;
        case UV:
            if (is_unknown)
                final_information = ray_information + visible * voxel_information;
            else
                final_information = ray_information;
            break;
        case RSE:
            if (is_endpoint) {
                if (previous_voxel_unknown) {
                    if (is_occupied)
                        final_information = ray_information + visible * voxel_information;
                    else
                        final_information = 0;
                } else
                    final_information = 0;
            } else {
                if (is_unknown) {
                    previous_voxel_unknown = true;
                    final_information = ray_information + visible * voxel_information;
                } else {
                    previous_voxel_unknown = false;
                    final_information = 0;
                }
            }
            break;
        case APORA:
            if (is_unknown) {
                final_information = ray_information + object * object_visible * voxel_information;
            } else {
                final_information = ray_information;
            }
            break;
        case Kr:
            if (is_endpoint) {
                if (is_occupied)
                    final_information = ray_information + voxel_information;
                else
                    final_information = 0;
            } else
                final_information = ray_information + voxel_information;
            break;
        default:
            cout << "There might be an error in the computation of final_information" << endl;
    }
    return final_information;
}

int frontier_check(octomap::point3d node,
                   octomap::ColorOcTree *octo_model,
                   Voxel_Information *voxel_information,
                   double octomap_resolution) {
    int free_cnt = 0;
    int occupied_cnt = 0;
    for (int i = -1; i <= 1; i++)
        for (int j = -1; j <= 1; j++)
            for (int k = -1; k <= 1; k++) {
                if (i == 0 && j == 0 && k == 0)
                    continue;
                double x = node.x() + i * octomap_resolution;
                double y = node.y() + j * octomap_resolution;
                double z = node.z() + k * octomap_resolution;
                octomap::point3d neighbour(x, y, z);
                octomap::OcTreeKey neighbour_key;
                bool neighbour_key_have = octo_model->coordToKeyChecked(neighbour, neighbour_key);
                if (neighbour_key_have) {
                    octomap::ColorOcTreeNode *neighbour_voxel = octo_model->search(neighbour_key);
                    if (neighbour_voxel != nullptr) {
                        free_cnt += voxel_information->voxel_free(neighbour_voxel) ? 1 : 0;
                        occupied_cnt += voxel_information->voxel_occupied(neighbour_voxel) ? 1 : 0;
                    }
                }
            }
    // edge
    if (free_cnt >= 1 && occupied_cnt >= 1)
        return 2;
    // Boundaries
    if (free_cnt >= 1)
        return 1;
    // Nothing
    return 0;
}
