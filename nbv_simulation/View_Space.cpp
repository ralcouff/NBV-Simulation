#include "View_Space.h"

View_Space::View_Space(int _id,
                       Share_Data *_share_data,
                       Voxel_Information *_voxel_information,
                       const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    share_data = _share_data;
    object_changed = false;
    id = _id;
    num_of_views = share_data->num_of_views;
    now_camera_pose_world = share_data->now_camera_pose_world;
    octo_model = share_data->octo_model;
    octomap_resolution = share_data->octomap_resolution;
    voxel_information = _voxel_information;
    viewer = share_data->viewer;
    views_key_set = new unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash>();
    /* Check if the View Space has already been generated */
    ifstream fin(share_data->pcd_file_path + share_data->name_of_pcd + ".txt");
    /* If it has been generated, we read the View Space file*/
    if (fin.is_open()) {
        /* Presence of a collection of read-on viewpoints for documents. */
        int num;
        fin >> num;
        if (num != num_of_views)
            cout << "View Space read error. Check input View Space size." << endl;
        double object_center[3];
        fin >> object_center[0] >> object_center[1] >> object_center[2];
        object_center_world(0) = object_center[0];
        object_center_world(1) = object_center[1];
        object_center_world(2) = object_center[2];
        fin >> predicted_size;
        for (int i = 0; i < num_of_views; i++) {
            double init_pos[3];
            fin >> init_pos[0] >> init_pos[1] >> init_pos[2];
            View view(Eigen::Vector3d(init_pos[0], init_pos[1], init_pos[2]));
            view.id = i;
            view.space_id = id;
            view.dis_to_obejct = (object_center_world - view.init_pos).norm();
            view.robot_cost =
                    (Eigen::Vector3d(
                            now_camera_pose_world(0, 3), now_camera_pose_world(1, 3), now_camera_pose_world(2, 3))
                             .eval() - view.init_pos).norm();
            views.push_back(view);
            views_key_set->insert(octo_model->coordToKey(init_pos[0], init_pos[1], init_pos[2]));
        }
        cout << "View Space read." << endl;
    } else {
        /* The View Space file hasn't been generated, we generate it. */
        /* Get Point Cloud BBOX. */
        /* Generate a point vector containing the points of the input cloud. */
        vector<Eigen::Vector3d> points;
        for (auto &ptr: cloud->points) {
            Eigen::Vector3d pt(ptr.x, ptr.y, ptr.z);
            points.push_back(pt);
        }
        /* Viewpoint Generator. */
        get_view_space(points);
        /* Writing in file the generated View Space. */
        Share_Data::access_directory(share_data->pcd_file_path);
        ofstream fout(share_data->pcd_file_path + share_data->name_of_pcd + ".txt");
        fout << num_of_views << '\n';
        fout << object_center_world(0) << ' ' << object_center_world(1) << ' ' << object_center_world(2) << '\n';
        fout << predicted_size << '\n';
        for (int i = 0; i < num_of_views; i++)
            fout << views[i].init_pos(0) << ' ' << views[i].init_pos(1) << ' ' << views[i].init_pos(2) << '\n';
        cout << "View Space acquired." << endl;
    }
    // Update the data area data
    share_data->object_center_world = object_center_world;
    share_data->predicted_size = predicted_size;
    double map_size = predicted_size + 3.0 * octomap_resolution;
    share_data->map_size = map_size;
    /* Filling the octo_model with empty nodes based on the BBOX of the model. */
    auto now_time = clock();
    for (double x = object_center_world(0) - predicted_size; x <= object_center_world(0) + predicted_size;
         x += octomap_resolution)
        for (double y = object_center_world(1) - predicted_size; y <= object_center_world(1) + predicted_size;
             y += octomap_resolution)
            for (double z = object_center_world(2) - predicted_size; z <= object_center_world(2) + predicted_size;
                 z += octomap_resolution)
                octo_model->setNodeValue(x, y, z, (float) 0, true);
    octo_model->updateInnerOccupancy();
    share_data->init_entropy = 0;
    share_data->voxels_in_BBX = 0;
    /* Update the entropy of the model. */
    for (octomap::ColorOcTree::leaf_iterator it = octo_model->begin_leafs(), end = octo_model->end_leafs();
         it != end;
         ++it) {
        double occupancy = (*it).getOccupancy();
        share_data->init_entropy += Voxel_Information::entropy(occupancy);
        share_data->voxels_in_BBX++;
    }
    voxel_information->init_mutex_voxels(share_data->voxels_in_BBX);
    cout << "Map_init has " << share_data->voxels_in_BBX << " voxels(in BBX), and "
         << share_data->init_entropy << " entropy" << endl;
    Share_Data::access_directory(share_data->save_path + "/quantitative");
    ofstream fout(share_data->save_path + "/quantitative/Map" + to_string(-1) + ".txt");
    fout << 0 << '\t' << share_data->init_entropy << '\t' << 0 << '\t' << 1 << endl;
}

bool View_Space::valid_view(View &view) {
    double x = view.init_pos(0);
    double y = view.init_pos(1);
    double z = view.init_pos(2);
    bool valid = true;
    // Object bbx does not create a point of view within 2 times expansion
    if (x > object_center_world(0) - 2 * predicted_size && x < object_center_world(0) + 2 * predicted_size &&
        y > object_center_world(1) - 2 * predicted_size && y < object_center_world(1) + 2 * predicted_size &&
        z > object_center_world(2) - 2 * predicted_size && z < object_center_world(2) + 2 * predicted_size)
        valid = false;
    // In a ball of radius 4 times the size of BBX
    if (pow2(x - object_center_world(0)) + pow2(y - object_center_world(1)) + pow2(z - object_center_world(2)) -
        pow2(4 * predicted_size) >
        0)
        valid = false;
    // Exists in the octree index and not in the hash table
    octomap::OcTreeKey key;
    bool key_have = octo_model->coordToKeyChecked(x, y, z, key);
    if (!key_have)
        valid = false;
    if (key_have && views_key_set->find(key) != views_key_set->end())
        valid = false;
    return valid;
}

double View_Space::check_size(double predicted_size, vector<Eigen::Vector3d> &points) {
    int valid_points = 0;
    for (auto &ptr: points) {
        if (ptr(0) < object_center_world(0) - predicted_size || ptr(0) > object_center_world(0) + predicted_size)
            continue;
        if (ptr(1) < object_center_world(1) - predicted_size || ptr(1) > object_center_world(1) + predicted_size)
            continue;
        if (ptr(2) < object_center_world(2) - predicted_size || ptr(2) > object_center_world(2) + predicted_size)
            continue;
        valid_points++;
    }
    return (double) valid_points / (double) points.size();
}

void View_Space::get_view_space(vector<Eigen::Vector3d> &points) {
    auto now_time = clock();
    /* FIXME Redundant with NBV_Planner:40 */
    object_center_world = Eigen::Vector3d(0, 0, 0);
    // Calculating point cloud center of mass
    for (auto &ptr: points) {
        object_center_world(0) += ptr(0);
        object_center_world(1) += ptr(1);
        object_center_world(2) += ptr(2);
    }
    object_center_world(0) /= (double) points.size();
    object_center_world(1) /= (double) points.size();
    object_center_world(2) /= (double) points.size();
    // Dichotomous search of the BBX radius, terminated by the ratio of the number of points in the BBX reaching
    // 0.90 - 0.95
    double l = 0, r = 0, mid;
    for (auto &ptr: points) {
        r = max(r, (object_center_world - ptr).norm());
    }
    mid = (l + r) / 2;
    double percent = check_size(mid, points);
    double pre_percent = percent;
    while (true) {
        if (percent > 0.95) {
            r = mid;
        } else if (percent < 1.0) {
            l = mid;
        }
        mid = (l + r) / 2;
        percent = check_size(mid, points);
        if (fabs(pre_percent - percent) < 0.001)
            break;
        pre_percent = percent;
    }
    predicted_size = 1.2 * mid;
    cout << "object's bbx solved within percentage " << percent << " with executed time " << clock() - now_time
         << " ms." << endl;
    cout << "object's pos is (" << object_center_world(0) << "," << object_center_world(1) << ","
         << object_center_world(2) << ") and size is " << predicted_size << endl;
    int sample_num = 0;
    int viewnum = 0;
    // The first point of view is fixed to the centre of the model
    View view(Eigen::Vector3d(object_center_world(0) - predicted_size * 2.5, 0, 0));
    if (!valid_view(view))
        cout << "check init view." << endl;
    views.push_back(view);
    views_key_set->insert(octo_model->coordToKey(view.init_pos(0), view.init_pos(1), view.init_pos(2)));
    viewnum++;
    while (viewnum != num_of_views) {
        // 3x BBX for one sample area
        double x = get_random_coordinate(object_center_world(0) - predicted_size * 4,
                                         object_center_world(0) + predicted_size * 4);
        double y = get_random_coordinate(object_center_world(1) - predicted_size * 4,
                                         object_center_world(1) + predicted_size * 4);
        double z = get_random_coordinate(object_center_world(2) - predicted_size * 4,
                                         object_center_world(2) + predicted_size * 4);
        View view(Eigen::Vector3d(x, y, z));
        view.id = viewnum;
        // cout << x<<" " << y << " " << z << endl;
        // Eligible viewpoint reservations
        if (valid_view(view)) {
            view.space_id = id;
            view.dis_to_obejct = (object_center_world - view.init_pos).norm();
            view.robot_cost =
                    (Eigen::Vector3d(
                            now_camera_pose_world(0, 3), now_camera_pose_world(1, 3), now_camera_pose_world(2, 3))
                             .eval() -
                     view.init_pos)
                            .norm();
            views.push_back(view);
            views_key_set->insert(octo_model->coordToKey(x, y, z));
            viewnum++;
        }
        sample_num++;
        if (sample_num >= 10 * num_of_views) {
            cout << "lack of space to get view. error." << endl;
            break;
        }
    }
    cout << "view set is " << views_key_set->size() << endl;
    cout << views.size() << " views getted with sample_times " << sample_num << endl;
    cout << "view_space getted form octomap with executed time " << clock() - now_time << " ms." << endl;
}

void View_Space::update(int _id,
                        Share_Data *_share_data,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr update_cloud) {
    share_data = _share_data;
    object_changed = false;
    id = _id;
    now_camera_pose_world = share_data->now_camera_pose_world;
    // Update viewpoint markers
    for (int i = 0; i < views.size(); i++) {
        views[i].space_id = id;
        views[i].robot_cost =
                (Eigen::Vector3d(now_camera_pose_world(0, 3), now_camera_pose_world(1, 3), now_camera_pose_world(2, 3))
                         .eval() -
                 views[i].init_pos)
                        .norm();
    }
    // Insert point cloud to intermediate data structure
    auto now_time = clock();
    double map_size = predicted_size + 3.0 * octomap_resolution;
    share_data->map_size = map_size;
    octomap::Pointcloud cloud_octo;
    for (auto p: update_cloud->points) {
        cloud_octo.push_back(p.x, p.y, p.z);
    }
    octo_model->insertPointCloud(
            cloud_octo,
            octomap::point3d(now_camera_pose_world(0, 3), now_camera_pose_world(1, 3), now_camera_pose_world(2, 3)),
            -1,
            true,
            false);
    for (auto p: update_cloud->points) {
        octo_model->integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);
    }
    octo_model->updateInnerOccupancy();
    cout << "Octomap updated via cloud with executed time " << clock() - now_time << " ms." << endl;
    // On the map, statistical information entropy
    map_entropy = 0;
    occupied_voxels = 0;
    for (octomap::ColorOcTree::leaf_iterator it = octo_model->begin_leafs(), end = octo_model->end_leafs();
         it != end;
         ++it) {
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        if (x >= object_center_world(0) - predicted_size && x <= object_center_world(0) + predicted_size &&
            y >= object_center_world(1) - predicted_size && y <= object_center_world(1) + predicted_size &&
            z >= object_center_world(2) - predicted_size && z <= object_center_world(2) + predicted_size) {
            double occupancy = (*it).getOccupancy();
            map_entropy += voxel_information->entropy(occupancy);
            if (voxel_information->is_occupied(occupancy))
                occupied_voxels++;
        }
    }
    /*//On the point cloud, count the number of reconstructed voxels
    share_data->cloud_model->insertPointCloud(cloud_octo, octomap::point3d(now_camera_pose_world(0, 3),
    now_camera_pose_world(1, 3), now_camera_pose_world(2, 3)), -1, true, false); for (auto p : update_cloud->points)
    { share_data->cloud_model->updateNode(p.x, p.y, p.z, true, true);
            share_data->cloud_model->integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);
    }
    share_data->cloud_model->updateInnerOccupancy();
    occupied_voxels = 0;
    for (octomap::ColorOcTree::leaf_iterator it = share_data->cloud_model->begin_leafs(), end =
    share_data->cloud_model->end_leafs(); it != end; ++it){ double x = it.getX(); double y = it.getY(); double z =
    it.getZ(); if (x >= object_center_world(0) - predicted_size && x <= object_center_world(0) + predicted_size
                    && y >= object_center_world(1) - predicted_size && y <= object_center_world(1) + predicted_size
                    && z >= object_center_world(2) - predicted_size && z <= object_center_world(2) +
    predicted_size){ double occupancy = (*it).getOccupancy(); if (voxel_information->is_occupied(occupancy))
                            occupied_voxels++;
            }
    }*/
    share_data->access_directory(share_data->save_path + "/octomaps");
    share_data->octo_model->write(share_data->save_path + "/octomaps/octomap" + to_string(id) + ".ot");
    // share_data->access_directory(share_data->save_path + "/octocloud");
    // share_data->cloud_model->write(share_data->save_path + "/octocloud/octocloud"+to_string(id)+".ot");
    cout << "Map " << id << " has voxels " << occupied_voxels << ". Map " << id << " has entropy " << map_entropy
         << endl;
    cout << "Map " << id << " has voxels(rate) " << 1.0 * occupied_voxels / share_data->init_voxels << ". Map "
         << id << " has entropy(rate) " << map_entropy / share_data->init_entropy << endl;
    share_data->access_directory(share_data->save_path + "/quantitative");
    ofstream fout(share_data->save_path + "/quantitative/Map" + to_string(id) + ".txt");
    fout << occupied_voxels << '\t' << map_entropy << '\t' << 1.0 * occupied_voxels / share_data->init_voxels
         << '\t' << map_entropy / share_data->init_entropy << endl;
}

void View_Space::add_bbx_to_cloud(pcl::visualization::PCLVisualizer::Ptr viewer) {
    double x1 = object_center_world(0) - predicted_size;
    double x2 = object_center_world(0) + predicted_size;
    double y1 = object_center_world(1) - predicted_size;
    double y2 = object_center_world(1) + predicted_size;
    double z1 = object_center_world(2) - predicted_size;
    double z2 = object_center_world(2) + predicted_size;
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x1, y1, z1), pcl::PointXYZ(x1, y2, z1), 0, 255, 0, "cube1");
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x1, y1, z1), pcl::PointXYZ(x2, y1, z1), 0, 255, 0, "cube2");
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x1, y1, z1), pcl::PointXYZ(x1, y1, z2), 0, 255, 0, "cube3");
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x2, y2, z2), pcl::PointXYZ(x1, y2, z2), 0, 255, 0, "cube4");
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x2, y2, z2), pcl::PointXYZ(x2, y1, z2), 0, 255, 0, "cube5");
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x2, y2, z2), pcl::PointXYZ(x2, y2, z1), 0, 255, 0, "cube6");
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x2, y1, z2), pcl::PointXYZ(x1, y1, z2), 0, 255, 0, "cube8");
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x2, y1, z2), pcl::PointXYZ(x2, y1, z1), 0, 255, 0, "cube9");
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x1, y2, z2), pcl::PointXYZ(x1, y1, z2), 0, 255, 0, "cube10");
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x1, y2, z2), pcl::PointXYZ(x1, y2, z1), 0, 255, 0, "cube11");
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x2, y2, z1), pcl::PointXYZ(x1, y2, z1), 0, 255, 0, "cube12");
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x2, y2, z1), pcl::PointXYZ(x2, y1, z1), 0, 255, 0, "cube7");
}

void add_trajectory_to_cloud(Eigen::Matrix4d now_camera_pose_world,
                             vector<Eigen::Vector3d> &points,
                             pcl::visualization::PCLVisualizer::Ptr viewer) {
    viewer->addLine<pcl::PointXYZ>(
            pcl::PointXYZ(now_camera_pose_world(0, 3), now_camera_pose_world(1, 3), now_camera_pose_world(2, 3)),
            pcl::PointXYZ(points[0](0), points[0](1), points[0](2)),
            255,
            255,
            0,
            "trajectory" + to_string(-1));
    for (int i = 0; i < points.size() - 1; i++) {
        viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(points[i](0), points[i](1), points[i](2)),
                                       pcl::PointXYZ(points[i + 1](0), points[i + 1](1), points[i + 1](2)),
                                       255,
                                       255,
                                       0,
                                       "trajectory" + to_string(i));
    }
}

void delete_trajectory_in_cloud(int num, pcl::visualization::PCLVisualizer::Ptr viewer) {
    viewer->removeCorrespondences("trajectory" + to_string(-1));
    for (int i = 0; i < num - 1; i++) {
        viewer->removeCorrespondences("trajectory" + to_string(i));
    }
}