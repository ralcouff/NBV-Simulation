#include "View_Space.h"

View_Space::View_Space(int _id,
                       Share_Data *_share_data,
                       Voxel_Information *_voxel_information,
                       const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    id = _id;
    share_data = _share_data;
    voxel_information = _voxel_information;

    num_of_views = share_data->num_of_views;
    current_camera_pose_world = share_data->current_camera_pose_world;
    octomap_resolution = share_data->octomap_resolution;
    octo_model = share_data->octo_model;

    views_key_set = new unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash>();

    /* Read or generate the View Space */
    ifstream fin(share_data->view_space_file_path);
    if (fin.is_open()) {
        /* The view space has already been generated */
        // Reading the View Space from file
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

            views.push_back(view);
            views_key_set->insert(octo_model->coordToKey(init_pos[0], init_pos[1], init_pos[2]));
        }
        cout << "View Space acquired" << endl;
    } else {
        /* The View Space file hasn't been generated, we generate it. */
        // Generating a vector containing the points of the input cloud
        vector<Eigen::Vector3d> points;
        for (auto &ptr: cloud->points) {
            Eigen::Vector3d pt(ptr.x, ptr.y, ptr.z);
            points.push_back(pt);
        }
        get_view_space(points);
        save_view_space();
    }
    cout << "View Space initialized" << endl;

    /* Updating the Share Data object */
    share_data->object_center_world = object_center_world;
    share_data->bbox_radius = predicted_size;
    share_data->init_entropy = 0;
    share_data->voxels_in_BBX = 0;
    double map_size = predicted_size + 3.0 * octomap_resolution;
    share_data->map_size = map_size;

    /* Filling the octomap */
    /* Filling the octomap with unknown nodes based on the BBOX of the model. */
//    octo_model->setProbHit(0.9);
//    octo_model->setProbMiss(0.01);
    auto now_time = clock();
    for (double x = object_center_world(0) - predicted_size; x <= object_center_world(0) + predicted_size;
         x += octomap_resolution) {
        for (double y = object_center_world(1) - predicted_size; y <= object_center_world(1) + predicted_size;
             y += octomap_resolution) {
            for (double z = object_center_world(2) - predicted_size; z <= object_center_world(2) + predicted_size;
                 z += octomap_resolution) {
                octo_model->setNodeValue(x, y, z, (float) -2, true);
            }
        }
    }

    /* Inserting the input point cloud inside the octomap. */
    // First
//    const octomap::point3d sensor_origin(0, 0, 0);
//    octomap::Pointcloud cloud_octo;
//    auto po = share_data->working_cloud->begin();
//    for (int i = 0; i < share_data->input_cloud->points.size(); i++, po++) {
//        cloud_octo.push_back((*po).x, (*po).y, (*po).z);
//    }
//    share_data->octo_model->insertPointCloud(cloud_octo, sensor_origin, -1, false, false);
//    share_data->octo_model->insertPointCloud(cloud_octo, sensor_origin, -1, false, false);
//    share_data->octo_model->insertPointCloud(cloud_octo, sensor_origin, -1, false, false);

    auto ptr = share_data->working_cloud->begin();
    for (int i = 0; i < share_data->working_cloud->points.size(); i++, ptr++) {
        octomap::OcTreeKey key;
        bool key_have =
                share_data->octo_model->coordToKeyChecked(octomap::point3d((*ptr).x, (*ptr).y, (*ptr).z), key);
        if (key_have) {
            octomap::ColorOcTreeNode *voxel = share_data->octo_model->search(key);
            if (voxel == nullptr) {
                share_data->octo_model->setNodeValue(
                        key, share_data->octo_model->getProbHitLog(), true);
            } else {
                share_data->octo_model->updateNode(key, true, false);
            }
        }
    }
    octo_model->updateInnerOccupancy();

    /* Computing and updating the entropy of the model */
    for (octomap::ColorOcTree::leaf_iterator it = octo_model->begin_leafs(), end = octo_model->end_leafs();
         it != end;
         ++it) {
        double occupancy = (*it).getOccupancy();
        share_data->init_entropy += Voxel_Information::entropy(occupancy);
        share_data->voxels_in_BBX++;
    }

    /* Initialize the list of mutex for each voxel */
    voxel_information->init_mutex_voxels((int) share_data->voxels_in_BBX);

    cout << "Map_init has " << share_data->voxels_in_BBX << " voxels (in BBX), and "
         << share_data->init_entropy << " entropy" << endl;
    Share_Data::access_directory(share_data->save_path + "/quantitative");
    ofstream fout(share_data->save_path + "/quantitative/Map" + to_string(-1) + ".txt");
    fout << 0 << '\t' << share_data->init_entropy << '\t' << 0 << '\t' << 1 << endl;
}

void View_Space::get_view_space(vector<Eigen::Vector3d> &points) {
    auto now_time = clock();
    /* Pre-computing the center of mass of the object and its BBOX. */
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
    cout << "Computed object BBOX within percentage: " << percent << " ; with executed time: " << clock() - now_time
         << "ms" << endl;
    cout << "Object position is: (" << object_center_world(0) << ", " << object_center_world(1) << ", "
         << object_center_world(2) << ") And size is " << predicted_size << endl;

    /* Generating a set of random views */
    int sample_num = 0; // The number of samples made to achieve the correct number of views.
    int view_num = 0; // The view number
    // The first point of view is fixed to the centre of the model
    View init_view(Eigen::Vector3d(object_center_world(0) - predicted_size * 2.5, 0, 0));
    if (!valid_view(init_view)) {
        cout << "There might be a problem with the initial view" << endl;
    }
    views.push_back(init_view);
    views_key_set->insert(octo_model->coordToKey(init_view.init_pos(0), init_view.init_pos(1), init_view.init_pos(2)));
    view_num++;
    //TODO: Add views from the previous reconstruction
    // view_num = read_sfm_views(view_num);
    while (view_num != num_of_views) {
        double x = get_random_coordinate(object_center_world(0) - predicted_size * 4,
                                         object_center_world(0) + predicted_size * 4);
        double y = get_random_coordinate(object_center_world(1) - predicted_size * 4,
                                         object_center_world(1) + predicted_size * 4);
        double z = get_random_coordinate(object_center_world(2) - predicted_size * 4,
                                         object_center_world(2) + predicted_size * 4);
        View view(Eigen::Vector3d(x, y, z));
        view.id = view_num;

        if (valid_view(view)) {
            view.space_id = id;
//            view.dis_to_object = (object_center_world - view.init_pos).norm();
//            view.robot_cost = (Eigen::Vector3d(current_camera_pose_world(0, 3), current_camera_pose_world(1, 3), current_camera_pose_world(2, 3)).eval() - view.init_pos).norm();
            views.push_back(view);
            views_key_set->insert(octo_model->coordToKey(x, y, z));
            view_num++;
        }
        sample_num++;
        if (sample_num >= 10 * num_of_views) {
            cout << "lack of space to get view. error." << endl;
            break;
        }
    }
    cout << "There are " << views_key_set->size() << " views in View Set" << endl;
    cout << views.size() << " views got with " << sample_num << " samples" << endl;
    cout << "View Space got from octomap in " << clock() - now_time << " ms." << endl;
}

double View_Space::check_size(double _predicted_size, vector<Eigen::Vector3d> &points) {
    int valid_points = 0;
    for (auto &ptr: points) {
        if (ptr(0) < object_center_world(0) - _predicted_size || ptr(0) > object_center_world(0) + _predicted_size)
            continue;
        if (ptr(1) < object_center_world(1) - _predicted_size || ptr(1) > object_center_world(1) + _predicted_size)
            continue;
        if (ptr(2) < object_center_world(2) - _predicted_size || ptr(2) > object_center_world(2) + _predicted_size)
            continue;
        valid_points++;
    }
    return (double) valid_points / (double) points.size();
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

void View_Space::save_view_space() {
    Share_Data::access_directory(share_data->object_file_path);
    ofstream fout(share_data->view_space_file_path);
    fout << num_of_views << '\n';
    fout << object_center_world(0) << ' ' << object_center_world(1) << ' ' << object_center_world(2) << '\n';
    fout << predicted_size << '\n';
    for (int i = 0; i < num_of_views; i++)
        fout << views[i].init_pos(0) << ' ' << views[i].init_pos(1) << ' ' << views[i].init_pos(2) << '\n';
    cout << "View Space saved in: " << share_data->object_file_path << endl;
}