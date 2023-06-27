#include "NBV_Planner.h"
#include "Views_Information.h"
#include "octomap/octomap_types.h"
#include <pcl/impl/point_types.hpp>
#include <string>

NBV_Planner::NBV_Planner(Share_Data *_share_data, int _status) {
    share_data = _share_data;
    status = _status;
    iterations = 0;
    share_data->now_view_space_processed = false;
    share_data->now_views_information_processed = false;
    share_data->move_on = false;

    /* Making sure the savePath exists. */
    Share_Data::access_directory(share_data->savePath);

    /* Initializing the Voxel Information */
    voxel_information = new Voxel_Information(share_data->p_unknown_lower_bound, share_data->p_unknown_upper_bound);

    /* Initializing GT cloud */
    share_data->cloud_ground_truth->is_dense = false;
    share_data->cloud_ground_truth->points.resize(share_data->cloud_pcd->points.size());
    share_data->cloud_ground_truth->width = share_data->cloud_pcd->points.size();
    share_data->cloud_ground_truth->height = 1;

    /* Creating iterators to the points of GT_cloud (ptr) and pcd_cloud (p). */
    auto ptr = share_data->cloud_ground_truth->points.begin();
    auto p = share_data->cloud_pcd->points.begin();

    /* Re-scaling the model if it's too big. */
    float unit = 1.0;
    for (auto &point: share_data->cloud_pcd->points) {
        if (fabs(point.x) >= 10 || fabs(point.y) >= 10 || fabs(point.z) >= 10) {
            unit = 0.001;
            cout << "Changing unit from <mm> to <m>." << endl;
            break;
        }
    }
    /* Check the size of the object and scale it uniformly to about 0.15m. */
    vector<Eigen::Vector3d> points;
    for (auto &point: share_data->cloud_pcd->points) {
        Eigen::Vector3d pt(point.x * unit, point.y * unit, point.z * unit);
        points.push_back(pt);
    }

    /* Computing and initializing the center of mass of the point cloud. */
    Eigen::Vector3d object_center_world = Eigen::Vector3d(0, 0, 0);
    /* Calculating point cloud center of mass. */
    for (auto &point: points) {
        object_center_world(0) += point(0);
        object_center_world(1) += point(1);
        object_center_world(2) += point(2);
    }
    object_center_world(0) /= (double) points.size();
    object_center_world(1) /= (double) points.size();
    object_center_world(2) /= (double) points.size();

    /* Dichotomous search of the BBX radius, terminated by the ratio of the number of points in the BBX reaching
     * 0.92 and more */
    double l = 0, r = 0, mid;
    for (auto &point: points) {
        r = max(r, (object_center_world - point).norm());
    }
    mid = (l + r) / 2;
    double percent = check_size(mid, object_center_world, points);
    double pre_percent = percent;
    while (true) {
        if (percent > 0.92) {
            r = mid;
        } else if (percent < 1.0) {
            l = mid;
        }
        mid = (l + r) / 2;
        percent = check_size(mid, object_center_world, points);
        if (fabs(pre_percent - percent) < 0.001)
            break;
        pre_percent = percent;
    }
    /* If the radius of the BBOX is too large, we scale the object to about 0.1m. */
    double predicted_size = 1.2 * mid;
    float scale = 1.0;
    if (predicted_size > 0.1) {
        scale = (float) (0.1 / predicted_size);
        cout << "Object large. Change scale to about 0.1 m." << endl;
        cout << "Scale = " << scale << endl;
    }

    /* Initializing the quality sets */
    share_data->gt_quality_weight = new std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>();
    share_data->gt_sample_quality_weight = new std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>();

    /* Converting the points. */
    ifstream quality_file(share_data->qualityFilePath);
    for (int i = 0; i < share_data->cloud_pcd->points.size(); i++, p++) {
        std::string line = "1";
        (*ptr).x = (*p).x * scale * unit;
        (*ptr).y = (*p).y * scale * unit;
        (*ptr).z = (*p).z * scale * unit;
        (*ptr).b = 168;
        (*ptr).g = 168;
        (*ptr).r = 168;
        /* If we have a quality file, we color the points in the octomap that have a "bad" quality. */
        if (quality_file.is_open()) {
            if (getline(quality_file, line)) {
                if (std::stod(line) != 1) {
                    (*ptr).b = 0;
                    (*ptr).g = 0;
                    (*ptr).r = 255;
                }
            }
        }
        /* Filling the GT octree. */
        octomap::OcTreeKey key;
        bool key_have =
                share_data->ground_truth_model->coordToKeyChecked(octomap::point3d((*ptr).x, (*ptr).y, (*ptr).z), key);
        if (key_have) {
            octomap::ColorOcTreeNode *voxel = share_data->ground_truth_model->search(key);
            if (voxel == nullptr) {
                share_data->ground_truth_model->setNodeValue(
                        key, share_data->ground_truth_model->getProbHitLog(), true);
                share_data->ground_truth_model->integrateNodeColor(key, (*ptr).r, (*ptr).g, (*ptr).b);
            }
            if ((*share_data->gt_quality_weight).find(key) == (*share_data->gt_quality_weight).end())
                (*share_data->gt_quality_weight)[key] = std::stod(line);
            else
                (*share_data->gt_quality_weight)[key] = std::min((*share_data->gt_quality_weight)[key],
                                                                 std::stod(line));
        }

        /* Filling the GT_Sample octree. */
        octomap::OcTreeKey key_sp;
        bool key_have_sp =
                share_data->GT_sample->coordToKeyChecked(octomap::point3d((*ptr).x, (*ptr).y, (*ptr).z), key_sp);
        if (key_have_sp) {
            octomap::ColorOcTreeNode *voxel_sp = share_data->GT_sample->search(key_sp);
            if (voxel_sp == nullptr) {
                share_data->GT_sample->setNodeValue(key_sp, share_data->GT_sample->getProbHitLog(), true);
                share_data->GT_sample->integrateNodeColor(key_sp, (*ptr).r, (*ptr).g, (*ptr).b);
            }
            if ((*share_data->gt_sample_quality_weight).find(key_sp) == (*share_data->gt_sample_quality_weight).end())
                (*share_data->gt_sample_quality_weight)[key_sp] = std::stod(line);
            else
                (*share_data->gt_sample_quality_weight)[key_sp] = std::min(
                        (*share_data->gt_sample_quality_weight)[key_sp], std::stod(line));

        }
        ptr++;
    }
    quality_file.close();

    // Displaying the repartition of quality in the octomap
//    save_quality_map(share_data, share_data->gt_quality_weight);
    std::map<double, int> qlt_map{};
    for (octomap::ColorOcTree::leaf_iterator it = share_data->ground_truth_model->begin_leafs(), end = share_data->ground_truth_model->end_leafs();
         it != end; ++it) {
        double qltt = Voxel_Information::voxel_quality(const_cast<octomap::OcTreeKey &>(it.getKey()),
                                                       share_data->gt_quality_weight);
        qlt_map[qltt]++;
    }
    cout << "Quality in gt_quality_weight" << endl;
    for (auto &it: qlt_map) {
        std::cout << it.first << " - " << it.second << endl;
    }

    // Displaying the repartition of quality in the octomap
//    save_quality_map(share_data, share_data->gt_sample_quality_weight);
    std::map<double, int> qlt_mapp{};
    for (octomap::ColorOcTree::leaf_iterator it = share_data->GT_sample->begin_leafs(), end = share_data->GT_sample->end_leafs();
         it != end; ++it) {
        double qltt = Voxel_Information::voxel_quality(const_cast<octomap::OcTreeKey &>(it.getKey()),
                                                       share_data->gt_sample_quality_weight);
        qlt_mapp[qltt]++;
    }
    cout << "Quality in gt_sample_quality_weight" << endl;
    for (auto &it: qlt_mapp) {
        std::cout << it.first << " - " << it.second << endl;
    }

    /* Add the initial PC to the sfm_data pipeline. */
    //TODO : Re_add the Sfm pipeline and save a rescaled_model of the object
    float index = 0;
    for (auto &pt: share_data->cloud_ground_truth->points) {
        share_data->sfm_data.getLandmarks().emplace(index, aliceVision::sfmData::Landmark(
                Eigen::Matrix<double, 3, 1>(pt.x, pt.y, pt.z), aliceVision::feature::EImageDescriberType::SIFT));
        index++;
    }
    /* Convert and save a version of the rescaled model */
    save_rescaled(scale, unit, share_data);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr scaled_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
//    transform_1(0,0) = scale*unit;
//    transform_1(1,1) = -scale*unit;
//    transform_1(2,2) = -scale*unit;
//    pcl::transformPointCloud(*(share_data->cloud_pcd), *scaled_cloud,transform_1);
//    pcl::io::savePLYFile("rescaled_model.ply", *scaled_cloud);
//    pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh ());
//    pcl::io::loadPLYFile("rescaled_model.ply", *mesh);
//    pcl::io::saveOBJFile("rescaled_model.obj",*mesh);

    /* GT voxels update. */
    share_data->ground_truth_model->updateInnerOccupancy();
    share_data->ground_truth_model->write(share_data->savePath + "/GT.ot");

    /* GT_sample_voxels update. */
    share_data->GT_sample->updateInnerOccupancy();
    share_data->GT_sample->write(share_data->savePath + "/GT_sample.ot");

    /* Determine the number of voxels in the octree. */
    share_data->init_voxels = (int) share_data->GT_sample->getNumLeafNodes();
    cout << "Octree GT_sample has " << share_data->init_voxels << " voxels" << endl;
    ofstream fout(share_data->savePath + "/GT_sample_voxels.txt");
    fout << share_data->init_voxels << endl;

    /* Initialize View Space. */
    /* Generates the set of views around the Point Cloud. */
    now_view_space = new View_Space(iterations, share_data, voxel_information, share_data->cloud_ground_truth);

    // Set the initial viewpoint to a uniform position
    now_view_space->views[0].vis++;
    now_best_view = new View(now_view_space->views[0]);
    now_best_view->get_next_camera_pos(share_data->now_camera_pose_world, share_data->object_center_world);
    Eigen::Matrix4d view_pose_world = (share_data->now_camera_pose_world * now_best_view->pose.inverse()).eval();

    // Camera class initialization
    percept = new Perception_3D(share_data);

    /* Displaying the correct initialization. */
//    if (share_data->show) {
//        // Show BBX, camera position, GT
//        pcl::visualization::PCLVisualizer::Ptr visualizer =
//                std::make_shared<pcl::visualization::PCLVisualizer>("Iteration");
//        visualizer->setBackgroundColor(0, 0, 0);
//        visualizer->addCoordinateSystem(0.1);
//        visualizer->initCameraParameters();
//        /* First frame camera position. */
//        Eigen::Vector4d X(0.05, 0, 0, 1);
//        Eigen::Vector4d Y(0, 0.05, 0, 1);
//        Eigen::Vector4d Z(0, 0, 0.05, 1);
//        Eigen::Vector4d O(0, 0, 0, 1);
//        X = view_pose_world * X;
//        Y = view_pose_world * Y;
//        Z = view_pose_world * Z;
//        O = view_pose_world * O;
//        /* Drawing the camera axis. */
//        visualizer->addLine<pcl::PointXYZ>(
//                pcl::PointXYZ((float) O(0), (float) O(1), (float) O(2)),
//                pcl::PointXYZ((float) X(0), (float) X(1), (float) X(2)), 255, 0, 0, "X" + to_string(-1));
//        visualizer->addLine<pcl::PointXYZ>(
//                pcl::PointXYZ((float) O(0), (float) O(1), (float) O(2)),
//                pcl::PointXYZ((float) Y(0), (float) Y(1), (float) Y(2)), 0, 255, 0, "Y" + to_string(-1));
//        visualizer->addLine<pcl::PointXYZ>(
//                pcl::PointXYZ((float) O(0), (float) O(1), (float) O(2)),
//                pcl::PointXYZ((float) Z(0), (float) Z(1), (float) Z(2)), 0, 0, 255, "Z" + to_string(-1));
//        /* Creating a point cloud containing the potential viewpoints, and displaying it in white in the frame. */
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_view_space(new pcl::PointCloud<pcl::PointXYZRGB>);
//        test_view_space->is_dense = false;
//        test_view_space->points.resize(now_view_space->views.size());
//        auto pt = test_view_space->points.begin();
//        for (int i = 0; i < now_view_space->views.size(); i++, pt++) {
//            (*pt).x = (float) now_view_space->views[i].init_pos(0);
//            (*pt).y = (float) now_view_space->views[i].init_pos(1);
//            (*pt).z = (float) now_view_space->views[i].init_pos(2);
//            /* Setting color to white */
//            (*pt).r = 255, (*pt).g = 255, (*pt).b = 255;
//        }
//        visualizer->addPointCloud<pcl::PointXYZRGB>(test_view_space, "test_view_space");
//        now_view_space->add_bbx_to_cloud(visualizer);
//        visualizer->addPointCloud<pcl::PointXYZRGB>(share_data->cloud_ground_truth, "cloud_ground_truth");
//        visualize_octomap(visualizer, *share_data->octo_model, share_data->object_center_world,
//                          share_data->predicted_size);
//        while (!visualizer->wasStopped()) {
//            visualizer->spin();
//            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//        }
//    }
}

double NBV_Planner::check_size(double predicted_size, Eigen::Vector3d object_center_world,
                               std::vector<Eigen::Vector3d> &points) {
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

int NBV_Planner::plan() {
    switch (status) {
        case Over:
            break;
        case WaitData:
            cout << "***************************************************************************" << endl;
            cout << "The current method is : " << endl;
            cout << NBV_Planner::share_data->method_of_IG << endl;
            cout << "***************************************************************************" << endl;
            if (percept->percept(now_best_view)) {
                thread next_view_space(create_view_space, &now_view_space, now_best_view, share_data, iterations);
                next_view_space.detach();
                status = WaitViewSpace;
            }
            if (iterations == share_data->reconstructionIterations) {
                cout
                        << "------------------------------------------------------------------------------------------------"
                        << endl;
                cout << "Changing the method of information gain from : " << share_data->method_of_IG << " to "
                     << share_data->alt_method_of_IG << '.' << endl;
                cout
                        << "------------------------------------------------------------------------------------------------"
                        << endl;
                share_data->method_of_IG = share_data->alt_method_of_IG;
            }
//            compute_completeness(share_data);
            break;
        case WaitViewSpace:
            if (share_data->now_view_space_processed) {
                thread next_views_information(create_views_information,
                                              &now_views_information,
                                              now_best_view,
                                              now_view_space,
                                              share_data,
                                              this,
                                              iterations);
                next_views_information.detach();
                status = WaitInformation;
            }
            break;
        case WaitInformation:
            if (share_data->now_views_information_processed) {
                std::map<double, int> occupancy{};
                for (octomap::ColorOcTree::leaf_iterator it = share_data->octo_model->begin_leafs(), end = share_data->octo_model->end_leafs();
                     it != end; ++it) {
                    double occ = it->getOccupancy();
                    occupancy[occ]++;
                }
                cout << "Occupancy in octo_model" << endl;
                for (auto &it: occupancy) {
                    std::cout << it.first << " - " << it.second << endl;
                }
                if (share_data->method_of_IG == 6) { // NBV-NET
                    Share_Data::access_directory(share_data->nbvNetPath + "/log");
                    ifstream ftest;
                    do {
                        ftest.open(share_data->nbvNetPath + "/log/ready.txt");
                    } while (!ftest.is_open());
                    ftest.close();
                    ifstream fin(share_data->nbvNetPath + "/log/" + share_data->nameOfObject + '_' +
                                 to_string(iterations) + ".txt");
                    double x, y, z, a, b, c;
                    fin >> x >> y >> z >> a >> b >> c;
                    cout << x << " " << y << " " << z << endl;
                    now_best_view->init_pos(0) = x;
                    now_best_view->init_pos(1) = y;
                    now_best_view->init_pos(2) = z;
                    Eigen::Matrix3d rotation;
                    rotation = Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(b, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(c, Eigen::Vector3d::UnitZ());
                    Eigen::Matrix4d R(Eigen::Matrix4d::Identity(4, 4));
                    R(0, 0) = rotation(0, 0);
                    R(0, 1) = rotation(0, 1);
                    R(0, 2) = rotation(0, 2);
                    R(0, 3) = x;
                    R(1, 0) = rotation(1, 0);
                    R(1, 1) = rotation(1, 1);
                    R(1, 2) = rotation(1, 2);
                    R(1, 3) = y;
                    R(2, 0) = rotation(2, 0);
                    R(2, 1) = rotation(2, 1);
                    R(2, 2) = rotation(2, 2);
                    R(2, 3) = z;
                    R(3, 0) = 0;
                    R(3, 1) = 0;
                    R(3, 2) = 0;
                    R(3, 3) = 1;
                    now_best_view->pose = R;
                    this_thread::sleep_for(chrono::seconds(5));
                    int removed = remove((share_data->nbvNetPath + "/log/ready.txt").c_str());
                    if (removed != 0)
                        cout << "cannot remove ready.txt." << endl;
                } else {
                    // Search algorithms
                    /* Sorting the viewpoints according to their utility. */
                    sort(now_view_space->views.begin(), now_view_space->views.end(), view_utility_compare);
                    /* Save a file containing the value of the local_information of each view. */
                    std::ofstream InfoOUT(
                            share_data->savePath + '/' + "global_information.csv");
                    for (auto &view: now_view_space->views) {
                        InfoOUT << view.id << "," << view.final_utility << endl;
                    }
                    // informed_viewspace
                    if (share_data->show) {
                        /* Show BBX with camera position. */
                        pcl::visualization::PCLVisualizer::Ptr visualizer = std::make_shared<pcl::visualization::PCLVisualizer>(
                                "Iteration" + to_string(iterations));
                        visualizer->setBackgroundColor(0, 0, 0);
                        visualizer->addCoordinateSystem(0.1);
                        visualizer->initCameraParameters();
                        std::vector<pcl::visualization::Camera> cameras{};
                        visualizer->getCameras(cameras);
                        auto pos = cameras[0].pos;
                        auto view = cameras[0].view;
                        visualizer->setCameraPosition(pos[0], pos[1], pos[2] - 0.5, view[0], view[1], view[2]);
                        // test_view_space
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_view_space(
                                new pcl::PointCloud<pcl::PointXYZRGB>);
                        test_view_space->is_dense = false;
                        test_view_space->points.resize(now_view_space->views.size());
                        auto ptr = test_view_space->points.begin();
                        int needed = 0;
                        for (int i = 0; i < now_view_space->views.size(); i++) {
                            (*ptr).x = (float) now_view_space->views[i].init_pos(0);
                            (*ptr).y = (float) now_view_space->views[i].init_pos(1);
                            (*ptr).z = (float) now_view_space->views[i].init_pos(2);
                            // Visited points are recorded in blue
                            if (now_view_space->views[i].vis)
                                (*ptr).r = 0, (*ptr).g = 0, (*ptr).b = 255;
                                // The setting within the network stream is yellow
                            else if (now_view_space->views[i].in_coverage[iterations] &&
                                     i < now_view_space->views.size() / 10)
                                (*ptr).r = 255, (*ptr).g = 255, (*ptr).b = 0;
                                // The setting within the network stream is red
                            else if (now_view_space->views[i].in_coverage[iterations])
                                (*ptr).r = 255, (*ptr).g = 0, (*ptr).b = 0;
                                // The top 10% of the weighted points are set to blue-green
                            else if (i < now_view_space->views.size() / 10)
                                (*ptr).r = 0, (*ptr).g = 255, (*ptr).b = 255;
                                // Don't want the rest of the points
                            else
                                continue;
                            ptr++;
                            needed++;
                        }
                        test_view_space->points.resize(needed);
                        visualizer->addPointCloud<pcl::PointXYZRGB>(test_view_space, "test_view_space");
                        bool best_have = false;
                        for (int i = 0; i < now_view_space->views.size(); i++) {
                            if (now_view_space->views[i].vis) {
                                now_view_space->views[i].get_next_camera_pos(share_data->now_camera_pose_world,
                                                                             share_data->object_center_world);
                                Eigen::Matrix4d view_pose_world =
                                        (share_data->now_camera_pose_world * now_view_space->views[i].pose.inverse())
                                                .eval();
                                Eigen::Vector4d X(0.03, 0, 0, 1);
                                Eigen::Vector4d Y(0, 0.03, 0, 1);
                                Eigen::Vector4d Z(0, 0, 0.03, 1);
                                Eigen::Vector4d O(0, 0, 0, 1);
                                X = view_pose_world * X;
                                Y = view_pose_world * Y;
                                Z = view_pose_world * Z;
                                O = view_pose_world * O;
                                visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)),
                                                                   pcl::PointXYZ(X(0), X(1), X(2)),
                                                                   255,
                                                                   0,
                                                                   0,
                                                                   "X" + to_string(i));
                                visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)),
                                                                   pcl::PointXYZ(Y(0), Y(1), Y(2)),
                                                                   0,
                                                                   255,
                                                                   0,
                                                                   "Y" + to_string(i));
                                visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)),
                                                                   pcl::PointXYZ(Z(0), Z(1), Z(2)),
                                                                   0,
                                                                   0,
                                                                   255,
                                                                   "Z" + to_string(i));
                            } else if (!best_have) {
                                now_view_space->views[i].get_next_camera_pos(share_data->now_camera_pose_world,
                                                                             share_data->object_center_world);
                                Eigen::Matrix4d view_pose_world =
                                        (share_data->now_camera_pose_world * now_view_space->views[i].pose.inverse())
                                                .eval();
                                // cout << "VP_World : " << view_pose_world << endl;
                                Eigen::Vector4d X(0.08, 0, 0, 1);
                                Eigen::Vector4d Y(0, 0.08, 0, 1);
                                Eigen::Vector4d Z(0, 0, 0.08, 1);
                                Eigen::Vector4d O(0, 0, 0, 1);
                                X = view_pose_world * X;
                                Y = view_pose_world * Y;
                                Z = view_pose_world * Z;
                                O = view_pose_world * O;
                                // cout << "O : " << O << endl;
                                int nb_nbv = (int) share_data->sfm_data.getPoses().size() + 1;
                                // TODO : Give a significant name to the view
                                share_data->sfm_data.getViews().emplace(nb_nbv,
                                                                        std::make_shared<aliceVision::sfmData::View>("",
                                                                                                                     nb_nbv,
                                                                                                                     0,
                                                                                                                     nb_nbv,
                                                                                                                     share_data->color_intrinsics.width,
                                                                                                                     share_data->color_intrinsics.height));
                                aliceVision::geometry::Pose3 transform = aliceVision::geometry::Pose3(
                                        view_pose_world.block<3, 3>(0, 0).transpose(), O.block<3, 1>(0, 0));
                                share_data->sfm_data.getPoses().emplace(nb_nbv,
                                                                        aliceVision::sfmData::CameraPose(transform,
                                                                                                         false));
                                visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)),
                                                                   pcl::PointXYZ(X(0), X(1), X(2)),
                                                                   255,
                                                                   0,
                                                                   0,
                                                                   "X" + to_string(i));
                                visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)),
                                                                   pcl::PointXYZ(Y(0), Y(1), Y(2)),
                                                                   0,
                                                                   255,
                                                                   0,
                                                                   "Y" + to_string(i));
                                visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)),
                                                                   pcl::PointXYZ(Z(0), Z(1), Z(2)),
                                                                   0,
                                                                   0,
                                                                   255,
                                                                   "Z" + to_string(i));
                                best_have = true;
                                std::vector<int> rays_id = (*now_views_information->views_to_rays_map)[now_view_space->views[i].id];
                                for (int r_id: rays_id) {
                                    auto start = now_views_information->rays_info[r_id]->ray->origin;
                                    auto stop = now_views_information->rays_info[r_id]->ray->end;
                                    auto stop_unknown = now_views_information->rays_info[r_id]->ray->end_unknown;
                                    octomap::point3d ss = share_data->octo_model->keyToCoord(start);
                                    octomap::point3d ee = share_data->octo_model->keyToCoord(stop);
                                    octomap::point3d ee_u = share_data->octo_model->keyToCoord(stop_unknown);
                                    visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(ss(0), ss(1), ss(2)),
                                                                       pcl::PointXYZ(ee(0), ee(1), ee(2)),
                                                                       255,
                                                                       0,
                                                                       255,
                                                                       "Ray" + to_string(r_id));
                                    visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(ss(0), ss(1), ss(2)),
                                                                       pcl::PointXYZ(ee_u(0), ee_u(1), ee_u(2)),
                                                                       168,
                                                                       168,
                                                                       0,
                                                                       "Ray" + to_string(r_id) + "_unknown");

                                }
                                cout << "There are: " << rays_id.size() << " rays projected in the octomap." << endl;
                            }
                        }
                        visualize_octomap(visualizer, *share_data->octo_model, share_data->object_center_world,
                                          share_data->predicted_size);
                        visualizer->addPointCloud<pcl::PointXYZRGB>(share_data->cloud_final, "cloud_now_iteration");
                        visualizer->saveScreenshot(
                                share_data->savePath + '/' + std::to_string(iterations) + "_screenshot.png");
                        visualizer->spinOnce(1);
                        visualizer->close();
//                        while (!visualizer->wasStopped()) {
//                            visualizer->spin();
//                            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//                        }
                    }
                    double max_utility = -1;
                    for (int i = 0; i < now_view_space->views.size(); i++) {
                        cout << "checking view " << i << endl;
                        if (now_view_space->views[i].vis)
                            continue;
                        now_best_view = new View(now_view_space->views[i]);
                        max_utility = now_best_view->final_utility;
                        now_view_space->views[i].vis++;
                        share_data->best_views.push_back(*now_best_view);
                        cout << "Best_Views : " << share_data->best_views.size() << endl;
                        now_view_space->views[i].can_move = true;
                        cout << "View " << i << " has been chosen." << endl;
                        break;
                    }
                    if (max_utility == -1) {
                        cout << "Can't move to any viewpoint. Stop." << endl;
                        status = Over;
                        break;
                    }
                    cout << " Next best view position is: (" << now_best_view->init_pos(0) << ", "
                         << now_best_view->init_pos(1) << ", " << now_best_view->init_pos(2) << ")" << endl;
                    cout << " Next best view final_utility is: " << now_best_view->final_utility << endl;
                    std::ofstream result(share_data->test_base_filename, std::ios_base::app);
                    result << share_data->alt_method_of_IG << ',' << share_data->method_of_IG << ','
                           << share_data->n_model << ',' << share_data->n_size << ','
                           << share_data->reconstructionIterations << ','
                           << std::to_string(iterations) << ',' << now_best_view->id << ','
                           << now_best_view->init_pos(0) << ',' << now_best_view->init_pos(1) << ','
                           << now_best_view->init_pos(2) << ',' << now_best_view->final_utility << ','
                           << compute_completeness(share_data) << endl;
                    result.close();
                }
                thread next_moving(move_robot, now_best_view, now_view_space, share_data, this);
                next_moving.detach();
                aliceVision::sfmDataIO::saveJSON(share_data->sfm_data,
                                                 share_data->savePath + '/' + share_data->nameOfObject + ".sfm",
                                                 aliceVision::sfmDataIO::ESfMData::ALL);
                aliceVision::sfmDataIO::Save(share_data->sfm_data,
                                             share_data->savePath + '/' + share_data->nameOfObject + ".abc",
                                             aliceVision::sfmDataIO::ESfMData::ALL);
                generate_images(iterations, false, share_data);
                status = WaitMoving;
            }
            break;
        case WaitMoving:
            // virtual move
            if (share_data->over) {
                cout << "Progress over. Saving octomap and cloud." << endl;
                status = Over;
                break;
            }
            if (share_data->move_on) {
                iterations++;
                share_data->now_view_space_processed = false;
                share_data->now_views_information_processed = false;
                share_data->move_on = false;
                status = WaitData;
            }
            break;
    }
    return status;
}

string NBV_Planner::out_status() {
    string status_string;
    switch (status) {
        case Over:
            status_string = "Over";
            break;
        case WaitData:
            status_string = "WaitData";
            break;
        case WaitViewSpace:
            status_string = "WaitViewSpace";
            break;
        case WaitInformation:
            status_string = "WaitInformation";
            break;
        case WaitMoving:
            status_string = "WaitMoving";
            break;
    }
    return status_string;
}

void save_cloud_mid_thread_process(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const string &name,
                                   Share_Data *share_data) {
    /* FIXME : Check if the cloud is well saved. */
    share_data->save_cloud_to_disk(cloud, "/clouds", name);
    cout << name << " saved" << endl;
}

void create_view_space(View_Space **now_view_space, View *now_best_view, Share_Data *share_data, int iterations) {
    // Calculating keyframe camera poses
    share_data->now_camera_pose_world = (share_data->now_camera_pose_world * now_best_view->pose.inverse()).eval();;
    // Handling view space
    (*now_view_space)->update(iterations, share_data, share_data->cloud_final, share_data->clouds[iterations]);
    // Save the results of intermediate iterations
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_mid(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_mid = *share_data->cloud_final;
    thread save_mid(save_cloud_mid_thread_process, cloud_mid, "pointcloud" + to_string(iterations), share_data);
    save_mid.detach();
    // Update flag bit
    share_data->now_view_space_processed = true;
}

void create_views_information(Views_Information **now_views_information,
                              View *now_best_view,
                              View_Space *now_view_space,
                              Share_Data *share_data,
                              NBV_Planner *nbv_plan,
                              int iterations) {
    if (share_data->method_of_IG == 6) { // NBV-NET
        // scale
        Share_Data::access_directory(share_data->nbvNetPath + "/viewspace");
        ofstream fout_vs(share_data->nbvNetPath + "/viewspace/" + share_data->nameOfObject + ".txt");
        double scale_of_object = 0;
        double x1 = now_view_space->object_center_world(0) - now_view_space->predicted_size;
        double x2 = now_view_space->object_center_world(0) + now_view_space->predicted_size;
        double y1 = now_view_space->object_center_world(1) - now_view_space->predicted_size;
        double y2 = now_view_space->object_center_world(1) + now_view_space->predicted_size;
        double z1 = now_view_space->object_center_world(2) - now_view_space->predicted_size;
        double z2 = now_view_space->object_center_world(2) + now_view_space->predicted_size;
        scale_of_object = max(scale_of_object, (Eigen::Vector3d(x1, y1, z1).eval()).norm());
        scale_of_object = max(scale_of_object, (Eigen::Vector3d(x1, y2, z1).eval()).norm());
        scale_of_object = max(scale_of_object, (Eigen::Vector3d(x1, y1, z2).eval()).norm());
        scale_of_object = max(scale_of_object, (Eigen::Vector3d(x1, y2, z2).eval()).norm());
        scale_of_object = max(scale_of_object, (Eigen::Vector3d(x2, y1, z1).eval()).norm());
        scale_of_object = max(scale_of_object, (Eigen::Vector3d(x2, y2, z1).eval()).norm());
        scale_of_object = max(scale_of_object, (Eigen::Vector3d(x2, y1, z2).eval()).norm());
        scale_of_object = max(scale_of_object, (Eigen::Vector3d(x2, y2, z2).eval()).norm());
        fout_vs << scale_of_object * 2.0 << '\n';
        // octotree
        Share_Data::access_directory(share_data->nbvNetPath + "/data");
        ofstream fout(share_data->nbvNetPath + "/data/" + share_data->nameOfObject + '_' + to_string(iterations) +
                      ".txt");
        for (octomap::ColorOcTree::leaf_iterator it = share_data->octo_model->begin_leafs(),
                     end = share_data->octo_model->end_leafs();
             it != end;
             ++it) {
            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();
            if (x >= now_view_space->object_center_world(0) - now_view_space->predicted_size &&
                x <= now_view_space->object_center_world(0) + now_view_space->predicted_size &&
                y >= now_view_space->object_center_world(1) - now_view_space->predicted_size &&
                y <= now_view_space->object_center_world(1) + now_view_space->predicted_size &&
                z >= now_view_space->object_center_world(2) - now_view_space->predicted_size &&
                z <= now_view_space->object_center_world(2) + now_view_space->predicted_size) {
                double occupancy = (*it).getOccupancy();
                fout << x << ' ' << y << ' ' << z << ' ' << occupancy << '\n';
            }
        }
    } else { // Search by
        /* If it is the first iteration, creates and compute the first Views Information, else update them. */
        if (iterations == 0)
            (*now_views_information) =
                    new Views_Information(share_data, nbv_plan->voxel_information, now_view_space, iterations);
        else
            (*now_views_information)->update(share_data, now_view_space, iterations);
        if (share_data->method_of_IG == OursIG || share_data->method_of_IG == Test_e) {
            // Handling network streams and obtaining global optimisation functions
            auto *set_cover_solver = new views_voxels_MF(share_data->num_of_max_flow_node,
                                                         now_view_space,
                                                         *now_views_information,
                                                         nbv_plan->voxel_information,
                                                         share_data);
            set_cover_solver->solve();
            /* Recover a set of view for the max coverage problem. */
            vector<int> coverage_view_id_set = set_cover_solver->get_view_id_set();
            for (int i: coverage_view_id_set)
                now_view_space->views[i].in_coverage[iterations] = 1;
        }
        // Combined calculation of local greed and global optimization to produce viewpoint information entropy
        share_data->sum_local_information = 0;
        share_data->sum_global_information = 0;
        share_data->sum_robot_cost = 0;
        for (auto &view: now_view_space->views) {
            share_data->sum_local_information += view.information_gain;
            share_data->sum_global_information += view.get_global_information();
            share_data->sum_robot_cost += view.robot_cost;
        }
        if (share_data->sum_local_information == 0)
            cout << "Full information is zero." << endl;
        if (share_data->sum_global_information == 0)
            cout << "Global information is zero." << endl;
        for (auto &view: now_view_space->views) {
            if (share_data->method_of_IG == OursIG)
                view.final_utility =
                        (1 - share_data->cost_weight) * view.information_gain /
                        share_data->sum_local_information +
                        share_data->cost_weight * view.get_global_information() /
                        share_data->sum_global_information;
            else if (share_data->method_of_IG == Test_e)
                view.final_utility = (share_data->sum_local_information == 0 ? 0 : (1 - share_data->cost_weight) * view.information_gain / share_data->sum_local_information);
            else if (share_data->method_of_IG == Test_o)
                view.final_utility = (share_data->sum_global_information == 0 ? 0 : share_data->cost_weight * view.get_global_information() / share_data->sum_global_information);
            else if (share_data->method_of_IG == APORA)
                view.final_utility = view.information_gain;
            else if (share_data->method_of_IG == Test_one || share_data->method_of_IG == Test_two)
//                view.final_utility = (share_data->sum_local_information == 0 ? 0 : view.information_gain /
//                                                                                   share_data->sum_local_information);
                view.final_utility = (share_data->sum_local_information == 0 ? 0 : view.information_gain);
            else
                view.final_utility =
                        0.7 * (share_data->sum_local_information == 0
                               ? 0
                               : view.information_gain / share_data->sum_local_information) +
                        0.3 * (share_data->robot_cost_negative ? -1 : 1) * view.robot_cost /
                        share_data->sum_robot_cost;
        }
    }
    // Update flag bit
    share_data->now_views_information_processed = true;
}

void move_robot(View *now_best_view, View_Space *now_view_space, Share_Data *share_data, NBV_Planner *nbv_plan) {
    if (share_data->num_of_max_iteration > 0 && nbv_plan->iterations + 1 >= share_data->num_of_max_iteration)
        share_data->over = true;
    if (!share_data->move_wait) {
        share_data->move_on = true;
    }
}

void visualize_octomap(pcl::visualization::PCLVisualizer::Ptr &visualizer, const octomap::ColorOcTree &octo_model,
                       Eigen::Matrix<double, 3, 1> object_center_world, double predicted_size) {
    double half_resolution = octo_model.getResolution() / 2;
    int n = 1245;
    double rr = 0.25;
    double gg = 0.25;
    double bb = 0.25;
    for (auto it = octo_model.begin_leafs(), end = octo_model.end_leafs(); it != end; ++it) {
        if (it->getOccupancy() > 0.5) {
            auto coord = it.getCoordinate();
            visualizer->addCube(coord.x() - half_resolution, coord.x() + half_resolution, coord.y() - half_resolution,
                                coord.y() + half_resolution, coord.z() - half_resolution, coord.z() + half_resolution,
                                rr, gg, bb, "cube" + to_string(++n));
//            pcl::PointXYZ pt1(coord.x() - half_resolution, coord.y() + half_resolution, coord.z() + half_resolution);
//            pcl::PointXYZ pt2(coord.x() - half_resolution, coord.y() - half_resolution, coord.z() + half_resolution);
//            pcl::PointXYZ pt3(coord.x() + half_resolution, coord.y() - half_resolution, coord.z() + half_resolution);
//            pcl::PointXYZ pt4(coord.x() + half_resolution, coord.y() + half_resolution, coord.z() + half_resolution);
//            pcl::PointXYZ pt5(coord.x() - half_resolution, coord.y() + half_resolution, coord.z() - half_resolution);
//            pcl::PointXYZ pt6(coord.x() - half_resolution, coord.y() - half_resolution, coord.z() - half_resolution);
//            pcl::PointXYZ pt7(coord.x() + half_resolution, coord.y() - half_resolution, coord.z() - half_resolution);
//            pcl::PointXYZ pt8(coord.x() + half_resolution, coord.y() + half_resolution, coord.z() - half_resolution);

//            visualizer->addLine<pcl::PointXYZ>(pt1, pt2, rr, gg, bb, "line" + to_string(++n));
//            visualizer->addLine<pcl::PointXYZ>(pt2, pt3, rr, gg, bb, "line" + to_string(++n));
//            visualizer->addLine<pcl::PointXYZ>(pt3, pt4, rr, gg, bb, "line" + to_string(++n));
//            visualizer->addLine<pcl::PointXYZ>(pt4, pt1, rr, gg, bb, "line" + to_string(++n));
//            visualizer->addLine<pcl::PointXYZ>(pt1, pt5, rr, gg, bb, "line" + to_string(++n));
//            visualizer->addLine<pcl::PointXYZ>(pt2, pt6, rr, gg, bb, "line" + to_string(++n));
//            visualizer->addLine<pcl::PointXYZ>(pt3, pt7, rr, gg, bb, "line" + to_string(++n));
//            visualizer->addLine<pcl::PointXYZ>(pt4, pt8, rr, gg, bb, "line" + to_string(++n));
//            visualizer->addLine<pcl::PointXYZ>(pt5, pt6, rr, gg, bb, "line" + to_string(++n));
//            visualizer->addLine<pcl::PointXYZ>(pt6, pt7, rr, gg, bb, "line" + to_string(++n));
//            visualizer->addLine<pcl::PointXYZ>(pt7, pt8, rr, gg, bb, "line" + to_string(++n));
//            visualizer->addLine<pcl::PointXYZ>(pt8, pt5, rr, gg, bb, "line" + to_string(++n));
        }
    }
    double x1 = object_center_world(0) - predicted_size;
    double x2 = object_center_world(0) + predicted_size;
    double y1 = object_center_world(1) - predicted_size;
    double y2 = object_center_world(1) + predicted_size;
    double z1 = object_center_world(2) - predicted_size;
    double z2 = object_center_world(2) + predicted_size;
    visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x1, y1, z1), pcl::PointXYZ(x1, y2, z1), 0, 255, 0, "cube13");
    visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x1, y1, z1), pcl::PointXYZ(x2, y1, z1), 0, 255, 0, "cube14");
    visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x1, y1, z1), pcl::PointXYZ(x1, y1, z2), 0, 255, 0, "cube15");
    visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x2, y2, z2), pcl::PointXYZ(x1, y2, z2), 0, 255, 0, "cube16");
    visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x2, y2, z2), pcl::PointXYZ(x2, y1, z2), 0, 255, 0, "cube17");
    visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x2, y2, z2), pcl::PointXYZ(x2, y2, z1), 0, 255, 0, "cube18");
    visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x2, y1, z2), pcl::PointXYZ(x1, y1, z2), 0, 255, 0, "cube19");
    visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x2, y1, z2), pcl::PointXYZ(x2, y1, z1), 0, 255, 0, "cube20");
    visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x1, y2, z2), pcl::PointXYZ(x1, y1, z2), 0, 255, 0, "cube21");
    visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x1, y2, z2), pcl::PointXYZ(x1, y2, z1), 0, 255, 0, "cube22");
    visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x2, y2, z1), pcl::PointXYZ(x1, y2, z1), 0, 255, 0, "cube23");
    visualizer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x2, y2, z1), pcl::PointXYZ(x2, y1, z1), 0, 255, 0, "cube24");
    cout << "Finished generating the octomap" << endl;
}

void generate_images(int iteration, bool save_mode, Share_Data *share_data) {
    // TODO : Make global variables in Share Data Object
    std::string path_to_abc = share_data->savePath + "/" + share_data->nameOfObject + ".abc";
    std::string path_to_obj_rescaled = share_data->savePath + "/" + share_data->nameOfObject + "_rescaled" + ".obj";
    std::string path_to_img_folder = share_data->savePath + "/img2/";
    std::string python_interpreter = "/home/alcoufr/dev/NBV-Simulation/Python_blender_API/python_env/bin/python";
    std::string python_script_folder = "Python_blender_API/";

    std::string script_name = python_script_folder + "load_render_abc.py";
    std::string parameters =
            "-f_abc " + path_to_abc + " -f_obj " + path_to_obj_rescaled + " -t " + to_string(save_mode ? 1 : 0) +
            " -s " + path_to_img_folder + share_data->nameOfObject + "_" + to_string(iteration) + ".png";
    std::string command = python_interpreter + " " + script_name + " " + parameters;
    cout << "Generating the image " << iteration << endl;
    system(command.c_str());
}

void save_rescaled(double scale, double unit, Share_Data *share_data) {
    /* Convert and save a version of the rescaled model */
    std::string filename = share_data->nameOfObject + "_rescaled";

    /* Saving the rescaled cloud with Python */
    // TODO : Add python variables to the Share Data object
    // TODO : Problem if no mtl given
    std::string path_to_obj = share_data->objectFilePath + ".obj";
    std::string path_to_obj_rescaled = share_data->savePath + '/' + filename + ".obj";
    if (std::filesystem::exists(path_to_obj_rescaled)) {
        cout << "The object has already been rescaled" << endl;
    } else if (unit * scale == 1) {
        cout << "The object doesn't need to be rescaled" << endl;
        std::string command = "cp " + path_to_obj + " " + path_to_obj_rescaled;
        system(command.c_str());
        command = "cp " + share_data->objectFilePath + ".mtl " + share_data->savePath + "/" +
                  filename + ".mtl";
        system(command.c_str());
    } else {
        cout << "The object needs to be rescaled" << endl;
        std::string python_interpreter = "/home/alcoufr/dev/NBV-Simulation/Python_blender_API/python_env/bin/python";
        std::string python_script_folder = "Python_blender_API/";
        std::string script_name = python_script_folder + "rescale_obj.py";
        std::string parameters =
                "-f_obj " + path_to_obj + " -u " + to_string(unit) + " -sc " +
                to_string(scale) + " -s " + path_to_obj_rescaled;
        std::string command = python_interpreter + " " + script_name + " " + parameters;
        system(command.c_str());
    }
}

double compute_completeness(Share_Data *share_data) {
    std::map<double, int> occ_map{};
    for (octomap::ColorOcTree::leaf_iterator it = share_data->GT_sample->begin_leafs(), end = share_data->GT_sample->end_leafs();
         it != end; ++it) {
        double occ = it->getOccupancy();
        occ_map[occ]++;
    }
    int tot_gt = 0;
    cout << "GT cpout" << endl;
    for (auto &it: occ_map) {
        if (it.first > 0.5) {
            tot_gt += it.second;
        }
//        cout << it.first << "," << it.second << endl;
    }

    std::map<double, int> occ_map_working{};
    for (octomap::ColorOcTree::leaf_iterator it = share_data->octo_model->begin_leafs(), end = share_data->octo_model->end_leafs();
         it != end; ++it) {
        double occ_work = it->getOccupancy();
        occ_map_working[occ_work]++;
    }
    int tot_work = 0;
    cout << "Octo cpout" << endl;
    for (auto &it: occ_map_working) {
        if (it.first > 0.5) {
            tot_work += it.second;
        }
//        cout << it.first << "," << it.second << endl;
    }
    double ratio = (double)tot_work / (double) tot_gt * 100;
    cout << "The object is reconstructed at : " << ratio << '%' << endl;
    return ratio;
}

[[maybe_unused]] void show_cloud(const pcl::visualization::PCLVisualizer::Ptr &viewer) {
    // pcl display point cloud
    while (!viewer->wasStopped()) {
        viewer->spin();
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}
