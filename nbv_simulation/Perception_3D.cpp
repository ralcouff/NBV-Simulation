#include "Perception_3D.h"

Perception_3D::Perception_3D(Share_Data *_share_data) {
    share_data = _share_data;
    ground_truth_model = share_data->ground_truth_model;
    iterations = 0;
}

bool Perception_3D::percept(View *now_best_view) {
    /* Setting up a timer. */
    auto now_time = clock();
    /* Create the current imaging point cloud. */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_parallel(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_parallel->is_dense = false;
    cloud_parallel->points.resize(share_data->color_intrinsics.width * share_data->color_intrinsics.height);
    /* Get the Viewpoint pose. */
    Eigen::Matrix4d view_pose_world;
    now_best_view->get_next_camera_pos(share_data->now_camera_pose_world, share_data->object_center_world);
    view_pose_world = (share_data->now_camera_pose_world * now_best_view->pose.inverse()).eval();
    if (share_data->reconstructionMethod == 0) {
        // Using the embedded perfect Lidar acquisition
        // Check the key of the viewpoint
        octomap::OcTreeKey key_origin;
        bool key_origin_have = ground_truth_model->coordToKeyChecked(
                now_best_view->init_pos(0), now_best_view->init_pos(1), now_best_view->init_pos(2), key_origin);
        if (key_origin_have) {
            octomap::point3d origin = ground_truth_model->keyToCoord(key_origin);
            // Traversing the image plane
//            thread** precept_process =
//              new thread*[share_data->color_intrinsics.width * share_data->color_intrinsics.height];
//            for(int x = 0; x < share_data->color_intrinsics.width; x++)
//                for(int y = 0; y < share_data->color_intrinsics.height; y++)
//                {
//                    int i = x * share_data->color_intrinsics.height + y;
//                    precept_process[i] =
//                      new thread(percept_thread_process, x, y, cloud_parallel, &origin, &view_pose_world, share_data);
//                }
//            for(int x = 0; x < share_data->color_intrinsics.width; x++)
//                for(int y = 0; y < share_data->color_intrinsics.height; y++)
//                {
//                    int i = x * share_data->color_intrinsics.height + y;
//                    (*precept_process[i]).join();
//                }
            /* Traversing the image plane. */
            for (int x = 0; x < share_data->color_intrinsics.width; ++x)
                for (int y = 0; y < share_data->color_intrinsics.height; ++y) {
                    int i = x * share_data->color_intrinsics.height + y;
                    percept_thread_process(x, y, cloud_parallel, &origin, &view_pose_world, share_data);
                }
        } else {
            cout << "View out of map. Check Viewpoint generation." << endl;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud = temp;
        cloud->is_dense = false;
        cloud->points.resize(share_data->color_intrinsics.width * share_data->color_intrinsics.height);
        auto ptr = cloud->points.begin();
        int valid_point = 0;
        auto p = cloud_parallel->points.begin();
        for (int i = 0; i < cloud_parallel->points.size(); i++, p++) {
            if ((*p).x == 0 && (*p).y == 0 && (*p).z == 0)
                continue;
            (*ptr).x = (*p).x;
            (*ptr).y = (*p).y;
            (*ptr).z = (*p).z;
            (*ptr).b = (*p).b;
            (*ptr).g = (*p).g;
            (*ptr).r = (*p).r;
            valid_point++;
            ptr++;
        }
        cloud->width = valid_point;
        cloud->height = 1;
        cloud->points.resize(valid_point);
        // Record the current collection point cloud
        share_data->valid_clouds++;
        share_data->clouds.push_back(cloud);
        // Rotate to the world coordinate system
        *share_data->cloud_final += *cloud;
        cout << "Virtual cloud got in: " << clock() - now_time << " ms." << endl;
        cout << valid_point << " points were acquired" << endl;
//    if (share_data->show) {
//        // Display imaging point clouds
//        pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("Camera"));
//        viewer1->setBackgroundColor(0, 0, 0);
//        viewer1->addCoordinateSystem(0.1);
//        viewer1->initCameraParameters();
//        viewer1->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
//        Eigen::Vector4d X(0.05, 0, 0, 1);
//        Eigen::Vector4d Y(0, 0.05, 0, 1);
//        Eigen::Vector4d Z(0, 0, 0.05, 1);
//        Eigen::Vector4d O(0, 0, 0, 1);
//        X = view_pose_world * X;
//        Y = view_pose_world * Y;
//        Z = view_pose_world * Z;
//        O = view_pose_world * O;
//        viewer1->addLine<pcl::PointXYZ>(
//                pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(X(0), X(1), X(2)), 255, 0, 0, "X" + std::to_string(-1));
//        viewer1->addLine<pcl::PointXYZ>(
//                pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Y(0), Y(1), Y(2)), 0, 255, 0, "Y" + std::to_string(-1));
//        viewer1->addLine<pcl::PointXYZ>(
//                pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Z(0), Z(1), Z(2)), 0, 0, 255, "Z" + std::to_string(-1));
//        while (!viewer1->wasStopped()) {
//            viewer1->spin();
//            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//        }
//    }
    } else {
        // Using and external reconstruction algorithm.
        // Loading the next partial 3D model.
        std::string pathTo3DModel;
        bool fileExist = false;
        bool is3DFile = false;
        while (!fileExist || !is3DFile) {
            cout << "Enter a valid path to the next partial reconstructed model." << endl;
            cin >> pathTo3DModel;
            share_data->pathToLast3DModel = pathTo3DModel;
            fileExist = std::filesystem::exists(pathTo3DModel);
            if (fileExist) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
                cloud = temp;
                cloud->is_dense = false;
                if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pathTo3DModel, *cloud) == -1) {
                    if (pcl::io::loadOBJFile<pcl::PointXYZRGB>(pathTo3DModel, *cloud) == -1) {
                        if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(pathTo3DModel, *cloud) == -1) {
                            cout << "No 3D file where found there. Check your input name : " << pathTo3DModel << endl;
                        } else {
                            is3DFile = true;
                        }
                    } else {
                        is3DFile = true;
                    }
                } else {
                    is3DFile = true;
                }
            }
        }
        // Record the current collection point cloud
        share_data->valid_clouds++;
        share_data->clouds.push_back(cloud);
        // Rotate to the world coordinate system
        *share_data->cloud_final += *cloud;
        cout << "Virtual cloud got in: " << clock() - now_time << " ms." << endl;
    }
    iterations++;
    return true;
}

void percept_thread_process(int x,
                            int y,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                            octomap::point3d *_origin,
                            Eigen::Matrix4d *_view_pose_world,
                            Share_Data *share_data) {
    octomap::point3d origin = *_origin; // The origin of the rays, corresponding to the position of camera.
    Eigen::Matrix4d view_pose_world = *_view_pose_world; // The current camera pose
    /* Reverse projection to find the endpoint. */
    octomap::point3d end = project_pixel_to_ray_end(x, y, share_data->color_intrinsics, view_pose_world, 1.0);
    octomap::OcTreeKey key_end;
    octomap::point3d direction = end - origin;
    octomap::point3d end_point;
    pcl::PointXYZRGB point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    point.b = 0;
    point.g = 0;
    point.r = 0;
    /* Casting a ray from the camera among a certain direction. */
    // Crossing unknown areas and finding the end
    bool found_end_point =
            share_data->ground_truth_model->castRay(origin, direction, end_point, true,
                                                    6.0 * share_data->predicted_size);
    if (!found_end_point) {
        // Endpoint not found, no observations available
        cloud->points[x * share_data->color_intrinsics.height + y] = point;
        return;
    }
    if (end_point == origin) {
        cout << "View in the object. Check Viewpoints generation !" << endl;
        cloud->points[x * share_data->color_intrinsics.height + y] = point;
        return;
    }
    // Check to see if the end is within the map limits
    bool key_end_have = share_data->ground_truth_model->coordToKeyChecked(end_point, key_end);
    if (key_end_have) {
        octomap::ColorOcTreeNode *node = share_data->ground_truth_model->search(key_end);
        if (node != nullptr) {
            octomap::ColorOcTreeNode::Color color = node->getColor();
            point.x = end_point.x();
            point.y = end_point.y();
            point.z = end_point.z();
            point.b = color.b;
            point.g = color.g;
            point.r = color.r;
        }
    }
    cloud->points[x * share_data->color_intrinsics.height + y] = point;
}

[[maybe_unused]] void icp_align_mesh(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_gt,
                                     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr icp;
    icp.reset(new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>());
    icp->setMaximumIterations(100);
    icp->setMaxCorrespondenceDistance(5);
    icp->setRANSACOutlierRejectionThreshold(0.2);
    pcl::registration::IncrementalRegistration<pcl::PointXYZRGB> iicp;
    iicp.setRegistration(icp);

    pcl::Indices dummyIndices;
    pcl::removeNaNFromPointCloud(*cloud_gt, *cloud_gt, dummyIndices);

    if (!iicp.registerCloud(cloud_gt)) {
        std::cout << "Registration failed. Resetting transform" << std::endl;
        iicp.reset();
        iicp.registerCloud(cloud_gt);
    };

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud_gt, *tmp, iicp.getAbsoluteTransform());

    std::cout << iicp.getAbsoluteTransform() << std::endl;
    std::string result_filename("../Banane.pcd");

    result_filename = result_filename.substr(result_filename.rfind('/') + 1);
    pcl::io::savePCDFileBinary(result_filename, *tmp);
    pcl::PolygonMesh mesh_obj;
    pcl::toPCLPointCloud2(*tmp, mesh_obj.cloud);
    pcl::io::saveOBJFile("Banane.obj", mesh_obj);
    std::cout << "saving result to " << result_filename << std::endl;

    pcl::removeNaNFromPointCloud(*cloud, *cloud, dummyIndices);

    if (!iicp.registerCloud(cloud)) {
        std::cout << "Registration failed. Resetting transform" << std::endl;
        iicp.reset();
        iicp.registerCloud(cloud);
    };

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmptwo(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud, *tmptwo, iicp.getAbsoluteTransform());

    std::cout << iicp.getAbsoluteTransform() << std::endl;
    std::string result_filename_two("../Pomme.pcd");

    result_filename_two = result_filename_two.substr(result_filename_two.rfind('/') + 1);
    pcl::io::savePCDFileBinary(result_filename_two, *tmptwo);
    pcl::PolygonMesh mesh_obj_two;
    pcl::toPCLPointCloud2(*tmptwo, mesh_obj_two.cloud);
    pcl::io::saveOBJFile("Pomme.obj", mesh_obj_two);
    std::cout << "saving result to " << result_filename_two << std::endl;
}