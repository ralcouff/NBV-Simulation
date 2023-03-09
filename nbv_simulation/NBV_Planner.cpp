#include "NBV_Planner.h"

NBV_Planner::NBV_Planner(Share_Data *_share_data, int _status) {
    // Initializing parameters of the NBV Planner
    share_data = _share_data;
    status = _status;
    iterations = 0;
    share_data->current_view_space_processed = false;
    share_data->current_views_information_processed = false;
    share_data->move_on = false;

    /* Making sure the save_path exists */
    Share_Data::access_directory(share_data->save_path);

    /* Initializing the working cloud */
    share_data->working_cloud->is_dense = false;
    share_data->working_cloud->points.resize(share_data->input_cloud->points.size());
    share_data->working_cloud->width = share_data->input_cloud->points.size();
    share_data->working_cloud->height = 1;

    /* Creating iterators to the points of working_cloud (ptr) and input_cloud (p). */
    auto p = share_data->input_cloud->points.begin();
    auto ptr = share_data->working_cloud->points.begin();

    /* Re-scaling the model if it's too big */
    float unit = 1.0;
    for (auto &point: share_data->input_cloud->points) {
        if (fabs(point.x) >= 10 || fabs(point.y) >= 10 || fabs(point.z) >= 10) {
            unit = 0.001;
            cout << "Changing unit from <mm> to <m>." << endl;
            break;
        }
    }
    /* Check the size of the object and scale it uniformly to about 0.15m. */
    vector<Eigen::Vector3d> points;
    for (auto &point: share_data->input_cloud->points) {
        Eigen::Vector3d pt(point.x * unit, point.y * unit, point.z * unit);
        points.push_back(pt);
    }

    /* Computing and initializing the center of mass of the point cloud */
    Eigen::Vector3d object_center_world = Eigen::Vector3d(0, 0, 0);
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

    /* Converting the Points. */
    for (int i = 0; i < share_data->input_cloud->points.size(); i++, p++) {
        (*ptr).x = (*p).x * scale * unit;
        (*ptr).y = (*p).y * scale * unit;
        (*ptr).z = (*p).z * scale * unit;
        (*ptr).b = 168;
        (*ptr).g = 168;
        (*ptr).r = 168;
        ptr++;
    }
    voxel_information = new Voxel_Information(share_data->p_unknown_lower_bound, share_data->p_unknown_upper_bound);
    current_view_space = new View_Space(iterations, share_data, voxel_information, share_data->working_cloud);
    current_view_space->views[0].vis++;
    current_best_view = new View(current_view_space->views[0]);
    percept = new Perception_3D(share_data);

    /* Compute the number of leaf nodes in the octo_model*/
    share_data->init_voxels = (int) share_data->octo_model->getNumLeafNodes();

    /* Octo model update. */
    share_data->octo_model->updateInnerOccupancy();

    /* Saving a version of the initial octree. */
    share_data->octo_model->write(share_data->save_path + "/GT.ot");

    std::map<double, int> occupancy{};
    for (octomap::ColorOcTree::tree_iterator it = share_data->octo_model->begin_tree(), end = share_data->octo_model->end_tree();
         it != end; ++it) {
        double occ = it->getOccupancy();
        occupancy[occ]++;
    }
    cout << "Occupancy in octo_model" << endl;
    for (auto &it: occupancy) {
        std::cout << it.first << " - " << it.second << endl;
    }

    /* Prints to check the good initialization. */
    cout << "Number of points in input_cloud: " << share_data->input_cloud->points.size() << endl;
    cout << "Number of nodes in octo_model: " << share_data->octo_model->calcNumNodes() << endl;
    cout << "Number of leaf nodes in octo_model: " << share_data->init_voxels << endl;

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
            if (percept->percept(current_best_view)) {
                create_view_space(&current_view_space, current_best_view, share_data, iterations);
                status = WaitViewSpace;
            }
            break;
        case WaitViewSpace:
            if (share_data->current_view_space_processed) {
                create_views_information(&current_views_information, current_best_view, current_view_space, share_data,
                                         this, iterations);
                status = WaitInformation;
            }
            break;
        case WaitInformation:
            if (share_data->current_views_information_processed) {
                // Sorting the views according to their final utility
                sort(current_view_space->views.begin(), current_view_space->views.end(), view_utility_compare);
                // Saving the utility of each view with its id
                std::ofstream InfoOUT(
                        share_data->save_path + '/' + "global_information.csv");
                for (auto &view: current_view_space->views) {
                    InfoOUT << view.id << "," << view.final_utility << endl;
                }
                double max_utility = -1;
                for (int i = 0; i < current_view_space->views.size(); i++) {
                    cout << "checking view " << i << endl;
                    if (current_view_space->views[i].vis)
                        continue;
                    current_best_view = new View(current_view_space->views[i]);
                    max_utility = current_best_view->final_utility;
                    current_view_space->views[i].vis++;
                    share_data->best_views.push_back(*current_best_view);
                    cout << "Best_Views : " << share_data->best_views.size() << endl;
                    cout << "View " << i << " has been chosen." << endl;
                    break;
                }
                if (max_utility == -1) {
                    cout << "Can't move to any viewpoint. Stop." << endl;
                    status = Over;
                    break;
                }
                cout << " Next best view position is:(" << current_best_view->init_pos(0) << ", "
                     << current_best_view->init_pos(1) << ", " << current_best_view->init_pos(2) << ")" << endl;
                cout << " Next best view final_utility is " << current_best_view->final_utility << endl;
                //TODO: Save the results in a csv file
//                std::ofstream result(share_data->test_base_filename, std::ios_base::app);
//                result << share_data->method_of_IG << ',' << share_data->n_model << ',' << share_data->n_size << ','
//                       << std::to_string(iterations) << ',' << current_best_view->id << ','
//                       << current_best_view->init_pos(0) << ',' << current_best_view->init_pos(1) << ','
//                       << current_best_view->init_pos(2) << ',' << current_best_view->final_utility << endl;
//                result.close();
                thread next_moving(move_robot, current_best_view, current_view_space, share_data, this);
                next_moving.detach();
                status = WaitMoving;
            }
            break;
        case WaitMoving:
            if (share_data->over) {
                cout << "Progress over. Saving octomap and cloud." << endl;
                status = Over;
                break;
            }
            if (share_data->move_on) {
                iterations++;
                share_data->current_view_space_processed = false;
                share_data->current_views_information_processed = false;
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

void create_view_space(View_Space **current_view_space,
                       View *current_best_view,
                       Share_Data *share_data,
                       int iterations) {
    // Updating the camera pose
    share_data->current_camera_pose_world = (share_data->current_camera_pose_world *
                                             current_best_view->pose.inverse()).eval();
    //NOTE: The update function is no longer usefull as we are only asking for one view and not all of them.
//    (*current_view_space)->update(iterations, share_data, share_data->working_cloud, share_data->clouds[iterations]);
    share_data->current_view_space_processed = true;
}

void create_views_information(Views_Information **current_views_information,
                              View *current_best_view,
                              View_Space *current_view_space,
                              Share_Data *share_data,
                              NBV_Planner *nbv_plan,
                              int iterations) {
    if (iterations == 0) {
        (*current_views_information) = new Views_Information(share_data, nbv_plan->voxel_information,
                                                             current_view_space, iterations);
    }
    //TODO: Make a distinction if it is our method or not
    share_data->sum_local_information = 0;
    share_data->sum_global_information = 0;
    for (auto &view: current_view_space->views) {
        share_data->sum_local_information += view.information_gain;
        share_data->sum_global_information += view.get_global_information();
    }
    if (share_data->sum_local_information == 0)
        cout << "Full local information is zero." << endl;
    if (share_data->sum_global_information == 0)
        cout << "Full global information is zero." << endl;
    for (auto &view: current_view_space->views) {
        if (share_data->method_of_IG == OursIG)
            view.final_utility =
                    (1 - share_data->cost_weight) * view.information_gain /
                    share_data->sum_local_information +
                    share_data->cost_weight * view.get_global_information() /
                    share_data->sum_global_information;
        else if (share_data->method_of_IG == APORA or share_data->method_of_IG == Test_one)
            view.final_utility = view.information_gain;
        else
            view.final_utility = 0;
        // TODO: Add robot cost and update that part
//            view.final_utility =
//                    0.7 * (share_data->sum_local_information == 0
//                           ? 0
//                           : view.information_gain / share_data->sum_local_information) +
//                    0.3 * (share_data->robot_cost_negative ? -1 : 1) * view.robot_cost /
//                    share_data->sum_robot_cost;

    }
    // Update flag bit
    share_data->current_views_information_processed = true;
}

void move_robot(View *now_best_view, View_Space *now_view_space, Share_Data *share_data, NBV_Planner *nbv_plan) {
    if (share_data->num_of_max_iteration > 0 && nbv_plan->iterations + 1 >= share_data->num_of_max_iteration)
        share_data->over = true;
    if (!share_data->move_wait)
        share_data->move_on = true;
}