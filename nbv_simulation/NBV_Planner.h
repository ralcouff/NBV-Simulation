#ifndef NBV_SIMULATION_NBV_PLANNER_H
#define NBV_SIMULATION_NBV_PLANNER_H

#pragma once


#include <thread>
#include "Perception_3D.h"
#include "Voxel_Information.h"
#include "View_Space.h"
#include "Views_Information.h"
#include "views_voxels_MF.h"

#define Over 0
#define WaitData 1
#define WaitViewSpace 2
#define WaitInformation 3
#define WaitMoving 4

class NBV_Planner {

public:
    atomic<int> status{};
    int iterations;
    Perception_3D *percept;
    Voxel_Information *voxel_information;
    View_Space *now_view_space;
    Views_Information *now_views_information{nullptr};
    View *now_best_view;
    Share_Data *share_data;
    pcl::visualization::PCLVisualizer::Ptr viewer;

    /**
     * Constructor of the NBV_Planner object.
     * @param _share_data The pointer to the data needed for the project
     * @param _status The initial status of the NBV_Planner (default=WaitData)
     */
    explicit NBV_Planner(Share_Data *_share_data, int _status = WaitData);

    /**
     * Give the percentage of points within a given radius around a 3D point.
     * @param predicted_size The estimated given radius
     * @param object_center_world The gravity center of the point cloud.
     * @param points The points of the point cloud.
     * @return The percentage of points of the point cloud within the given radius.
     */
    static double check_size(double predicted_size, Eigen::Vector3d object_center_world, vector<Eigen::Vector3d> &points);

    int plan();

    string out_status();

};

void save_cloud_mid_thread_process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string name, Share_Data *share_data);

void create_view_space(View_Space **now_view_space, View *now_best_view, Share_Data *share_data, int iterations);

void create_views_information(Views_Information **now_views_information,
                              View *now_best_view,
                              View_Space *now_view_space,
                              Share_Data *share_data,
                              NBV_Planner *nbv_plan,
                              int iterations);

void move_robot(View *now_best_view, View_Space *now_view_space, Share_Data *share_data, NBV_Planner *nbv_plan);

void show_cloud(pcl::visualization::PCLVisualizer::Ptr viewer);


#endif //NBV_SIMULATION_NBV_PLANNER_H
