#ifndef NBV_SIMULATION_NBV_PLANNER_H
#define NBV_SIMULATION_NBV_PLANNER_H

#include <pcl/visualization/pcl_visualizer.h>

#include "Perception_3D.h"
#include "Views_Information.h"
#include "View_Space.h"

#define Over 0
#define WaitData 1
#define WaitViewSpace 2
#define WaitInformation 3
#define WaitMoving 4

class NBV_Planner {
public:
    atomic<int> status{};
    int iterations;
    View_Space *current_view_space;
    View *current_best_view;
    Share_Data *share_data;
    pcl::visualization::PCLVisualizer::Ptr visualizer;
    Perception_3D *percept;
    Voxel_Information *voxel_information;
    Views_Information *current_views_information{};


    /**
     * Constructor of the NBV_Planner object.
     * It opens a window with the ground truth point cloud, the candidate viewpoints and the first camera pose.
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
    static double
    check_size(double predicted_size, Eigen::Vector3d object_center_world, vector<Eigen::Vector3d> &points);

    /**
     * Compute and select the next best view
     * @return The status of the algorithm
     */
    int plan();

    /**
     * Give the status of the NBV Planner
     * @return The status of the NBV Planner as a string
     */
    string out_status();

};

/**
 * Updating the View Space according to the current best view.
 * @param current_view_space The View Space to update
 * @param current_best_view The current best view
 * @param share_data The data shared through all the files
 * @param iterations The iteration number
 */
void
create_view_space(View_Space **current_view_space, View *current_best_view, Share_Data *share_data, int iterations);

/**
 * Compute the global information function for each view (eq. 6)
 * @param current_views_information The information contained in the views
 * @param current_best_view FIXME : unused
 * @param current_view_space The current view space
 * @param share_data The data shared through the whole project
 * @param nbv_plan The NBV Planner
 * @param iterations The number of iterations
 */
void create_views_information(Views_Information **current_views_information,
                              View *current_best_view,
                              View_Space *current_view_space,
                              Share_Data *share_data,
                              NBV_Planner *nbv_plan,
                              int iterations);

/**
 * Virtually move the robot to the chosen best view
 * @param now_best_view The view where the robot has to move
 * @param now_view_space The current view space
 * @param share_data The data shared through the whole project
 * @param nbv_plan The NBV Planner
 */
void move_robot(View *now_best_view, View_Space *now_view_space, Share_Data *share_data, NBV_Planner *nbv_plan);

#endif //NBV_SIMULATION_NBV_PLANNER_H