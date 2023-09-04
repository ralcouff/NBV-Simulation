#ifndef NBV_SIMULATION_PERCEPTION_3D_H
#define NBV_SIMULATION_PERCEPTION_3D_H

#include <boost/thread/thread.hpp>
#include <octomap/ColorOcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/incremental_registration.h>

#include "Share_Data.h"
#include "View.h"

class Perception_3D {

public:
    Share_Data *share_data;
    octomap::ColorOcTree *ground_truth_model;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    int iterations;

    /**
     * Constructor of the Perception_3D object
     * @param _share_data The data shared through the whole project
     */
    explicit Perception_3D(Share_Data *_share_data);

    /**
     * Generate the voxels seen by the camera at the position now_best_view
     * @param now_best_view The view from where to generate the cloud
     * @return TRUE if the cloud has been correctly generated
     */
    bool percept(View *now_best_view);

};

/**
 * Add the end points of the ray casting to the current cloud
 * @param x The x coordinate of the pixel in image
 * @param y The y coordinate of the pixel in image
 * @param cloud The cloud onto which we are working
 * @param _origin A pointer to the origin of the world
 * @param _view_pose_world The current view pose
 * @param share_data The shared data among the whole project
 */
void percept_thread_process(int x,
                            int y,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                            octomap::point3d *_origin,
                            Eigen::Matrix4d *_view_pose_world,
                            Share_Data *share_data);

#endif //NBV_SIMULATION_PERCEPTION_3D_H
