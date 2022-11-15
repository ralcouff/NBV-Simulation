#ifndef NBV_SIMULATION_PERCEPTION_3D_H
#define NBV_SIMULATION_PERCEPTION_3D_H

#include <boost/thread/thread.hpp>
#include <octomap/ColorOcTree.h>
#include <pcl/point_cloud.h>

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

    bool precept(View *now_best_view);

};

void precept_thread_process(int x,
                            int y,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                            octomap::point3d *_origin,
                            Eigen::Matrix4d *_view_pose_world,
                            Share_Data *share_data);

#endif //NBV_SIMULATION_PERCEPTION_3D_H
