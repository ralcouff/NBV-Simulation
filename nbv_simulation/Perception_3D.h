#ifndef NBV_SIMULATION_PERCEPTION_3D_H
#define NBV_SIMULATION_PERCEPTION_3D_H

#include "Share_Data.h"
#include "View.h"

class Perception_3D {

public:
    Share_Data *share_data;
    octomap::ColorOcTree *octo_model;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    int iterations;

    /**
     * Constructor of the Perception_3D object
     * @param _share_data The data shared through the whole project
     */
    explicit Perception_3D(Share_Data *_share_data);

    /**
     * Generate the voxels seen by the camera at the position now_best_view
     * @param current_best_view The view from where to generate the cloud
     * @return TRUE if the cloud has been correctly generated
     */
    bool percept(View *current_best_view);
};

#endif //NBV_SIMULATION_PERCEPTION_3D_H
