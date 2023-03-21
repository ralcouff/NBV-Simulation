#pragma once

#include <octomap/ColorOcTree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Voxel_Information.h"
#include "Ray_Information.h"
#include "Ray_Hash.h"
#include "View_Space.h"

/**
 *
 * @param node
 * @param octo_model
 * @param voxel_information
 * @param octomap_resolution
 * @return
 */
int frontier_check(octomap::point3d node,
                   octomap::ColorOcTree *octo_model,
                   Voxel_Information *voxel_information,
                   double octomap_resolution);

/**
 * Casting and creating all the Rays going from the views to the model
 * @param ray_num
 * @param rays_info
 * @param rays_map
 * @param views_to_rays_map
 * @param rays_to_views_map
 * @param octo_model
 * @param voxel_information
 * @param view_space
 * @param color_intrinsics
 * @param pos
 */
void ray_cast_thread_process(int *ray_num,
                             Ray_Information **rays_info,
                             unordered_map<Ray, int, Ray_Hash> *rays_map,
                             unordered_map<int, vector<int>> *views_to_rays_map,
                             unordered_map<int, vector<int>> *rays_to_views_map,
                             octomap::ColorOcTree *octo_model,
                             Voxel_Information *voxel_information,
                             View_Space *view_space,
                             rs2_intrinsics *color_intrinsics,
                             int pos);

/**
 *
 * @param ray_id
 * @param rays_info
 * @param rays_map
 * @param occupancy_map
 * @param object_weight
 * @param quality_weight
 * @param octo_model
 * @param voxel_information
 * @param view_space
 * @param method
 */
void ray_information_thread_process(
        int ray_id,
        Ray_Information **rays_info,
        unordered_map<Ray, int, Ray_Hash> *rays_map,
        unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *occupancy_map,
        unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *object_weight,
        unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> *quality_weight,
        octomap::ColorOcTree *octo_model,
        Voxel_Information *voxel_information,
        View_Space *view_space,
        short method);

/**
 *
 * @param method An int describing the method to use to compute the information
 * @param ray_information
 * @param voxel_information
 * @param visible p_vis
 * @param is_unknown
 * @param previous_voxel_unknown
 * @param is_endpoint
 * @param is_occupied
 * @param object p_obj
 * @param object_visible \PI{1-P_obj_x}
 * @return information gain for each ray
 */
double information_function(short &method,
                            double &ray_information,
                            double voxel_information,
                            double &visible,
                            bool &is_unknown,
                            bool &previous_voxel_unknown,
                            bool &is_endpoint,
                            bool &is_occupied,
                            double &object,
                            double &qlt,
                            double &object_visible);

/**
 * Compute the information gain for each view according to the information gain of each ray containing that view
 * @param rays_info The information about rays
 * @param views_to_rays_map A mapping of Views to rays
 * @param view_space The View Space
 * @param pos The considered voxel
 */
void information_gain_thread_process(Ray_Information **rays_info,
                                     unordered_map<int, vector<int>> *views_to_rays_map,
                                     View_Space *view_space,
                                     int pos);