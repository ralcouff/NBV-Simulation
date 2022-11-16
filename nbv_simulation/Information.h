#pragma once

#include "View_Space.h"
#include "Ray_Hash.h"
#include "Ray_Information.h"
#include "MCMF.h"

#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <opencv2/opencv.hpp>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std;

void ray_expand_thread_process(int *ray_num,
                               Ray_Information **rays_info,
                               unordered_map<Ray, int, Ray_Hash> *rays_map,
                               unordered_map<int, vector<int>> *views_to_rays_map,
                               unordered_map<int, vector<int>> *rays_to_views_map,
                               octomap::ColorOcTree *octo_model,
                               Voxel_Information *voxel_information,
                               View_Space *view_space,
                               rs2_intrinsics *color_intrinsics,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr frontier,
                               int pos);

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

double information_function(short &method,
                            double &ray_information,
                            double voxel_information,
                            double &visible,
                            bool &is_unknown,
                            bool &previous_voxel_unknown,
                            bool &is_endpoint,
                            bool &is_occupied,
                            double &object,
                            double &object_visible);

/**
 *
 * @param ray_id
 * @param rays_info
 * @param rays_map
 * @param occupancy_map
 * @param object_weight
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
        octomap::ColorOcTree *octo_model,
        Voxel_Information *voxel_information,
        View_Space *view_space,
        short method);

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

void information_gain_thread_process(Ray_Information **rays_info,
                                     unordered_map<int, vector<int>> *views_to_rays_map,
                                     View_Space *view_space,
                                     int pos);
