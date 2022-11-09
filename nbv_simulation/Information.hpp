#pragma once

#include "View_Space.hpp"
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

// void ray_graph_thread_process(int ray_id,Ray_Information** rays_info, unordered_map<int, vector<int>>*
// rays_to_viwes_map, unordered_map<octomap::OcTreeKey, unordered_set<int>, octomap::OcTreeKey::KeyHash>* end_id_map,
// Voxel_Information* voxel_information);
void information_gain_thread_process(Ray_Information** rays_info,
                                     unordered_map<int, vector<int>>* views_to_rays_map,
                                     View_Space* view_space,
                                     int pos);

void ray_expand_thread_process(int* ray_num,
                               Ray_Information** rays_info,
                               unordered_map<Ray, int, Ray_Hash>* rays_map,
                               unordered_map<int, vector<int>>* views_to_rays_map,
                               unordered_map<int, vector<int>>* rays_to_viwes_map,
                               octomap::ColorOcTree* octo_model,
                               Voxel_Information* voxel_information,
                               View_Space* view_space,
                               rs2_intrinsics* color_intrinsics,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr frontier,
                               int pos);

void ray_cast_thread_process(int* ray_num,
                             Ray_Information** rays_info,
                             unordered_map<Ray, int, Ray_Hash>* rays_map,
                             unordered_map<int, vector<int>>* views_to_rays_map,
                             unordered_map<int, vector<int>>* rays_to_viwes_map,
                             octomap::ColorOcTree* octo_model,
                             Voxel_Information* voxel_information,
                             View_Space* view_space,
                             rs2_intrinsics* color_intrinsics,
                             int pos);

bool is_pixel_in_convex(vector<cv::Point2f>& hull, cv::Point2f& pixel);

vector<cv::Point2f> get_convex_on_image(vector<Eigen::Vector4d>& convex_3d,
                                        Eigen::Matrix4d& now_camera_pose_world,
                                        rs2_intrinsics& color_intrinsics,
                                        int& pixel_interval,
                                        double& max_range,
                                        double& octomap_resolution);

octomap::point3d project_pixel_to_ray_end(int x,
                                          int y,
                                          rs2_intrinsics& color_intrinsics,
                                          Eigen::Matrix4d& now_camera_pose_world,
                                          float max_range = 1.0);

double information_function(short& method,
                            double& ray_information,
                            double voxel_information,
                            double& visible,
                            bool& is_unknown,
                            bool& previous_voxel_unknown,
                            bool& is_endpoint,
                            bool& is_occupied,
                            double& object,
                            double& object_visible);

void ray_information_thread_process(
  int ray_id,
  Ray_Information** rays_info,
  unordered_map<Ray, int, Ray_Hash>* rays_map,
  unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>* occupancy_map,
  unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>* object_weight,
  octomap::ColorOcTree* octo_model,
  Voxel_Information* voxel_information,
  View_Space* view_space,
  short method);

int frontier_check(octomap::point3d node,
                   octomap::ColorOcTree* octo_model,
                   Voxel_Information* voxel_information,
                   double octomap_resolution);

double distance_function(double distance, double alpha);

class Views_Information
{
  public:
    double cost_weight;
    Ray_Information** rays_info;
    unordered_map<int, vector<int>>* views_to_rays_map;
    unordered_map<int, vector<int>>* rays_to_viwes_map;
    unordered_map<Ray, int, Ray_Hash>* rays_map;
    unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>* occupancy_map;
    unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>* object_weight;
    long long max_num_of_rays;
    int ray_num;
    double alpha;
    int K = 6;
    rs2_intrinsics color_intrinsics;
    Voxel_Information* voxel_information;
    octomap::ColorOcTree* octo_model;
    double octomap_resolution;
    int method;
    int pre_edge_cnt;
    std::size_t edge_cnt;

    Views_Information(Share_Data* share_data,
                      Voxel_Information* _voxel_information,
                      View_Space* view_space,
                      int iterations)
    {
        // Update internal data
        voxel_information = _voxel_information;
        cost_weight = share_data->cost_weight;
        color_intrinsics = share_data->color_intrinsics;
        method = share_data->method_of_IG;
        octo_model = share_data->octo_model;
        octomap_resolution = share_data->octomap_resolution;
        voxel_information->octomap_resolution = octomap_resolution;
        alpha = 0.1 / octomap_resolution;
        voxel_information->skip_coefficient = share_data->skip_coefficient;
        // Note that the viewpoints need to be sorted by id to create the mapping
        sort(view_space->views.begin(), view_space->views.end(), view_id_compare);
        auto now_time = clock();
        views_to_rays_map = new unordered_map<int, vector<int>>();
        rays_to_viwes_map = new unordered_map<int, vector<int>>();
        rays_map = new unordered_map<Ray, int, Ray_Hash>();
        object_weight = new unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>();
        occupancy_map = new unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>();
        // Define frontier
        vector<octomap::point3d> points;
        pcl::PointCloud<pcl::PointXYZ>::Ptr edge(new pcl::PointCloud<pcl::PointXYZ>);
        double map_size = view_space->predicted_size;
        // Find edges in the map
        for(octomap::ColorOcTree::leaf_iterator it = octo_model->begin_leafs(), end = octo_model->end_leafs();
            it != end;
            ++it)
        {
            double occupancy = (*it).getOccupancy();
            // Record the mapping of key to occ rate in bbx for repeated queries
            (*occupancy_map)[it.getKey()] = occupancy;
            if(voxel_information->is_unknown(occupancy))
            {
                auto coordinate = it.getCoordinate();
                if(coordinate.x() >= view_space->object_center_world(0) - map_size &&
                   coordinate.x() <= view_space->object_center_world(0) + map_size &&
                   coordinate.y() >= view_space->object_center_world(1) - map_size &&
                   coordinate.y() <= view_space->object_center_world(1) + map_size &&
                   coordinate.z() >= view_space->object_center_world(2) - map_size &&
                   coordinate.z() <= view_space->object_center_world(2) + map_size)
                {
                    points.push_back(coordinate);
                    if(frontier_check(coordinate, octo_model, voxel_information, octomap_resolution) == 2)
                        edge->points.emplace_back(coordinate.x(), coordinate.y(), coordinate.z());
                }
            }
        }
        pre_edge_cnt = 0x3f3f3f3f;
        edge_cnt = edge->points.size();
        // Calculate the probability that the point in the map is the surface of an object based on the nearest frontier
        if(edge->points.size() != 0)
        {
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(edge);
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            for(int i = 0; i < points.size(); i++)
            {
                octomap::OcTreeKey key;
                bool key_have = octo_model->coordToKeyChecked(points[i], key);
                if(key_have)
                {
                    pcl::PointXYZ searchPoint(points[i].x(), points[i].y(), points[i].z());
                    int num = kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
                    if(num > 0)
                    {
                        double p_obj = 1;
                        for(int j = 0; j < pointIdxNKNSearch.size(); j++)
                        {
                            p_obj *= distance_function(pointNKNSquaredDistance[j], alpha);
                        }
                        (*object_weight)[key] = p_obj;
                    }
                }
            }
        }
        cout << "occupancy_map is " << occupancy_map->size() << endl;
        cout << "edge is " << edge->points.size() << endl;
        cout << "object_map is " << object_weight->size() << endl;
        // Calculate the maximum number of rays according to BBX, the maximum number of rays is the size of the
        //  surface area * volume, used to allocate pointer memory
        double pre_line_point = 2.0 * map_size / octomap_resolution;
        long long superficial = ceil(5.0 * pre_line_point * pre_line_point);
        long long volume = ceil(pre_line_point * pre_line_point * pre_line_point);
        max_num_of_rays = superficial * volume;
        rays_info = new Ray_Information*[max_num_of_rays];
        cout << "full rays num is " << max_num_of_rays << endl;
        // Calculate the eight vertices of BBX for delineating the ray range
        vector<Eigen::Vector4d> convex_3d;
        double x1 = view_space->object_center_world(0) - map_size;
        double x2 = view_space->object_center_world(0) + map_size;
        double y1 = view_space->object_center_world(1) - map_size;
        double y2 = view_space->object_center_world(1) + map_size;
        double z1 = view_space->object_center_world(2) - map_size;
        double z2 = view_space->object_center_world(2) + map_size;
        convex_3d.emplace_back(x1, y1, z1, 1);
        convex_3d.emplace_back(x1, y2, z1, 1);
        convex_3d.emplace_back(x2, y1, z1, 1);
        convex_3d.emplace_back(x2, y2, z1, 1);
        convex_3d.emplace_back(x1, y1, z2, 1);
        convex_3d.emplace_back(x1, y2, z2, 1);
        convex_3d.emplace_back(x2, y1, z2, 1);
        convex_3d.emplace_back(x2, y2, z2, 1);
        voxel_information->convex = convex_3d;
        // Ray generators for assigning viewpoints
        thread** ray_caster = new thread*[view_space->views.size()];
        // Initial subscripts for rays start from 0
        ray_num = 0;
        for(int i = 0; i < view_space->views.size(); i++)
        {
            // Threads that divide into rays for this viewpoint
            ray_caster[i] = new thread(ray_cast_thread_process,
                                       &ray_num,
                                       rays_info,
                                       rays_map,
                                       views_to_rays_map,
                                       rays_to_viwes_map,
                                       octo_model,
                                       voxel_information,
                                       view_space,
                                       &color_intrinsics,
                                       i);
        }
        // Wait for each viewpoint ray generator to complete its calculation
        for(int i = 0; i < view_space->views.size(); i++)
        {
            (*ray_caster[i]).join();
        }
        cout << "ray_num is " << ray_num << endl;
        cout << "All views' rays generated with executed time " << clock() - now_time << " ms. Startring compution."
             << endl;
        // Allocate a thread to each ray
        now_time = clock();
        thread** rays_process = new thread*[ray_num];
        for(int i = 0; i < ray_num; i++)
        {
            rays_process[i] = new thread(ray_information_thread_process,
                                         i,
                                         rays_info,
                                         rays_map,
                                         occupancy_map,
                                         object_weight,
                                         octo_model,
                                         voxel_information,
                                         view_space,
                                         method);
        }
        // Waiting for the ray calculation to be completed
        for(int i = 0; i < ray_num; i++)
        {
            (*rays_process[i]).join();
        }
        auto cost_time = clock() - now_time;
        cout << "All rays' threads over with executed time " << cost_time << " ms." << endl;
        share_data->access_directory(share_data->save_path + "/run_time");
        ofstream fout(share_data->save_path + "/run_time/IG" + to_string(view_space->id) + ".txt");
        fout << cost_time << endl;
        // Information counters for assigning viewpoints
        now_time = clock();
        thread** view_gain = new thread*[view_space->views.size()];
        for(int i = 0; i < view_space->views.size(); i++)
        {
            // Threads that distribute information statistics for this viewpoint
            view_gain[i] = new thread(information_gain_thread_process, rays_info, views_to_rays_map, view_space, i);
        }
        // Wait for the information on each viewpoint to be counted
        for(int i = 0; i < view_space->views.size(); i++)
        {
            (*view_gain[i]).join();
        }
        cout << "All views' gain threads over with executed time " << clock() - now_time << " ms." << endl;
    }

    void update(Share_Data* share_data, View_Space* view_space, int iterations)
    {
        // Update internal data
        auto now_time = clock();
        double map_size = view_space->predicted_size;
        // Note that the viewpoints need to be sorted by id to create the mapping
        sort(view_space->views.begin(), view_space->views.end(), view_id_compare);
        // Re-recording the octree
        octo_model = share_data->octo_model;
        octomap_resolution = share_data->octomap_resolution;
        alpha = 0.1 / octomap_resolution;
        voxel_information->octomap_resolution = octomap_resolution;
        voxel_information->skip_coefficient = share_data->skip_coefficient;
        // Clear viewpoint information
        for(int i = 0; i < view_space->views.size(); i++)
        {
            view_space->views[i].information_gain = 0;
            view_space->views[i].voxel_num = 0;
        }
        // Avoid duplicate searches
        delete occupancy_map;
        occupancy_map = new unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>();
        delete object_weight;
        object_weight = new unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>();
        // Update frontier
        vector<octomap::point3d> points;
        pcl::PointCloud<pcl::PointXYZ>::Ptr edge(new pcl::PointCloud<pcl::PointXYZ>);
        for(octomap::ColorOcTree::leaf_iterator it = octo_model->begin_leafs(), end = octo_model->end_leafs();
            it != end;
            ++it)
        {
            double occupancy = (*it).getOccupancy();
            (*occupancy_map)[it.getKey()] = occupancy;
            if(voxel_information->is_unknown(occupancy))
            {
                auto coordinate = it.getCoordinate();
                if(coordinate.x() >= view_space->object_center_world(0) - map_size &&
                   coordinate.x() <= view_space->object_center_world(0) + map_size &&
                   coordinate.y() >= view_space->object_center_world(1) - map_size &&
                   coordinate.y() <= view_space->object_center_world(1) + map_size &&
                   coordinate.z() >= view_space->object_center_world(2) - map_size &&
                   coordinate.z() <= view_space->object_center_world(2) + map_size)
                {
                    points.push_back(coordinate);
                    if(frontier_check(coordinate, octo_model, voxel_information, octomap_resolution) == 2)
                        edge->points.emplace_back(coordinate.x(), coordinate.y(), coordinate.z());
                }
            }
        }
        edge_cnt = edge->points.size();
        if(edge_cnt > pre_edge_cnt)
            pre_edge_cnt = 0x3f3f3f3f;
        if(edge->points.size() != 0)
        {
            // Calculating frontier
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(edge);
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            for(int i = 0; i < points.size(); i++)
            {
                octomap::OcTreeKey key;
                bool key_have = octo_model->coordToKeyChecked(points[i], key);
                if(key_have)
                {
                    pcl::PointXYZ searchPoint(points[i].x(), points[i].y(), points[i].z());
                    int num = kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
                    if(num > 0)
                    {
                        double p_obj = 1;
                        for(int j = 0; j < pointIdxNKNSearch.size(); j++)
                        {
                            p_obj *= distance_function(pointNKNSquaredDistance[j], alpha);
                        }
                        (*object_weight)[key] = p_obj;
                    }
                }
            }
        }
        cout << "edge is " << edge->points.size() << endl;
        cout << "object_map is " << object_weight->size() << endl;
        cout << "occupancy_map is " << occupancy_map->size() << endl;
        cout << "frontier updated with executed time " << clock() - now_time << " ms." << endl;
        // Detects if re-generation
        now_time = clock();
        bool regenerate = false;
        if(view_space->object_changed)
        {
            regenerate = true;
        }
        // Update data structure if regenerated
        if(regenerate)
        {
            // Recalculation of the maximum number of rays, starting from 0
            double pre_line_point = 2.0 * map_size / octomap_resolution;
            long long superficial = ceil(5.0 * pre_line_point * pre_line_point);
            long long volume = ceil(pre_line_point * pre_line_point * pre_line_point);
            max_num_of_rays = superficial * volume;
            delete[] rays_info;
            rays_info = new Ray_Information*[max_num_of_rays];
            cout << "full rays num is " << max_num_of_rays << endl;
            ray_num = 0;
            delete views_to_rays_map;
            views_to_rays_map = new unordered_map<int, vector<int>>();
            delete rays_to_viwes_map;
            rays_to_viwes_map = new unordered_map<int, vector<int>>();
            delete rays_map;
            rays_map = new unordered_map<Ray, int, Ray_Hash>();

            // Calculate the eight vertices of BBX for delineating the ray range
            vector<Eigen::Vector4d> convex_3d;
            double x1 = view_space->object_center_world(0) - map_size;
            double x2 = view_space->object_center_world(0) + map_size;
            double y1 = view_space->object_center_world(1) - map_size;
            double y2 = view_space->object_center_world(1) + map_size;
            double z1 = view_space->object_center_world(2) - map_size;
            double z2 = view_space->object_center_world(2) + map_size;
            convex_3d.emplace_back(x1, y1, z1, 1);
            convex_3d.emplace_back(x1, y2, z1, 1);
            convex_3d.emplace_back(x2, y1, z1, 1);
            convex_3d.emplace_back(x2, y2, z1, 1);
            convex_3d.emplace_back(x1, y1, z2, 1);
            convex_3d.emplace_back(x1, y2, z2, 1);
            convex_3d.emplace_back(x2, y1, z2, 1);
            convex_3d.emplace_back(x2, y2, z2, 1);
            voxel_information->convex = convex_3d;
            // Ray generators for assigning viewpoints
            thread** ray_caster = new thread*[view_space->views.size()];
            for(int i = 0; i < view_space->views.size(); i++)
            {
                // Threads that divide into rays for this viewpoint
                ray_caster[i] = new thread(ray_cast_thread_process,
                                           &ray_num,
                                           rays_info,
                                           rays_map,
                                           views_to_rays_map,
                                           rays_to_viwes_map,
                                           octo_model,
                                           voxel_information,
                                           view_space,
                                           &color_intrinsics,
                                           i);
            }
            // Wait for each viewpoint ray generator to complete its calculation
            for(int i = 0; i < view_space->views.size(); i++)
            {
                (*ray_caster[i]).join();
            }
            cout << "ray_num is " << ray_num << endl;
            cout << "All views' rays generated with executed time " << clock() - now_time << " ms. Starting computation."
                 << endl;
        }
        // Allocate a thread to each ray
        now_time = clock();
        thread** rays_process = new thread*[ray_num];
        for(int i = 0; i < ray_num; i++)
        {
            rays_info[i]->clear();
            rays_process[i] = new thread(ray_information_thread_process,
                                         i,
                                         rays_info,
                                         rays_map,
                                         occupancy_map,
                                         object_weight,
                                         octo_model,
                                         voxel_information,
                                         view_space,
                                         method);
        }
        // Waiting for the ray calculation to be completed
        for(int i = 0; i < ray_num; i++)
        {
            (*rays_process[i]).join();
        }
        double cost_time = clock() - now_time;
        cout << "All rays' threads over with executed time " << cost_time << " ms." << endl;
        share_data->access_directory(share_data->save_path + "/run_time");
        ofstream fout(share_data->save_path + "/run_time/IG" + to_string(view_space->id) + ".txt");
        fout << cost_time << endl;
        now_time = clock();
        thread** view_gain = new thread*[view_space->views.size()];
        for(int i = 0; i < view_space->views.size(); i++)
        {
            // Threads that distribute information statistics for this viewpoint
            view_gain[i] = new thread(information_gain_thread_process, rays_info, views_to_rays_map, view_space, i);
        }
        // Wait for the information on each viewpoint to be counted
        for(int i = 0; i < view_space->views.size(); i++)
        {
            (*view_gain[i]).join();
        }
        cout << "All views' gain threads over with executed time " << clock() - now_time << " ms." << endl;
    }
};

void information_gain_thread_process(Ray_Information** rays_info,
                                     unordered_map<int, vector<int>>* views_to_rays_map,
                                     View_Space* view_space,
                                     int pos)
{
    // Information about each relevant ray of the viewpoint is added to the viewpoint
    for(auto it = (*views_to_rays_map)[pos].begin(); it != (*views_to_rays_map)[pos].end(); it++)
    {
        view_space->views[pos].information_gain += rays_info[*it]->information_gain;
        view_space->views[pos].voxel_num += rays_info[*it]->voxel_num;
    }
}

void ray_cast_thread_process(int* ray_num,
                             Ray_Information** rays_info,
                             unordered_map<Ray, int, Ray_Hash>* rays_map,
                             unordered_map<int, vector<int>>* views_to_rays_map,
                             unordered_map<int, vector<int>>* rays_to_viwes_map,
                             octomap::ColorOcTree* octo_model,
                             Voxel_Information* voxel_information,
                             View_Space* view_space,
                             rs2_intrinsics* color_intrinsics,
                             int pos)
{
    // Get a viewpoint pose
    view_space->views[pos].get_next_camera_pos(view_space->now_camera_pose_world, view_space->object_center_world);
    Eigen::Matrix4d view_pose_world =
      (view_space->now_camera_pose_world * view_space->views[pos].pose.inverse()).eval();
    // Projection of the 3D object BBX onto the convex pack area of the picture according to the viewpoint pose
    double skip_coefficient = voxel_information->skip_coefficient;
    // Control the ray traversal according to the accessible voxels, noting the interval jump parameter
    int pixel_interval = color_intrinsics->width;
    double max_range = 6.0 * view_space->predicted_size;
    vector<cv::Point2f> hull;
    hull = get_convex_on_image(voxel_information->convex,
                               view_pose_world,
                               *color_intrinsics,
                               pixel_interval,
                               max_range,
                               voxel_information->octomap_resolution);
    // if (hull.size() != 4 && hull.size() != 5 && hull.size() != 6) cout << "hull wrong with size " << hull.size() <<
    // endl; Calculating the enclosing box of a convex pack
    vector<int> boundary;
    boundary = get_xmax_xmin_ymax_ymin_in_hull(hull, *color_intrinsics);
    int xmax = boundary[0];
    int xmin = boundary[1];
    int ymax = boundary[2];
    int ymin = boundary[3];
    // cout << xmax << " " << xmin << " " << ymax << " " << ymin << " ," << pixel_interval <<endl;
    // Intermediate data structures
    vector<Ray*> rays;
    // int num = 0;
    // Check the key of the viewpoint
    octomap::OcTreeKey key_origin;
    bool key_origin_have = octo_model->coordToKeyChecked(view_space->views[pos].init_pos(0),
                                                         view_space->views[pos].init_pos(1),
                                                         view_space->views[pos].init_pos(2),
                                                         key_origin);
    if(key_origin_have)
    {
        octomap::point3d origin = octo_model->keyToCoord(key_origin);
        // Traversing the wrap-around box
        // srand(pos);
        // int rr = rand() % 256, gg = rand() % 256, bb = rand() % 256;
        for(int x = xmin; x <= xmax; x += (int)(pixel_interval * skip_coefficient))
            for(int y = ymin; y <= ymax; y += (int)(pixel_interval * skip_coefficient))
            {
                // num++;
                cv::Point2f pixel(x, y);
                // Check if it is inside the convex bale area
                if(!is_pixel_in_convex(hull, pixel))
                    continue;
                // Reverse projection to find the end point
                octomap::point3d end = project_pixel_to_ray_end(x, y, *color_intrinsics, view_pose_world, max_range);
                // Show it
                // view_space->viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(origin(0), origin(1), origin(2)),
                // pcl::PointXYZ(end(0), end(1), end(2)), rr, gg, bb, "line" + to_string(pos) + "-" + to_string(x) + "-"
                // + to_string(y));
                octomap::OcTreeKey key_end;
                octomap::point3d direction = end - origin;
                octomap::point3d end_point;
                // Crossing unknown areas and finding the end
                bool found_end_point = octo_model->castRay(origin, direction, end_point, true, max_range);
                if(!found_end_point)
                {
                    // End point not found, set end point as maximum distance
                    end_point =
                      origin +
                      direction.normalized() *
                        max_range; // use max range instead of stopping at the unknown       found_endpoint = true;
                }
                // Check that the end is within the map limits and hits BBX
                bool key_end_have = octo_model->coordToKeyChecked(end_point, key_end);
                if(key_end_have)
                {
                    // Generation of rays
                    octomap::KeyRay* ray_set = new octomap::KeyRay();
                    // Get the ray array, without the end node
                    bool point_on_ray_getted = octo_model->computeRayKeys(origin, end_point, *ray_set);
                    if(!point_on_ray_getted)
                        cout << "Warning. ray cast with wrong max_range." << endl;
                    if(ray_set->size() > 950)
                        cout << ray_set->size() << " rewrite the vector size in octreekey.h." << endl;
                    // Putting the end point into the ray group
                    ray_set->addKey(key_end);
                    // The first non-empty node is used as the start of the ray and the last non-empty element from the
                    // tail is used as the end of the ray
                    auto last = ray_set->end();
                    last--;
                    while(last != ray_set->begin() && (octo_model->search(*last) == nullptr))
                        last--;
                    // Dichotomous first non-empty element
                    auto l = ray_set->begin();
                    auto r = last;
                    auto mid = l + (r - l) / 2;
                    while(mid != r)
                    {
                        if(octo_model->search(*mid) != nullptr)
                            r = mid;
                        else
                            l = mid + 1;
                        mid = l + (r - l) / 2;
                    }
                    auto first = mid;
                    while(first != ray_set->end() &&
                          (octo_model->keyToCoord(*first).x() <
                             view_space->object_center_world(0) - view_space->predicted_size ||
                           octo_model->keyToCoord(*first).x() >
                             view_space->object_center_world(0) + view_space->predicted_size ||
                           octo_model->keyToCoord(*first).y() <
                             view_space->object_center_world(1) - view_space->predicted_size ||
                           octo_model->keyToCoord(*first).y() >
                             view_space->object_center_world(1) + view_space->predicted_size ||
                           octo_model->keyToCoord(*first).z() <
                             view_space->object_center_world(2) - view_space->predicted_size ||
                           octo_model->keyToCoord(*first).z() >
                             view_space->object_center_world(2) + view_space->predicted_size))
                        first++;
                    // If there are no non-empty elements, just discard the ray
                    if(last - first < 0)
                    {
                        delete ray_set;
                        continue;
                    }
                    auto stop = last;
                    stop++;
                    // Show it
                    // while (octo_model->keyToCoord(*first).x() < view_space->object_center_world(0) -
                    // view_space->predicted_size || octo_model->keyToCoord(*first).x() >
                    // view_space->object_center_world(0) + view_space->predicted_size
                    //	|| octo_model->keyToCoord(*first).y() < view_space->object_center_world(1) -
                    //view_space->predicted_size || octo_model->keyToCoord(*first).y() >
                    //view_space->object_center_world(1) + view_space->predicted_size
                    //	|| octo_model->keyToCoord(*first).z() < min(view_space->height_of_ground,
                    //view_space->object_center_world(2) - view_space->predicted_size) ||
                    //octo_model->keyToCoord(*first).z() > view_space->object_center_world(2) +
                    //view_space->predicted_size) first++; octomap::point3d ss = octo_model->keyToCoord(*first);
                    // octomap::point3d ee = octo_model->keyToCoord(*last);
                    // view_space->viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(ss(0), ss(1), ss(2)),
                    // pcl::PointXYZ(ee(0), ee(1), ee(2)), rr, gg, bb, "line" + to_string(pos) + "-" + to_string(x) +
                    // "-" + to_string(y)); Add the ray to the set of viewpoints, the first element with the last
                    // element key+array+head+tail
                    Ray* ray = new Ray(*first, *last, ray_set, first, stop);
                    rays.push_back(ray);
                }
            }
    }
    else
    {
        cout << pos << "th view out of map.check." << endl;
    }
    // cout << "rays " << rays.size() <<" num "<<num<< endl;
    // Array of subscripts for this viewpoint ray
    vector<int> ray_ids;
    ray_ids.resize(rays.size());
    // Note that common data structures should be locked
    voxel_information->mutex_rays.lock();
    // Get the position of the current ray
    int ray_id = (*ray_num);
    for(int i = 0; i < rays.size(); i++)
    {
        // For these rays, hash queries to see if there are duplicates
        auto hash_this_ray = rays_map->find(*rays[i]);
        // If there are no duplicates, save the ray
        if(hash_this_ray == rays_map->end())
        {
            (*rays_map)[*rays[i]] = ray_id;
            ray_ids[i] = ray_id;
            // Creation ray calculation class
            rays_info[ray_id] = new Ray_Information(rays[i]);
            vector<int> view_ids;
            view_ids.push_back(pos);
            (*rays_to_viwes_map)[ray_id] = view_ids;
            ray_id++;
        }
        // If there is a duplicate, it means that other viewpoints are also counted to that ray, so put the
        // corresponding id into the subscript array
        else
        {
            ray_ids[i] = hash_this_ray->second;
            delete rays[i]->ray_set;
            // Rays already recorded in other viewpoints, put in the record of this viewpoint
            vector<int> view_ids = (*rays_to_viwes_map)[ray_ids[i]];
            view_ids.push_back(pos);
            (*rays_to_viwes_map)[ray_ids[i]] = view_ids;
        }
    }
    // Updating the number of rays
    (*ray_num) = ray_id;
    // Update the ray array for the viewpoint mapping
    (*views_to_rays_map)[pos] = ray_ids;
    // Release the lock
    voxel_information->mutex_rays.unlock();
}

void ray_information_thread_process(
  int ray_id,
  Ray_Information** rays_info,
  unordered_map<Ray, int, Ray_Hash>* rays_map,
  unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>* occupancy_map,
  unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash>* object_weight,
  octomap::ColorOcTree* octo_model,
  Voxel_Information* voxel_information,
  View_Space* view_space,
  short method)
{
    // Since it is checked, the first node is a non-empty node
    auto first = rays_info[ray_id]->ray->start;
    auto last = rays_info[ray_id]->ray->stop;
    last--;
    for(auto it = rays_info[ray_id]->ray->start; it != rays_info[ray_id]->ray->stop; ++it)
    {
        // Look up the key from the hash table
        auto hash_this_key = (*occupancy_map).find(*it);
        // Next if you can't find a node
        if(hash_this_key == (*occupancy_map).end())
        {
            if(method == RSE && it == last)
                rays_info[ray_id]->information_gain = 0;
            continue;
        }
        // Read node probability values
        double occupancy = hash_this_key->second;
        // Check to see if the current node is occupied
        bool voxel_occupied = voxel_information->is_occupied(occupancy);
        // Check to see if the node is unknown
        bool voxel_unknown = voxel_information->is_unknown(occupancy);
        // Read the node for the surface rate of the object
        double on_object = voxel_information->voxel_object(*it, object_weight);
        // If it is occupied, it is the last node
        if(voxel_occupied)
            last = it;
        // If free, the initial node is to be updated
        if(it == first && (!voxel_unknown && !voxel_occupied))
            first = it;
        // Determine if it is the last node
        bool is_end = (it == last);
        // Statistical information entropy
        rays_info[ray_id]->information_gain = information_function(method,
                                                                   rays_info[ray_id]->information_gain,
                                                                   voxel_information->entropy(occupancy),
                                                                   rays_info[ray_id]->visible,
                                                                   voxel_unknown,
                                                                   rays_info[ray_id]->previous_voxel_unknown,
                                                                   is_end,
                                                                   voxel_occupied,
                                                                   on_object,
                                                                   rays_info[ray_id]->object_visible);
        rays_info[ray_id]->object_visible *= (1 - on_object);
        if(method == OursIG)
            rays_info[ray_id]->visible *= voxel_information->get_voxel_visible(occupancy);
        else
            rays_info[ray_id]->visible *= occupancy;
        rays_info[ray_id]->voxel_num++;
        // Exit if it's the end
        if(is_end)
            break;
    }
    while(last - first < -1)
        first--;
    last++;
    // Update stop to one iterator after the last node
    rays_info[ray_id]->ray->stop = last;
    // Update start to the first iterator
    rays_info[ray_id]->ray->start = first;
}

inline double information_function(short& method,
                                   double& ray_information,
                                   double voxel_information,
                                   double& visible,
                                   bool& is_unknown,
                                   bool& previous_voxel_unknown,
                                   bool& is_endpoint,
                                   bool& is_occupied,
                                   double& object,
                                   double& object_visible)
{
    double final_information = 0;
    switch(method)
    {
        case OursIG:
            if(is_unknown)
            {
                final_information = ray_information + object * visible * voxel_information;
            }
            else
            {
                final_information = ray_information;
            }
            break;
        case OA: final_information = ray_information + visible * voxel_information; break;
        case UV:
            if(is_unknown)
                final_information = ray_information + visible * voxel_information;
            else
                final_information = ray_information;
            break;
        case RSE:
            if(is_endpoint)
            {
                if(previous_voxel_unknown)
                {
                    if(is_occupied)
                        final_information = ray_information + visible * voxel_information;
                    else
                        final_information = 0;
                }
                else
                    final_information = 0;
            }
            else
            {
                if(is_unknown)
                {
                    previous_voxel_unknown = true;
                    final_information = ray_information + visible * voxel_information;
                }
                else
                {
                    previous_voxel_unknown = false;
                    final_information = 0;
                }
            }
            break;
        case APORA:
            if(is_unknown)
            {
                final_information = ray_information + object * object_visible * voxel_information;
            }
            else
            {
                final_information = ray_information;
            }
            break;
        case Kr:
            if(is_endpoint)
            {
                if(is_occupied)
                    final_information = ray_information + voxel_information;
                else
                    final_information = 0;
            }
            else
                final_information = ray_information + voxel_information;
            break;
    }
    return final_information;
}

inline int frontier_check(octomap::point3d node,
                          octomap::ColorOcTree* octo_model,
                          Voxel_Information* voxel_information,
                          double octomap_resolution)
{
    int free_cnt = 0;
    int occupied_cnt = 0;
    for(int i = -1; i <= 1; i++)
        for(int j = -1; j <= 1; j++)
            for(int k = -1; k <= 1; k++)
            {
                if(i == 0 && j == 0 && k == 0)
                    continue;
                double x = node.x() + i * octomap_resolution;
                double y = node.y() + j * octomap_resolution;
                double z = node.z() + k * octomap_resolution;
                octomap::point3d neighbour(x, y, z);
                octomap::OcTreeKey neighbour_key;
                bool neighbour_key_have = octo_model->coordToKeyChecked(neighbour, neighbour_key);
                if(neighbour_key_have)
                {
                    octomap::ColorOcTreeNode* neighbour_voxel = octo_model->search(neighbour_key);
                    if(neighbour_voxel != nullptr)
                    {
                        free_cnt += voxel_information->voxel_free(neighbour_voxel) == true ? 1 : 0;
                        occupied_cnt += voxel_information->voxel_occupied(neighbour_voxel) == true ? 1 : 0;
                    }
                }
            }
    // edge
    if(free_cnt >= 1 && occupied_cnt >= 1)
        return 2;
    // Boundaries
    if(free_cnt >= 1)
        return 1;
    // Nothing
    return 0;
}

inline double distance_function(double distance, double alpha) { return exp(-pow2(alpha) * distance); }

/*
solving by Max Flow
*/

void adjacency_list_thread_process(int ray_id,
                                   int* nz,
                                   int ray_index_shift,
                                   int voxel_index_shift,
                                   unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>* voxel_id_map,
                                   vector<vector<pair<int, double>>>* bipartite_list,
                                   View_Space* view_space,
                                   Views_Information* views_information,
                                   Voxel_Information* voxel_information,
                                   Share_Data* share_data);

class views_voxels_MF
{
  public:
    // Number of points on three sides, number of viewpoints nx, number of rays ny,number of voxels nz
    int nx, ny, nz;
    // Adjacency table
    vector<vector<pair<int, double>>>* bipartite_list;
    View_Space* view_space;
    Views_Information* views_information;
    Voxel_Information* voxel_information;
    Share_Data* share_data;
    // Voxel subscript
    unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>* voxel_id_map;
    MCMF* mcmf;
    vector<int> view_id_set;

    void solve()
    {
        auto now_time = clock();
        view_id_set = mcmf->work(*bipartite_list);
        double cost_time = clock() - now_time;
        cout << "flow network solved with executed time " << cost_time << " ms." << endl;
        cout << view_id_set.size() << " views got by max flow." << endl;
        share_data->access_directory(share_data->save_path + "/run_time");
        ofstream fout(share_data->save_path + "/run_time/MF" + to_string(view_space->id) + ".txt");
        fout << cost_time << '\t' << view_id_set.size() << endl;
    }

    vector<int> get_view_id_set() const { return view_id_set; }

    views_voxels_MF(int _nx,
                    View_Space* _view_space,
                    Views_Information* _views_information,
                    Voxel_Information* _voxel_information,
                    Share_Data* _share_data)
    {
        auto now_time = clock();
        view_space = _view_space;
        views_information = _views_information;
        voxel_information = _voxel_information;
        share_data = _share_data;
        // Viewpoints are sorted by id and a trilateration adjacency table is created
        sort(view_space->views.begin(), view_space->views.end(), view_id_compare);
        nx = _nx;
        ny = views_information->ray_num;
        bipartite_list = new vector<vector<pair<int, double>>>;
        bipartite_list->resize(nx + ny + share_data->voxels_in_BBX);
        // Create a table of ids for voxels
        voxel_id_map = new unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>;
        // Parallel traversal of the voxels on each ray to the corresponding viewpoint
        nz = 0;
        thread** adjacency_list_process = new thread*[views_information->ray_num];
        for(int i = 0; i < views_information->ray_num; i++)
        {
            adjacency_list_process[i] = new thread(adjacency_list_thread_process,
                                                   i,
                                                   &nz,
                                                   nx,
                                                   nx + ny,
                                                   voxel_id_map,
                                                   bipartite_list,
                                                   view_space,
                                                   views_information,
                                                   voxel_information,
                                                   share_data);
        }
        for(int i = 0; i < views_information->ray_num; i++)
        {
            (*adjacency_list_process[i]).join();
        }
        // Output the exact figure size
        if(nz != voxel_id_map->size())
            cout << "node_z wrong." << endl;
        std::size_t num_of_all_edge = 0;
        std::size_t num_of_view_edge = 0;
        for(int i = 0; i < bipartite_list->size(); i++)
        {
            num_of_all_edge += (*bipartite_list)[i].size();
            if(i > nx && i < nx + ny)
                num_of_view_edge += (*bipartite_list)[i].size();
        }
        cout << "Full edge is " << num_of_all_edge << ". View edge(in) is " << num_of_view_edge
             << ". Voxel edge(out) is " << num_of_all_edge - num_of_view_edge << "." << endl;
        cout << "adjacency list with interested voxels num " << ny << " got with executed time "
             << clock() - now_time << " ms." << endl;
        mcmf = new MCMF();
    }

    ~views_voxels_MF()
    {
        delete bipartite_list;
        delete voxel_id_map;
        delete mcmf;
    }
};

void adjacency_list_thread_process(int ray_id,
                                   int* nz,
                                   int ray_index_shift,
                                   int voxel_index_shift,
                                   unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>* voxel_id_map,
                                   vector<vector<pair<int, double>>>* bipartite_list,
                                   View_Space* view_space,
                                   Views_Information* views_information,
                                   Voxel_Information* voxel_information,
                                   Share_Data* share_data)
{
    // Which viewpoints the ray is seen by, added to the diagram
    vector<int> views_id = (*views_information->rays_to_viwes_map)[ray_id];
    for(int i = 0; i < views_id.size(); i++)
        (*bipartite_list)[ray_id + ray_index_shift].push_back(make_pair(views_id[i], 0.0));
    // Retain only voxels of interest
    double visible = 1.0;
    auto first = views_information->rays_info[ray_id]->ray->start;
    auto last = views_information->rays_info[ray_id]->ray->stop;
    for(auto it = views_information->rays_info[ray_id]->ray->start;
        it != views_information->rays_info[ray_id]->ray->stop;
        ++it)
    {
        // Look up the key from the hash table
        auto hash_this_key = (*views_information->occupancy_map).find(*it);
        // Next if you can't find a node
        if(hash_this_key == (*views_information->occupancy_map).end())
            continue;
        // Read node probability values
        double occupancy = hash_this_key->second;
        // Read the node for the surface rate of the object
        double on_object = voxel_information->voxel_object(*it, views_information->object_weight);
        // Statistical information entropy
        double information_gain = on_object * visible * voxel_information->entropy(occupancy);
        visible *= voxel_information->get_voxel_visible(occupancy);
        if(information_gain > share_data->interesting_threshold)
        {
            octomap::OcTreeKey node_y = *it;
            int voxel_id;
            voxel_information->mutex_rays.lock();
            auto hash_this_node = voxel_id_map->find(node_y);
            // If not recorded, it is considered a new voxel
            if(hash_this_node == voxel_id_map->end())
            {
                voxel_id = (*nz) + voxel_index_shift;
                (*voxel_id_map)[node_y] = voxel_id;
                (*nz)++;
            }
            else
            {
                voxel_id = hash_this_node->second;
            }
            voxel_information->mutex_rays.unlock();
            // For each viewpoint, count the id and value of the voxel
            for(int i = 0; i < views_id.size(); i++)
            {
                (*voxel_information->mutex_voxels[voxel_id - voxel_index_shift]).lock();
                (*bipartite_list)[voxel_id].push_back(make_pair(ray_id + ray_index_shift, information_gain));
                (*voxel_information->mutex_voxels[voxel_id - voxel_index_shift]).unlock();
            }
        }
    }
}
