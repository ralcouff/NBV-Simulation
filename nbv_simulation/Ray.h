#ifndef NBV_SIMULATION_RAY_H
#define NBV_SIMULATION_RAY_H

#pragma once

#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>

/**
 * Implementation of a Ray in our octree.
 *
 * The Ray is categorised by its origin, its end, and the voxels it is passing through.
 */
class Ray {

public:
    /**
     * @brief Constructor of a Ray
     * @param _origin The key to the origin voxel
     * @param _end The key to the end voxel
     * @param _ray_set A pointer to the set containing all the voxels traversed by the Ray - TODO : To check
     * @param _start An iterator on the first element - TODO : To check
     * @param _stop An iterator on the last element - TODO : To check
     */
    Ray(octomap::OcTreeKey _origin,
        octomap::OcTreeKey _end,
        octomap::KeyRay* _ray_set,
        octomap::KeyRay::iterator _start,
        octomap::KeyRay::iterator _stop);

    octomap::OcTreeKey origin;
    octomap::OcTreeKey end;
    octomap::KeyRay* ray_set;
    octomap::KeyRay::iterator start;
    octomap::KeyRay::iterator stop;

};

/**
 * Check the equality of two rays
 * Two rays are considered equals if they have the same origin and the same end
 *
 * @param current The current Ray
 * @param other A pointer to the other Ray
 * @return TRUE if they are equal, FALSE if they're not
 */
bool operator==(Ray const& current, Ray const& other);

#endif //NBV_SIMULATION_RAY_H
