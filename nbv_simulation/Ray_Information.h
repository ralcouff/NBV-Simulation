#ifndef NBV_SIMULATION_RAY_INFORMATION_H
#define NBV_SIMULATION_RAY_INFORMATION_H

#pragma once

#include "Ray.h"

/**
 * An implementation of the Information contained by a Ray
 */
class Ray_Information {

public:
    Ray* ray;
    double information_gain;
    double visible;
    double object_visible;
    int voxel_num;
    bool previous_voxel_unknown;

    /**
     * Constructor of the Ray_Information class
     * @param _ray A pointer to the concerned Ray
     */
    explicit Ray_Information(Ray* _ray);

    /**
     * Destructor of the Ray_Information
     */
    ~Ray_Information();

    /**
     * Clearing the Ray_Information object, setting the default values
     */
    void clear();

};

#endif //NBV_SIMULATION_RAY_INFORMATION_H
