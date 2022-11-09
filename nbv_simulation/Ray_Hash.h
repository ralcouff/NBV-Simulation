#ifndef NBV_SIMULATION_RAY_HASH_H
#define NBV_SIMULATION_RAY_HASH_H

#pragma once

#include "Ray.h"

/**
 * Creates a Hash for the Ray
 * It will be used to create unordered maps
 */
class Ray_Hash {

public:
    size_t operator()(const Ray& ray) const;
};


#endif //NBV_SIMULATION_RAY_HASH_H
