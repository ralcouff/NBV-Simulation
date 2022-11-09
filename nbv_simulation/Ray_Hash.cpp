#include "Ray_Hash.h"

size_t Ray_Hash::operator()(const Ray &ray) const {
    return octomap::OcTreeKey::KeyHash()(ray.origin) ^ octomap::OcTreeKey::KeyHash()(ray.end);
}
