#include "Ray.h"

Ray::Ray(octomap::OcTreeKey _origin, octomap::OcTreeKey _end, octomap::KeyRay *_ray_set,
         octomap::KeyRay::iterator _start, octomap::KeyRay::iterator _stop) {
    origin = _origin;
    end = _end;
    ray_set = _ray_set;
    start = _start;
    stop = _stop;
}

// Two Rays are considered equal if they have the same origin and end
bool operator==( Ray const& current, Ray const& other)
{
    return (current.origin == other.origin) && (current.end == other.end);
}
