#include "Ray.h"

Ray::Ray(octomap::OcTreeKey _origin, octomap::OcTreeKey _end, octomap::KeyRay *_ray_set,
         octomap::KeyRay::iterator _start, octomap::KeyRay::iterator _stop, octomap::OcTreeKey _end_unknown,
         octomap::KeyRay *_ray_set_unknown, octomap::KeyRay::iterator _start_unknown,
         octomap::KeyRay::iterator _stop_unknown) {
    origin = _origin;
    end = _end;
    ray_set = _ray_set;
    start = _start;
    stop = _stop;
    end_unknown = _end_unknown;
    ray_set_unknown = _ray_set_unknown;
    start_unknown = _start_unknown;
    stop_unknown = _stop_unknown;
}

// Two Rays are considered equal if they have the same origin and end
bool operator==( Ray const& current, Ray const& other)
{
    return (current.origin == other.origin) && (current.end == other.end);
}
