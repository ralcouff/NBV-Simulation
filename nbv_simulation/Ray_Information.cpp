#include "Ray_Information.h"

Ray_Information::Ray_Information(Ray *_ray) {
    ray = _ray;
    information_gain = 0;
    visible = 1;
    object_visible = 1;
    previous_voxel_unknown = false;
    voxel_num = 0;
}

Ray_Information::~Ray_Information() {
    delete ray;
}

void Ray_Information::clear() {
    information_gain = 0;
    visible = 1;
    object_visible = 1;
    previous_voxel_unknown = false;
    voxel_num = 0;
}
