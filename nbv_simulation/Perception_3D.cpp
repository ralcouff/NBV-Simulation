#include "Perception_3D.h"

Perception_3D::Perception_3D(Share_Data *_share_data) {
    share_data = _share_data;
    octo_model = share_data->octo_model;
    iterations = 0;
}

bool Perception_3D::percept(View *current_best_view) {
    share_data->valid_clouds++;
    share_data->clouds.push_back(share_data->working_cloud);
    iterations++;
    return true;
}
