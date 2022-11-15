# Remaining functions

## Share_Data.hpp
- [x] enum `rs2_distortion` &rarr; `utils.cpp`
- [x] struct `rs2_intrinsics` &rarr; `utils.cpp`
- [x] `rs2_project_point_to_pixel` &rarr; `utils.cpp`
- [x] `rs2_deproject_pixel_to_point` &rarr; `utils.cpp`
- [x] class `Share_Data` &rarr; `Share_Data.hpp/.cpp`
- [x] `pow2` &rarr; `utils.cpp`

## Information.hpp
- [x] class `Ray` &rarr; `Ray.hpp/.cpp`
- [x] class `Ray_Hash` &rarr; `Ray_Hash.hpp/.cpp`
- [x] class `Ray_Information` &rarr; `Ray_Information.hpp/.cpp`
- [x] `get_xmax_xmin_ymax_ymin_in_hull` &rarr; `utils.cpp`
- [x] `is_pixel_in_convex` &rarr; `utils.cpp`
- [x] `get_convex_on_image` &rarr; `utils.cpp`
- [x] `project_pixel_to_ray_end` &rarr; `utils.cpp`
- [x] `frontier_check` &rarr; `Information.h/.cpp`
- [x] `distance_function` &rarr; `utils.cpp`
- [x] class `Views_Information` &rarr; `Views_Information.hpp/.cpp`
- [x] `information_gain_thread_process` &rarr; `Information.h/.cpp`
- [x] `ray_expand_thread_process` &rarr; `Information.h/.cpp`
- [x] `ray_cast_thread_process` &rarr; `Information.h/.cpp`
- [x] `ray_information_thread_process` &rarr; `Information.h/.cpp`
- [x] `adjacency_thread_process` &rarr; `Information.h/.cpp`
- [x] `adjacency_list_thread_process` &rarr; `Information.h/.cpp`
- [x] `information_function` &rarr; `Information.h/.cpp`
- [x] class `views_voxels_MF` &rarr; `views_voxels_MF.hpp/.cpp`
- [x] class `MCMF` &rarr; `MCMF.hpp/.cpp`

## View_Space.hpp
- [x] class `Voxel_Information` &rarr; `Voxel_Information.hpp/.cpp`
- [x] `get_random_coordinates` &rarr; `utils.cpp`
- [x] `add_trajectory_to_cloud`&rarr; `View_Space.hpp/.cpp`
- [x] `delete_trajectory_in_cloud`&rarr; `View_Space.hpp/.cpp`
- [x] class `View` &rarr; `View.hpp/.cpp`
- [x] `view_id_compare` &rarr; `View.cpp`
- [x] `view_utility_compare` &rarr; `View.cpp`
- [x] class `View_Space` &rarr; `View_Space.hpp/.cpp`

## Main.cpp
- [x] `percept_thread_process` &rarr; `Perception_3D.hpp`
- [x] class `Perception_3D` &rarr; `Perception_3D.hpp/.cpp`
- [x] `save_cloud_mid` &rarr; `NBV_Planner.hpp/.cpp`
- [x] `create_view_space` &rarr; `NBV_Planner.hpp/.cpp`
- [x] `create_views_information` &rarr; `NBV_Planner.hpp/.cpp`
- [x] `move_robot` &rarr; `NBV_Planner.hpp/.cpp`
- [x] `show_cloud` &rarr; `NBV_Planner.hpp/.cpp`
- [x] class `NBV_Planner` &rarr; `NBV_Planner.hpp/.cpp`
- [x] `get_command` &rarr; `main.cpp`
- [x] `get_run` &rarr; `main.cpp`
- [x] `main`