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
- [ ] `get_xmax_xmin_ymax_ymin_in_hull` &rarr; `utils.cpp`
- [ ] `is_pixel_in_convex` &rarr; `utils.cpp`
- [ ] `get_convex_on_image` &rarr; `utils.cpp`
- [ ] `project_pixel_to_ray_end` &rarr; `utils.cpp`
- [ ] `frontier_check` &rarr; `utils.cpp`
- [ ] `distance_function` &rarr; `utils.cpp`
- [ ] class `Views_Information` &rarr; `Views_Information.hpp/.cpp` Missing `ray_cast_thread_process`, `ray_information_thread_process`, `information_gain_thread_process`
- [ ] `information_gain_thread_process`
- [ ] `ray_expand_thread_process`
- [ ] `ray_cast_thread_process`
- [ ] `ray_information_thread_process`
- [ ] `adjacency_thread_process`
- [ ] `adjacency_list_thread_process`
- [ ] `information_function`
- [ ] class `views_voxels_MF` &rarr; Missing `adjacency_list_thread_process`
- [x] class `MCMF` &rarr; `MCMF.hpp/.cpp`

## View_Space.hpp
- [ ] class `Voxel_Information` &rarr; `Voxel_Information.hpp/.cpp`
- [ ] `get_random_coordinates` &rarr; `utils.cpp`
- [ ] `add_trajectory_to_cloud`
- [ ] `delete_trajectory_in_cloud`
- [ ] class `View` &rarr; `View.hpp/.cpp`
- [ ] `view_id_compare` &rarr; `View.cpp`
- [ ] `view_utility_compare` &rarr; `View.cpp`
- [ ] class `View_Space` &rarr; `View_Space.hpp/.cpp`

## Main.cpp
- [ ] `precept_thread_process` &rarr; `Perception_3D.hpp`
- [ ] class `Perception_3D` &rarr; `Perception_3D.hpp/.cpp`
- [ ] `save_cloud_mid` &rarr; `main.cpp`
- [ ] `create_view_space`
- [ ] `create_views_information`
- [ ] `move_robot`
- [ ] `show_cloud`
- [ ] class `NBV_Planner` &rarr; `NBV_Planner.hpp/.cpp` Missing `move_robot`, `create_view_space`, `create_views_information`
- [ ] `get_command` &rarr; `main.cpp`
- [ ] `get_run` &rarr; `main.cpp`
- [ ] `main`