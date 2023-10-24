import os

# config_file = "DefaultConfiguration.yaml"
# # methods = [0, 10, 11, 101]
# methods = [11]
# nb_model = 5
# n_rings = [5, 6, 7]
# # n_rings = [5]
# reconstruction_iter = 0
# reconstruction_method = 0
# save_path = "media/alcoufr/These_Remy/results_1909"
# for method in methods:
#     for i in range(nb_model):
#         for n_ring in n_rings:
#             save_folder = os.path.join(save_path, f"{method}_{i}_sphere_qlt_{n_ring}_{reconstruction_iter}")
#             if not os.path.exists(save_folder):
#                 os.makedirs(save_folder)
#             model_path = f"3d_models/{i}_sphere_qlt/{i}_sphere_qlt_{n_ring}"
#             model_qlt_path = f"3d_models/{i}_sphere_qlt/{i}_sphere_qlt_{n_ring}.qlt"
#
#             command = f".cmake-build-debug-nbv-sim1/nbv_simulation {config_file} {model_path} {model_qlt_path} {method} {reconstruction_iter} {reconstruction_method} {save_folder} 2023-04-05"
#             print(command)
#             os.system(command)

config_file = "DefaultConfiguration.yaml"
model_path = "3d_models/hippo_color"
model_qlt_path = "3d_models/hippo_color.qlt"
method = 0
reconstruction_iter = 15
reconstruction_method = 1
save_folder = "/home/alcoufr/dev/NBV_base/NBV-Simulation_1/ROBOVIS_hippo_rec_default_0"

command = f".cmake-build-debug-nbv-sim1/nbv_simulation {config_file} {model_path} {model_qlt_path} {method} {reconstruction_iter} {reconstruction_method} {save_folder} 2023-04-05"
print(command)
os.system(command)