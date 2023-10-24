import os

basename = "/home/alcoufr/dev/NBV_base/NBV-Simulation_1"
# "Elephant_zun_bakedUV", "ET26_L_MK1", "pizza_bakedUV", "SculptureShaman", "bronzeCat"
model_path = "Elephant_zun_bakedUV"

config_file = os.path.join(basename, "DefaultConfiguration.yaml")
m_file = os.path.join(basename, "3d_models/TextureQualityMesh/Decimated", model_path)
model_qlt_path = os.path.join(basename, "3d_models/TextureQualityMesh/Decimated", model_path + ".qlt")
method = 103
reconstruction_iter = 25
reconstruction_method = 1
save_folder = os.path.join(basename, f"/home/alcoufr/Documents/Papiers/ROBOVIS/ROBOVIS_{model_path}_test_101_v2_mean_plane/")
init_cam_file = os.path.join(basename, "ROBOVIS_Elephant_zun_bakedUV_init_views_0", "results.csv")


command = f"./cmake-build-debug-nbv-sim1/nbv_simulation {config_file} {m_file} {model_qlt_path} {method} {reconstruction_iter} {reconstruction_method} {save_folder} {init_cam_file} 2023-04-05"
print(command)
os.system(command)
