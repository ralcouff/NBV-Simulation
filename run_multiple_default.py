import os

config_file = "DefaultConfiguration.yaml"
basename = "/home/alcoufr/dev/NBV_base/NBV-Simulation_1"
# model_path = ["Elephant_zun_bakedUV", "ET26_L_MK1", "pizza_bakedUV", "SculptureShaman", "bronzeCat"]
model_path = ["ET26_L_MK1", "pizza_bakedUV", "SculptureShaman", "bronzeCat"]
# model_path = ["Elephant_zun_bakedUV", "ET26_L_MK1"]
reconstruction_iter = 0
reconstruction_method = 0
method = 0
init_cam_file = '_'

for m in model_path:
    m_file = os.path.join(basename, "3d_models/TextureQualityMesh/Decimated", m)
    model_qlt_path = os.path.join(basename, "3d_models/TextureQualityMesh/Decimated", m + ".qlt")
    save_folder = os.path.join(basename, f"ROBOVIS_{m}_init_views_0/")
    command = f"./cmake-build-debug-nbv-sim1/nbv_simulation {config_file} {m_file} {model_qlt_path} {method} {reconstruction_iter} {reconstruction_method} {save_folder} {init_cam_file} 2023-04-05"
    print(command)
    os.system(command)