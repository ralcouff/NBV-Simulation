# NBV based on quality

This is a NBV Simulation system for comparative experiments.

It is largely inspired by the NBV-Simulation system : https://github.com/psc0628/NBV-Simulation

I adapted the previous code in order to integrate geometric quality of the 3D reconstructed object.

## Requirements & Dependencies

### Python Submodules

### C++ NBV-Simulator
The NBV simulator has been written in C++ and needs various dependencies:

- [OpenCV (4.4.0)]()
- [PCL (1.9.1)]()
- [Eigen (3.3.9)]()
- [OctoMap (1.9.6)]()

## Installation

## Usage

### Parameters

- **Configuration file**: a YAML file containing all the parameters of the simulation.
- **Path to the 3D model**: the 3D model to use for the simulation (either pcd, obj or ply).
- **Path to the quality file**: the file containing for each vertex of the model a value of quality between 0 and 1.
- **The method to use for the NBV estimation**: an integer corresponding to the method of computation of the NBV.
- **The number of reconstruction iterations**: he number of iteration of the algorithm to use to reconstruct a first
  partial 3D model. This reconstruction is using the method 0 (default).
- **The reconstruction method to use**: an integer corresponding to the reconstruction method to use (default i.e.
  perfect LiDAR or meshroom).
- **Path to the save folder**: the folder that will contain the results of the reconstruction.
- **A string for a unique folder name**: unused

#### Methods for computing the NBV
There are different method implemented in this program to compute the NBV, you can find a complete description of those metrics in the paper ["A Global Max-flow-based
Multi-resolution Next-best-view Method for Reconstruction of 3D Unknown Objects"](https://ieeexplore.ieee.org/document/9635628)

- 0: The metric defined in the paper by Pan et Wei.
- 1: OA
- 2: UV
- 3: RSE
- 4: APORA
- 5: Kr 
- 6: NBV-net

If you want to run with NBV-Net, please check nbvNetPath in file `DefaultConfiguration.yaml`, and run both compiled program of main.cpp
and "python nbv_net/run_test.py {}" (replace {} by your model name) in pytorch environment at the same time.

### Command Line



### Example

```bash
./nbv_simulation ../DefaultConfiguration.yaml ../3d_models/Armadillo ../3d_models/Armadillo.qlt 0 0 0 ../Armadillo_pipeline 2023-07-05
```

### Output

## Limitations

## Acknowledgements

## Questions

# NBV-Simulation

This is a nbv simulation system for comparative experiment, supporting our paper "A Global Max-flow-based
Multi-resolution Next-best-view Method for Reconstruction of 3D Unknown Objects" (doi: 10.1109/LRA.2021.3132430).
Armadillo_example_Ours directory contains an result example of our code with sampled viewsapce (the file
3d_models/Armadillo.txt).

## Installion

For our nbv_simulation c++ code, these libraries need to be installed: opencv 4.4.0, PCL 1.9.1, Eigen 3.3.9, OctoMap
1.9.6.
For nbv_net, please follow https://github.com/irvingvasquez/nbv-net.
We tested our codes on Windows 10. For other system, please check the file read/write or multithreading functions in the
codes.

## Note

Change "const static size_t maxSize = 100000;" to "const static size_t maxSize = 1000" in file OcTreeKey.h, so that the
code will run faster.

## Usage

1. Sample your 3d object model from *.obj or *.ply to *.pcd, and there is an example in directory 3d_models. For the
   sampling method, please follow   https://github.com/PointCloudLibrary/pcl/blob/master/tools/mesh_sampling.cpp, and
   run with "./pcl_mesh_sampling.exe *.ply *.pcd -n_samples 100000 -leaf_size 0.5 -no_vis_result" or -leaf_size 0.0005,
   depending on the model.
2. Change the directory and model name in file DefaultConfiguration.yaml.
3. Put the file DefaultConfiguration.yaml in the correct path, and then run compiled program of main.cpp.
4. For the method_of_IG: Ours is 0, OA is 1, UV is 2, RSE is 3, APORA is 4, Kr is 5, NBVNET is 6. If you want to run
   with NBV-Net, please check nbvNetPath in file DefaultConfiguration.yaml, and run both compiled program of main.cpp
   and "python nbv_net/run_test.py {}" (replace {} by your model name) in pytorch environment at the same time.
5. There is a parameter "show", by default is 1, which means that the middle cloud will be shown in a pcl window, and
   close it to continue. If you don't want to show the middle cloud, change it to 0.

## Questions

Please contact 18210240033@fudan.edu.cn
