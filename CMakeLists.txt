cmake_minimum_required(VERSION 3.10)

project(nbv_simulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV 4.4.0 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen3 3.3.9 REQUIRED)

find_package(PCL 1.9.1 REQUIRED)

find_package(octomap 1.9.1 REQUIRED)

find_package(AliceVision REQUIRED)

set(NBV_SIMULATION_SOURCES
        nbv_simulation/main.cpp
        nbv_simulation/utils.cpp
        nbv_simulation/Ray.cpp
        nbv_simulation/Ray.h
        nbv_simulation/Ray_Hash.cpp
        nbv_simulation/Ray_Hash.h
        nbv_simulation/Ray_Information.cpp
        nbv_simulation/Ray_Information.h
        nbv_simulation/MCMF.cpp
        nbv_simulation/MCMF.h
        nbv_simulation/Share_Data.cpp
        nbv_simulation/Share_Data.h
        nbv_simulation/View.cpp
        nbv_simulation/View.h
        nbv_simulation/Voxel_Information.cpp
        nbv_simulation/Voxel_Information.h
        nbv_simulation/View_Space.cpp
        nbv_simulation/View_Space.h
        nbv_simulation/Perception_3D.cpp
        nbv_simulation/Perception_3D.h
        nbv_simulation/Views_Information.cpp
        nbv_simulation/Views_Information.h
        nbv_simulation/views_voxels_MF.cpp
        nbv_simulation/views_voxels_MF.h
        nbv_simulation/Information.cpp
        nbv_simulation/Information.h
        nbv_simulation/NBV_Planner.cpp
        nbv_simulation/NBV_Planner.h
        nbv_simulation/Visual.cpp
        nbv_simulation/Visual.h nbv_simulation/Visual.h)

#set(NBV_SIMULATION_HEADERS
#        nbv_simulation/Information.hpp
#        nbv_simulation/Share_Data.cpp
#        nbv_simulation/View_Space.cpp)

add_executable(nbv_simulation ${NBV_SIMULATION_SOURCES})

target_include_directories(nbv_simulation PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/nbv_simulation>
        $<INSTALL_INTERFACE:include>
        ${OCTOMAP_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS})

target_link_libraries(nbv_simulation PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
        Boost::thread
        aliceVision_sfmData
        aliceVision_sfmDataIO
        )