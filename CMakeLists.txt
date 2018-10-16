cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
add_definitions(-std=c++11)
project(linemod_dev)

INCLUDE_DIRECTORIES(./include ./src)
find_package(OpenCV 3 REQUIRED)
find_package(Open3D REQUIRED)
INCLUDE_DIRECTORIES(${Open3D_INCLUDE_DIRS})
message("opencv found in ${OpenCV_INCLUDE_DIRS}")
add_definitions(-DCVAPI_EXPORTS)
aux_source_directory(./src  lmd_src)
add_library(lmd_dev  SHARED ${lmd_src})
target_link_libraries(lmd_dev ${OpenCV_LIBS} ${Open3D_LIBRARIES})

message("open3d libs: ${Open3D_LIBRARIES}")
add_subdirectory(./src/App/)
add_subdirectory(./src/test/)

