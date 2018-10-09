cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(linemod_dev)

INCLUDE_DIRECTORIES(./include)
find_package(OpenCV 3 REQUIRED)
message("opencv found in ${OpenCV_INCLUDE_DIRS}")
add_definitions(-DCVAPI_EXPORTS)
aux_source_directory(./src  lmd_src)
add_library(lmd_dev  SHARED ${lmd_src})
target_link_libraries(lmd_dev ${OpenCV_LIBS})

add_subdirectory(./src/App/)

