cmake_minimum_required(VERSION 3.10)
project(vgicp)

# set(CMAKE_BUILD_TYPE Release)
find_package(OpenMP) # ndt_omp用到了openMP库

if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Eigen
find_package(Eigen3 REQUIRED)

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gtest
# find_package(GTest REQUIRED)
# include_directories(${GTEST_INCLUDE_DIRS})

# gtsam
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# CUDA
find_package(CUDA REQUIRED)
include_directories(
    ${CUDA_INCLUDE_DIRS}
    thirdparty/Eigen
    thirdparty/nvbio
)
link_directories(${CUDA_LIBRARY_DIRS})

set(THIRD_PARTY_LIBS
    ${PCL_LIBRARIES}
    ${GLOG_LIBRARIES}
    fast_gicp
    yaml-cpp
    glog::glog
    gflags
)

include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PROJECT_SOURCE_DIR}/lib)

add_library(mapping_front mapping_front.cc)
target_link_libraries(mapping_front
    ${THIRD_PARTY_LIBS}
)

add_library(mapping_loop loop_closing.cc)
target_link_libraries(mapping_loop
    ${THIRD_PARTY_LIBS}
)

add_library(mapping_back back_end.cc)
target_link_libraries(mapping_back
    ${THIRD_PARTY_LIBS}
    gtsam
)

add_executable(mapping_node main_node_flow.cc)
target_link_libraries(mapping_node
    mapping_front
    mapping_loop
    mapping_back
)