cmake_minimum_required(VERSION 2.8)
project(gridMap)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")


include_directories(${PROJECT_SOURCE_DIR})


# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# eigen
include_directories("/usr/include/eigen3/")

# pcl
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} SHARED
        grid_map.cpp
        grid_mapper.cpp
        )

add_executable(createMap main.cpp)
target_link_libraries(createMap ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${PCL_LIBRARIES}
        ${PROJECT_NAME}
        )

