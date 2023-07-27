cmake_minimum_required(VERSION 3.0.0)
project(MultiLidarSplicing VERSION 0.1.0)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

find_package(message_filters REQUIRED)
include_directories(${message_filters_INCLUDE_DIRS})

find_package(pcl_ros REQUIRED)
include_directories(${pcl_ros_INCLUDE_DIRS})

if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(DEFAULT_COMPILE -std=c++14 -fpermissive -O0 -g3 -Wall -fPIC)
    file(GLOB_RECURSE SOURCE_PATH ${PROJECT_SOURCE_DIR}/src/*.cpp)
    add_library(${PROJECT_NAME} SHARED ${SOURCE_PATH})
    target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES} ${message_filters_LIBRARIES} ${pcl_ros_LIBRARIES} libjsoncpp.so)
elseif (CMAKE_BUILD_TYPE MATCHES Release)
    set(DEFAULT_COMPILE -std=c++14 -fpermissive -O3 -Wall -fPIC)
    file(GLOB_RECURSE SOURCE_PATH ${PROJECT_SOURCE_DIR}/src/*.cpp)
    add_library(${PROJECT_NAME} SHARED ${SOURCE_PATH})
    target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES} ${message_filters_LIBRARIES} ${pcl_ros_LIBRARIES} libjsoncpp.so)
else()
    set(DEFAULT_COMPILE -std=c++14 -fpermissive -O3 -Wall -fPIC)   
endif()

add_executable(main ${PROJECT_SOURCE_DIR}/app/main.cpp)
target_link_libraries(main ${PROJECT_NAME} ${roscpp_LIBRARIES} libjsoncpp.so)