cmake_minimum_required(VERSION 2.8.3)
project(prophesee_recorder)

find_package(catkin_simple REQUIRED )
find_package(catkin REQUIRED )
find_package(HDF5 COMPONENTS CXX)
include_directories(${HDF5_INCLUDE_DIR})

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -fopenmp")
SET(CMAKE_BUILD_TYPE Release)

set(HEADERS
    include/prophesee_recorder.h
)

set(SOURCES
    src/prophesee_recorder.cpp
)

cs_add_library(prophesee_recorder ${SOURCES} ${HEADERS})

# make executable
cs_add_executable(prophesee_recorder_node src/prophesee_recorder_node.cpp)
target_link_libraries(prophesee_recorder_node prophesee_recorder ${catkin_LIBRARIES} ${HDF5_CXX_LIBRARIES})

cs_install()
cs_export()