cmake_minimum_required(VERSION 3.12)

project(Wattzon)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -lboost_system")

set( SOURCE_FILES
     #src/main.cpp
     src/serialIO.cpp
     src/roomba.cpp
     src/roombaTest.cpp)

find_package(realsense2 REQUIRED )

add_executable(Wattzon ${SOURCE_FILES}) 
include_directories(Wattzon ${realsense_INCLUDE_DIR})
target_include_directories(Wattzon PRIVATE ${realsense_INCLUDE_DIR})
target_link_libraries(Wattzon ${realsense2_LIBRARY})

