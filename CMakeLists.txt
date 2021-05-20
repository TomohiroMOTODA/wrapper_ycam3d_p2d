cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(plyConverter)

add_executable (plyConverter main.cpp)

#### PCL Library #####
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries (plyConverter ${PCL_LIBRARIES})

#### OpenCV Library ####
find_package(OpenCV 3.2.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries (plyConverter ${OpenCV_LIBS})

#### Boost Library ####
set(Boost_USE_STATIC_LIBS        ON)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     ON)
find_package(Boost 1.36.0 REQUIRED COMPONENTS date_time system)
include_directories(${BOOST_INCLUDE_DIRS})
target_link_libraries(plyConverter ${BOOST_LIBRARY_DIRS})
