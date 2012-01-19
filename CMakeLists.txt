cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(smooth_normal_segmentation C CXX)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
link_directories(${CMAKE_BINARY_DIR}/lib)
include_directories(${CMAKE_SOURCE_DIR}/include)

SET(CMAKE_BUILD_TYPE "Debug")

find_package(PCL 1.5 REQUIRED COMPONENTS 
			filters io search kdtree features)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


FIND_PACKAGE( Boost 1.40 COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_executable(segment src/SmoothNormal.cpp src/segmentation.cpp src/segmentation_tester.cpp)
target_link_libraries(segment ${PCL_LIBRARIES} ${Boost_LIBRARIES})									 

