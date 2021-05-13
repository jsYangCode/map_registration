cmake_minimum_required(VERSION 2.8.3)
project(map_registration)

# -mavx causes a lot of errors!!
add_definitions(-std=c++11)
# add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
# set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_ros
  roscpp
  rospy
  sensor_msgs
 # ndt_omp
  geometry_msgs
  tf2_geometry_msgs
        tf2
        tf2_msgs
        tf2_ros
)


find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
        #geometry_msgs
        tf2
        tf2_msgs
        tf2_ros

  INCLUDE_DIRS include
#  LIBRARIES hdl_scan_matching_odometry
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(main apps/main.cpp)
target_link_libraries(main ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable( map_registration apps/map_registration1.cpp)
target_link_libraries( map_registration ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(test1 apps/test1.cpp)
target_link_libraries(test1 ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})



