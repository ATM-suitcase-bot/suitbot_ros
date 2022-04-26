cmake_minimum_required(VERSION 3.1.0)
project(suitbot_ros)

set(CMAKE_BUILD_TYPE "Release")

execute_process(COMMAND lsb_release -cs
    OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
)

if(${LSB_RELEASE_ID_SHORT} MATCHES "bionic")
  set(CMAKE_CXX_FLAGS "-std=c++11")
  message("-- Systemm version: ${LSB_RELEASE_ID_SHORT}, set to C++11")
elseif(${LSB_RELEASE_ID_SHORT} MATCHES "focal")
  set(CMAKE_CXX_FLAGS "-std=c++14")
  message("-- Systemm version: ${LSB_RELEASE_ID_SHORT}, set to C++14")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  tf2_ros
  tf_conversions
  eigen_conversions
  visualization_msgs
  image_transport
  cv_bridge
)


find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)



add_message_files(FILES
  CmdMsg.msg
  TwoFloats.msg
  LocalMapMsg.msg
)

add_service_files(FILES
  InitializationSrvMsg.srv
  SetCourse.srv
  SpeechSrv.srv
)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp 
                  rospy
                  message_runtime
                  std_msgs 
                  geometry_msgs
                  sensor_msgs
                  nav_msgs
                  tf2_ros
                  tf
                  tf_conversions
                  eigen_conversions
                  image_transport
                  cv_bridge
                  visualization_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()


add_executable(job_management
  src/job_management.cpp
)

target_link_libraries(job_management ${catkin_LIBRARIES})


# add_executable(localization_node
#   src/localization/localization_node.cpp
#   src/occupancy_map.cpp
#   src/lidar.cpp
#   src/utility/pcl_utils.cpp
#   src/utility/utils.cpp
# )

# target_link_libraries(localization_node 
#   ${catkin_LIBRARIES} 
#   ${PCL_LIBRARIES} 
#   ${PCL_COMMON_LIBRARIES} 
#   ${PCL_IO_LIBRARIES} 
#   ${PCL_VISUALIZATION_LIBRARIES}
#   ${OpenCV_LIBS}
# )


add_executable(planner_node 
  src/path_planning/planner_node.cpp
  src/path_planning/planner.cpp
  src/path_planning/a_star.cpp
  src/occupancy_map.cpp
  src/utility/pcl_utils.cpp
  src/utility/utils.cpp
)

target_link_libraries(planner_node 
${catkin_LIBRARIES}
${PCL_LIBRARIES} 
${PCL_COMMON_LIBRARIES} 
${PCL_IO_LIBRARIES} 
${PCL_VISUALIZATION_LIBRARIES} 
${OpenCV_LIBS})


add_executable(local_map_generator 
  src/local_map_generation.cpp
  src/occupancy_map.cpp
  src/lidar.cpp
  src/utility/pcl_utils.cpp
  src/utility/utils.cpp
)



target_link_libraries(local_map_generator
${catkin_LIBRARIES} 
${PCL_LIBRARIES} 
${PCL_COMMON_LIBRARIES} 
${PCL_IO_LIBRARIES} 
${PCL_VISUALIZATION_LIBRARIES}
${OpenCV_LIBS})

add_executable(lidar_parse
  src/lidar_parse.cpp
  src/occupancy_map.cpp
  src/lidar.cpp
  src/utility/pcl_utils.cpp
  src/utility/utils.cpp
)


target_link_libraries(lidar_parse
${catkin_LIBRARIES} 
${PCL_LIBRARIES} 
${PCL_COMMON_LIBRARIES} 
${PCL_IO_LIBRARIES} 
${PCL_VISUALIZATION_LIBRARIES}
${OpenCV_LIBS})

add_executable(combine_pcd
  src/process_map/combine_pcd.cpp
  src/occupancy_map.cpp
  src/lidar.cpp
  src/utility/pcl_utils.cpp
  src/utility/utils.cpp
)


target_link_libraries(combine_pcd
${catkin_LIBRARIES} 
${PCL_LIBRARIES} 
${PCL_COMMON_LIBRARIES} 
${PCL_IO_LIBRARIES} 
${PCL_VISUALIZATION_LIBRARIES}
${OpenCV_LIBS}
)

add_executable(map_processing
  src/process_map/pcd_map_process.cpp
  src/occupancy_map.cpp
  src/lidar.cpp
  src/utility/pcl_utils.cpp
  src/utility/utils.cpp
)


target_link_libraries(map_processing
${catkin_LIBRARIES} 
${PCL_LIBRARIES} 
${PCL_COMMON_LIBRARIES} 
${PCL_IO_LIBRARIES} 
${PCL_VISUALIZATION_LIBRARIES}
${OpenCV_LIBS}
)


add_executable(unit_test
  src/test/unit_test.cpp
  src/occupancy_map.cpp
  src/lidar.cpp
  src/utility/pcl_utils.cpp
  src/utility/utils.cpp
)


target_link_libraries(unit_test
${catkin_LIBRARIES} 
${PCL_LIBRARIES} 
${PCL_COMMON_LIBRARIES} 
${PCL_IO_LIBRARIES} 
${PCL_VISUALIZATION_LIBRARIES}
${OpenCV_LIBS}
)