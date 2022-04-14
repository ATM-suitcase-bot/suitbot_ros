cmake_minimum_required(VERSION 3.1.0)
project(suitbot_ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
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
  visualization_msgs
  cv_bridge
)

find_package(OpenCV 3.2 REQUIRED)
# find_package(PCL REQUIRED)
# find_package(Eigen3 REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)



add_message_files(FILES
  CmdMsg.msg
  TwoFloats.msg
)

add_service_files(FILES
  InitializationSrvMsg.srv
  SetCourse.srv
  SpeechSrv.srv
)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
                  rospy
                  message_runtime
                  std_msgs 
                  geometry_msgs
                  sensor_msgs
                  nav_msgs
                  tf2_ros
                  visualization_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  # ${EIGEN3_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
)

# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

catkin_package()


add_executable(job_management
  src/job_management.cpp
)

target_link_libraries(job_management ${catkin_LIBRARIES})


# add_executable(localization_node
#   src/localization/localization_node.cpp
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
)

target_link_libraries(planner_node ${catkin_LIBRARIES} ${OpenCV_LIBS})




