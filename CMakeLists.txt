cmake_minimum_required(VERSION 3.1.0)
project(suitbot_ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)



add_message_files(FILES
  CmdMsg.msg
)

add_service_files(FILES
  InitializationSrvMsg.srv
)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
                  message_runtime
                  std_msgs 
                  geometry_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_package()

add_executable(job_management
  src/job_management.cpp
)

target_link_libraries(job_management ${catkin_LIBRARIES})


add_executable(initialization_node
  src/localization/initialization.cpp
)

target_link_libraries(initialization_node ${catkin_LIBRARIES})



