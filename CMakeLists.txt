cmake_minimum_required(VERSION 2.8.3)
project(pioneer3at)

find_package(catkin REQUIRED COMPONENTS 
        roscpp 
        sensor_msgs 
        nav_msgs 
        tf
        message_generation
        roslib
)

#######################
## Gazebo Standalone ##
#######################

find_package(gazebo REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(OpenCV)
   include_directories(${OpenCV_INCLUDE_DIRS})


###################################################
## Declare things to be passed to other projects ##
###################################################

add_message_files(
  FILES
  Poses.msg
  Signal.msg
  OccMap.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES p2os_urdf
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf message_runtime cv_bridge roslib
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

## Declare a cpp executable
add_executable(joint_state_publisher src/joint_state_publisher.cc)
target_link_libraries(joint_state_publisher ${catkin_LIBRARIES} )
add_dependencies(joint_state_publisher pioneer3at_generate_messages_cpp)

add_executable(print_coords  src/print_coords.cc)
target_link_libraries(print_coords  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(print_coords pioneer3at_generate_messages_cpp)

add_executable(laserscan  src/laserscan.cc)
target_link_libraries(laserscan  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(laserscan pioneer3at_generate_messages_cpp)

add_executable(explorer  src/explorer.cc)
target_link_libraries(explorer  ${catkin_LIBRARIES})
add_dependencies(explorer pioneer3at_generate_messages_cpp)

add_executable(map_node  src/map_node.cc)
target_link_libraries(map_node  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(map_node pioneer3at_generate_messages_cpp)

add_executable(map_node_icra  src/map_node_icra.cc)
target_link_libraries(map_node_icra  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(map_node_icra pioneer3at_generate_messages_cpp)

add_executable(new_map_node  src/new_map_node.cc)
target_link_libraries(new_map_node  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(new_map_node pioneer3at_generate_messages_cpp)

add_executable(map_node_real  src/map_node_real.cc)
target_link_libraries(map_node_real  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(map_node_real pioneer3at_generate_messages_cpp)

add_executable(image_save  src/image_save.cc)
target_link_libraries(image_save  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(image_save pioneer3at_generate_messages_cpp)


#############
## Install ##
#############
install(PROGRAMS  scripts/init_amcl
                  scripts/teleop_joy
#                  scripts/gazebo_p3at
                  scripts/proto/__init__.py
                  scripts/proto/header.py
                  scripts/proto/packet.py
                  scripts/proto/pose.py
                  scripts/proto/publish.py
                  scripts/proto/quaternion.py
                  scripts/proto/subscribe.py
                  scripts/proto/time.py
                  scripts/proto/vector3d.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS joint_state_publisher
#                gazebo_bridge
#                gazebo_laserscan
#                frontier_explorer
                print_coords
                laserscan
#                multi_frontier_explorer
#                frontier_window_explorer
                map_node
                image_save
#                minpos_explorer
#                new_explorer
#                new_explorer_ent
#                new_explorer_inf
#                new_explorer_real_info
#                new_explorer_real_ent
#                map_node_icra
                explorer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY 
  urdf 
  launch
  meshes 
  models
  config 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

