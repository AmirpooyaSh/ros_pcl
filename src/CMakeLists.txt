cmake_minimum_required(VERSION 3.10)
project(pki_simulation)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    pluginlib
    #geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    roscpp
    rospy
    darknet_ros_msgs
    cv_bridge
    message_generation
)

find_package(Eigen3 REQUIRED)

#find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
  include
  ${OpenCV_INCLUDE_DIRS}
)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_service_files(
  FILES
  node_ready.srv
  move_base_request.srv
  execute_plan.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

### Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)


catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    interactive_markers
    message_runtime
  DEPENDS
    #Eigen3
)

###########
## Build ##
###########

include_directories(SYSTEM
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  /usr/include/vtk-9.2
  #/usr/local/include/pcl-1.13/pcl
)
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  /usr/include/vtk-9.2
  #/usr/local/include/pcl-1.13/pcl
)

#add_executable(move_base src/move_base.cpp)
#target_link_libraries(move_base ${catkin_LIBRARIES})
#install(TARGETS move_base DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# add_executable(inspection_bot
#   src/inspection_bot.cpp
#   src/facilityManagementPlan.cpp
#   src/inventoryClerk.cpp
#   src/cvUtils.cpp
   #src/CVLoadAndPublish.cpp
# )
# target_link_libraries(inspection_bot ${catkin_LIBRARIES})
# install(TARGETS inspection_bot DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(people_observer 
  #src/people_observer.cpp
  #src/PeopleLocalizer.cpp
  #src/frustum.cpp
  #src/move_base.cpp
  #src/yolo_action_client.cpp
  #src/cvUtils.cpp
  #src/segmentation_pipeline.cpp
  #src/convexHull.cpp
  #src/gaussKernel.cpp
#)
#target_link_libraries(people_observer
  #Eigen3::Eigen
  #${catkin_LIBRARIES}
  #pcl_visualization
#)
add_executable(culling
  src/frustum.cpp
  #src/PeopleLocalizer.cpp
  #src/move_base.cpp
  #src/yolo_action_client.cpp
  #src/cvUtils.cpp
  #src/segmentation_pipeline.cpp
  #src/convexHull.cpp
  #src/gaussKernel.cpp
  src/culling.cpp

)
target_link_libraries(culling
  Eigen3::Eigen
  pcl_visualization
  ${catkin_LIBRARIES}
)


#install(TARGETS people_observer DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



