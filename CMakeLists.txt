cmake_minimum_required(VERSION 2.8.3)
project(dich)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  clarus
  cv_video
  sensor_msgs
  terminal_io
  yamabros
)

# Add FFTW support
list(APPEND FFTW3_LIBS fftw3)
add_library(fftw3 SHARED IMPORTED)
set_target_properties(fftw3 PROPERTIES IMPORTED_LOCATION
  "/usr/lib/x86_64-linux-gnu/libfftw3.so"
)

# Add threaded FFTW support
find_package(Threads)
list(APPEND FFTW3_LIBS fftw3_threads)
add_library(fftw3_threads SHARED IMPORTED)
set_target_properties(fftw3_threads PROPERTIES IMPORTED_LOCATION
  "/usr/lib/x86_64-linux-gnu/libfftw3_threads.so"
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dich
  CATKIN_DEPENDS sensor_msgs clarus cv_video terminal_io yamabros
  DEPENDS OpenCV boost
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  "include/"
)

add_library(dich
  "src/dich/correspondences.cpp"
  "src/dich/difference_filter.cpp"
  "src/dich/difference_image.cpp"
  "src/dich/difference_pairing.cpp"
  "src/dich/difference_shift.cpp"
  "src/dich/io.cpp"
  "src/dich/settings.cpp"
  "src/dich/replayer.cpp"
  "src/dich/teacher.cpp"
)

add_executable(ground_truth
  "src/dich/ground_truth.cpp"
)

add_executable(replay
  "src/dich/replay.cpp"
)

add_executable(teach
  "src/dich/teach.cpp"
)

add_executable(steering_offline
  "src/dich/steering_offline.cpp"
)

target_link_libraries(dich
  ${catkin_LIBRARIES}
)

target_link_libraries(ground_truth
  ${catkin_LIBRARIES}
  dich
)

target_link_libraries(replay
  ${catkin_LIBRARIES}
  dich
  ${FFTW3_LIBS}
)

target_link_libraries(teach
  ${catkin_LIBRARIES}
  dich
)

target_link_libraries(steering_offline
  ${catkin_LIBRARIES}
  dich
)

#############
## Install ##
#############

# TODO

#############
## Testing ##
#############

# TODO
