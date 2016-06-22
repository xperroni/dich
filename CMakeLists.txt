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

add_message_files(
  FILES
  Match.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

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
  "src/dich/steerer.cpp"
  "src/dich/replayer.cpp"
  "src/dich/teacher.cpp"
)

add_executable(ground_truth
  "src/dich/ground_truth.cpp"
)

add_executable(replay
  "src/dich/replay.cpp"
)

add_executable(steering
  "src/dich/steering.cpp"
)

add_executable(steering_offline
  "src/dich/steering_offline.cpp"
)

add_executable(teach
  "src/dich/teach.cpp"
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

target_link_libraries(steering
  ${catkin_LIBRARIES}
  dich
)

target_link_libraries(steering_offline
  ${catkin_LIBRARIES}
  dich
)

target_link_libraries(teach
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
