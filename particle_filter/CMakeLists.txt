cmake_minimum_required(VERSION 2.4.6)
project(particle_filter)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  geometry_msgs
  dot_finder
  cv_bridge
  image_transport
  tum_ardrone
  ardrone_autonomy
# Uncomment this for using this project in ROS Indigo and comment it for using this project in ROS Hydro
  cmake_modules
)

# since we need eigen search it as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(Eigen REQUIRED)

# Add opencv
find_package(OpenCV REQUIRED)

# Generate the reconfigure options
generate_dynamic_reconfigure_options(cfg/ParticleFilter.cfg)

# export the dependencies of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp 
  dynamic_reconfigure 
  geometry_msgs 
  dot_finder
  tf
  tum_ardrone
  ardrone_autonomy
  DEPENDS eigen OpenCV
)


# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# make the executable
add_executable(particle_filter
  cfg/ParticleFilter.cfg

  include/particle_filter/visualization.h
  include/particle_filter/region_of_interest.h
  include/particle_filter/mutual_pose_estimation.h
  include/particle_filter/particle_filter_node.h

  src/visualization.cpp
  src/region_of_interest.cpp
  src/particle_filter_node.cpp 
  src/mutual_pose_estimation.cpp
)

# to build the executable we depend on other packets,
# they need to be build beforehand, especially the messages
add_dependencies(particle_filter ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(particle_filter ${catkin_EXPORTED_TARGETS}  tf ${PROJECT_NAME}_gencf)

# link the executable to the necessary libs
target_link_libraries(particle_filter
   yaml-cpp
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
