cmake_minimum_required(VERSION 2.8.3)
project(optris_drivers_v2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  camera_info_manager
)
##Dodano
add_service_files(
  DIRECTORY
    srv
  FILES
    Palette.srv
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES optris_drivers_v2
#  CATKIN_DEPENDS image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/libirimager
  
)

LINK_DIRECTORIES(/usr/lib)

LINK_LIBRARIES(irimager irimageprocessing ircore udev)

add_executable(optris_imager_node src/optris_imager_node.cpp src/IRImagerHandler.cpp)

##Dodano
add_executable(optris_colorconvert_node src/optris_colorconvert_node.cpp)
##Dodano
add_dependencies(optris_colorconvert_node optris_drivers_generate_messages_cpp)

target_link_libraries(optris_imager_node ${catkin_LIBRARIES} 
 )
##Dodano
target_link_libraries(optris_colorconvert_node
  ${catkin_LIBRARIES}
)
##Dodano
install(TARGETS optris_colorconvert_node optris_imager_node
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




