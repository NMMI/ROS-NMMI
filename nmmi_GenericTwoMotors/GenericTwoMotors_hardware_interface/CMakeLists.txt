cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(generictwomotors_hardware_interface VERSION 2.1.1 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    control_toolbox
    transmission_interface
    qb_device_hardware_interface
    nmmi_srvs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    control_toolbox
    transmission_interface
    qb_device_hardware_interface
    nmmi_srvs
)

# Exported libraries
add_library(${PROJECT_NAME}
  src/GenericTwoMotors_hardware_interface.cpp
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# C++ Settings
if (CMAKE_VERSION VERSION_LESS "3.1")
  target_compile_options(${PROJECT_NAME}
    PUBLIC
      "-std=c++14"
  )
else ()
  set_property(
    TARGET
      ${PROJECT_NAME}
    PROPERTY CXX_STANDARD 14
  )

  set_property(
    TARGET
      ${PROJECT_NAME}
    PROPERTY CXX_STANDARD_REQUIRED ON
  )
endif ()

# Installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
