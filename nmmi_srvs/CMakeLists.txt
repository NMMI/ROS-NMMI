cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(nmmi_srvs VERSION 2.0.3 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    std_srvs
    nmmi_msgs
    message_generation
)

add_service_files(
  DIRECTORY srv
  FILES
    GetADCRawValues.srv
    GetADCMap.srv
    GetControlMode.srv
    GetEncoderRawValues.srv
    GetEncoderMap.srv
    GetIMUValues.srv
    GetIMUParam.srv
)

generate_messages(
  DEPENDENCIES
    std_srvs
    nmmi_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    std_srvs
    nmmi_msgs
    message_runtime
)

# Installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)