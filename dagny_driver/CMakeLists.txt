cmake_minimum_required(VERSION 2.8.3)
project(dagny_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
  diagnostic_msgs
  message_generation)

add_message_files(FILES Encoder.msg Goal.msg Battery.msg NavSatFix.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(dagny_driver src/hardware_interface.cpp
  src/protocol.cpp src/steer.cpp)
target_link_libraries(dagny_driver ${catkin_LIBRARIES})
add_dependencies(dagny_driver dagny_driver_generate_messages_cpp)

install(TARGETS dagny_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
