cmake_minimum_required(VERSION 2.8.3)
project(dagny_test)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  roscpp)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(drive src/drive.cpp)
target_link_libraries(drive ${catkin_LIBRARIES})

add_executable(count src/count.cpp)
target_link_libraries(count ${catkin_LIBRARIES})

add_executable(steer src/steer.cpp)
target_link_libraries(steer ${catkin_LIBRARIES})

add_executable(log src/log.cpp)
target_link_libraries(log ${catkin_LIBRARIES})

add_executable(laser src/laser.cpp)
target_link_libraries(laser ${catkin_LIBRARIES})

install(TARGETS drive count steer log laser
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
