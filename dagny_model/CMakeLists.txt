cmake_minimum_required(VERSION 2.8.3)
project(dagny_model)

find_package(catkin REQUIRED)

catkin_package()

install(FILES
  body.stl
  dagny.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
