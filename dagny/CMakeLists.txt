cmake_minimum_required(VERSION 2.8.3)
project(dagny)

find_package(catkin REQUIRED)

catkin_package()

install(FILES
  camera.launch
  dagny.launch
  dagny_model.launch
  diagnostics.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  bin/compass_cal.py
  bin/manual_cal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
