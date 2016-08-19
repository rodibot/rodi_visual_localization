cmake_minimum_required(VERSION 2.8.3)
project(rodi_visual_localization)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/rodi_visual_localization
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch  
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
