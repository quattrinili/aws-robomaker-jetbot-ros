cmake_minimum_required(VERSION 2.8.3)

project(gazebo_dance_world)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models worlds launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
)