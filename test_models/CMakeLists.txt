cmake_minimum_required(VERSION 2.8.3)
project(test_models)
find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY abb baxter denso fanuc fetch jaco pr2 ur
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
