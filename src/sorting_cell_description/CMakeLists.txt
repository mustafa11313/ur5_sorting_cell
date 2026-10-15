cmake_minimum_required(VERSION 3.16)
project(sorting_cell_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_xacro test/test_xacro.py)
endif()

ament_package()
