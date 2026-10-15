cmake_minimum_required(VERSION 3.16)
project(sorting_cell_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(sdformat12 REQUIRED)

add_library(SortingCellSystem SHARED src/sorting_cell_system.cpp)
target_include_directories(SortingCellSystem PRIVATE include)
target_link_libraries(SortingCellSystem
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::register
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8
  sdformat12::sdformat12)

install(TARGETS SortingCellSystem
  LIBRARY DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

# Let Gazebo find the plugin when this workspace is sourced.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
