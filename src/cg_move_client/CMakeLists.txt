cmake_minimum_required(VERSION 3.8)
project(cg_move_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

add_executable(move_client src/map_move.cpp)
ament_target_dependencies(move_client rclcpp cg_interfaces)

add_executable(astar src/astar.cpp)
ament_target_dependencies(astar rclcpp cg_interfaces)

install(TARGETS
  move_client
  astar
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
