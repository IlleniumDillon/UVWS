cmake_minimum_required(VERSION 3.5)
project(communication)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_list REQUIRED)
find_package(uvinterfaces REQUIRED)
find_package(serial REQUIRED)
include_directories(
  include/communication
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB SOURCEFILE src/*.cc)

add_executable(communication_node ${SOURCEFILE})
ament_target_dependencies(
  communication_node
  "rclcpp"
  "message_list"
  "serial"
  "uvinterfaces"
)

install(TARGETS
  communication_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
