cmake_minimum_required(VERSION 3.5)
project(uvplanner)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uvinterfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# find_package(matplotlib_cpp REQUIRED)
# find_package(Python3 COMPONENTS Development NumPy)

include_directories(include/uvplanner)
include_directories(${OpenCV_INCLUDE_DIRS}) 

file(GLOB SOURCEFILE src/*.cpp)

add_executable(uvplanner_node ${SOURCEFILE})
# target_include_directories(uvplanner_node PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS} lib)
# target_link_libraries(uvplanner_node Python3::Python Python3::NumPy)
ament_target_dependencies(
  uvplanner_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "OpenCV"
  "Eigen3"
)

install(TARGETS
  uvplanner_node
  DESTINATION lib/${PROJECT_NAME})

# add_executable(test_node ${TESTFILE})
# ament_target_dependencies(
#   test_node
#   "rclcpp"
#   "uvinterfaces"
# )

# install(TARGETS
#   test_node
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch map
  DESTINATION share/${PROJECT_NAME}
)

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
