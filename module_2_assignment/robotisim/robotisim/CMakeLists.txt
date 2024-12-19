cmake_minimum_required(VERSION 3.8)
project(robotisim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)



# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add the executable

add_executable(move_circle src/move_circle.cpp)
add_executable(move_logarithmic src/move_logarithmic.cpp)
add_executable(task5_pub src/task5_pub.cpp)
add_executable(task5_sub src/task5_sub.cpp)

# Link the executable with dependencies
ament_target_dependencies(move_circle  rclcpp geometry_msgs)
ament_target_dependencies(move_logarithmic  rclcpp geometry_msgs)
ament_target_dependencies(task5_pub  rclcpp std_msgs)
ament_target_dependencies(task5_sub  rclcpp std_msgs)
#add new folder

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)

# Install the executable
install(TARGETS
  move_circle
  move_logarithmic
  task5_pub
  task5_sub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
