cmake_minimum_required(VERSION 3.8)
project(gazebo_env)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Find Gazebo
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(no_physics_plugin SHARED no_physics_plugin/no_physics_plugin.cc)
ament_target_dependencies(no_physics_plugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "std_msgs"
)
target_link_libraries(no_physics_plugin ${GAZEBO_LIBRARIES})


# Include Cpp "include" directory
include_directories(include
${gazebo_dev_INCLUDE_DIRS}
${gazebo_ros_INCLUDE_DIRS})

## COMPILE
add_library(
  gazebo_env
  SHARED
  hardware/urbot.cpp
  #hardware/rrbot.cpp
)

target_compile_features(gazebo_env PUBLIC cxx_std_17)
target_include_directories(gazebo_env PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/gazebo_env>
)
ament_target_dependencies(
  gazebo_env PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "GAZEBO_ENV_BUILDING_DLL")
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_ur5e.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/gazebo_env
)
install(
  DIRECTORY meshes
  DESTINATION share/gazebo_env
)
install(
  DIRECTORY description/urdf
  DESTINATION share/gazebo_env
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/gazebo_env
)
install(TARGETS gazebo_env
  EXPORT export_gazebo_env
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

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

ament_export_targets(export_gazebo_env HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
