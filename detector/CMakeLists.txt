cmake_minimum_required(VERSION 3.8)
project(detector)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -O3)
## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(interfaces REQUIRED)


ament_auto_find_build_dependencies()
# find_package(rclcpp REQUIRED)
# find_package(OpenCV REQUIRED)


ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN DetectorNode
  EXECUTABLE detector_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  #############################################
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install(TARGETS
#   corner_detection
#   DESTINATION lib/${PROJECT_NAME})

# ament_package()
ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
