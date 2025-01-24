cmake_minimum_required(VERSION 3.8)
project(simplecam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ADDED---------------------------------------------------
# 1. Find Packages 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# 2. Create Executable for Joint Publisher
add_executable(picamcsi src/picamcsi.cpp)
ament_target_dependencies(picamcsi rclcpp std_msgs sensor_msgs cv_bridge image_transport)

target_link_libraries(picamcsi ${OpenCV_LIBS})

# 3. Copy Files 
install(TARGETS
  picamcsi
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
    launch 
  DESTINATION share/${PROJECT_NAME}
)
# END ADDED---------------------------------------------------


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()