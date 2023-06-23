cmake_minimum_required(VERSION 3.8)
project(uv_obstacle_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  visualization_msgs
  image_transport
  cv_bridge
  OpenCV
  Eigen3
)


set(library_name libraries)

include_directories(
  include
)

add_library(${library_name}
  src/kalman_filter.cpp
  src/UV_detector.cpp
)
target_link_libraries(${library_name} ${OpenCV_LIBS})
ament_target_dependencies(${library_name} ${dependencies})

add_executable(uv_obstacle_detector_node src/main.cpp)
ament_target_dependencies(uv_obstacle_detector_node ${dependencies})
target_link_libraries(uv_obstacle_detector_node ${library_name})



install(TARGETS uv_obstacle_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
