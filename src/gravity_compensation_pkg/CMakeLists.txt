cmake_minimum_required(VERSION 3.8)
project(gravity_compensation_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# <<< 1. 이 위치에 패키지를 찾도록 추가합니다. >>>
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# <<< 2. 기존 install(DIRECTORY...)는 이 위치로 옮기는 것이 더 좋습니다. >>>
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

add_executable(gravity_comp_node src/gravity_comp_node.cpp)

# <<< 3. gravity_comp_node가 rclcpp에 의존한다는 것을 명시합니다. (가장 중요!) >>>
ament_target_dependencies(gravity_comp_node rclcpp tf2_ros geometry_msgs)

target_include_directories(gravity_comp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gravity_comp_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS gravity_comp_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()