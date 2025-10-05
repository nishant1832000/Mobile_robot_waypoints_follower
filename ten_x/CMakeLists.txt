cmake_minimum_required(VERSION 3.8)
project(ten_x)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

include_directories(include)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(dependencies
    rclcpp
    geometry_msgs
    visualization_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
    )


add_executable(smoother src/path_smoother.cpp)
add_executable(trapezoid src/trapezoid_profile.cpp)

ament_target_dependencies(smoother
      ${dependencies}
)
ament_target_dependencies(trapezoid
      ${dependencies}
)


target_include_directories(smoother PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(smoother PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(trapezoid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(trapezoid PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS smoother
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS trapezoid
  DESTINATION lib/${PROJECT_NAME}) 


install(DIRECTORY include/
  DESTINATION include/
)

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

ament_package()
