cmake_minimum_required(VERSION 3.8)
project(fake_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package( OpenCV REQUIRED )

include_directories( 
    include 
    ${OpenCV_INCLUDE_DIRS} 
)

# CAMERA SUBSCRIBER
add_executable(camera_subscriber src/camera_subscriber.cpp)
target_include_directories(camera_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(camera_subscriber PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  camera_subscriber
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(
    camera_subscriber
    ${OpenCV_LIBS}
)


# LIDAR SUBSCRIBER
add_executable(lidar_subscriber src/lidar_subscriber.cpp)
target_include_directories(lidar_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(lidar_subscriber PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  lidar_subscriber
  rclcpp
  sensor_msgs
  std_msgs
)

#FAKE_CAMERA_PRO
add_executable(fake_camera_pro src/fake_camera_pro.cpp)
target_include_directories(fake_camera_pro PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(fake_camera_pro PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  fake_camera_pro
  rclcpp
  sensor_msgs
)

#FALE LIDAR PRO
add_executable(fake_lidar_pro src/fake_lidar_pro.cpp)
target_include_directories(fake_lidar_pro PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(fake_lidar_pro PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  fake_lidar_pro
  rclcpp
  sensor_msgs
)

install(TARGETS camera_subscriber lidar_subscriber fake_lidar_pro fake_camera_pro
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY launch
#     DESTINATION share/${PROJECT_NAME}
# )

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
