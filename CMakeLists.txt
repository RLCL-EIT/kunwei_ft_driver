cmake_minimum_required(VERSION 3.16)
project(kunwei_ft_driver VERSION 0.1.0 LANGUAGES CXX)
include(GNUInstallDirs)

add_library(kunwei_ft_driver SHARED src/client.cpp)
add_library(kunwei_ft_driver::driver ALIAS kunwei_ft_driver)
target_compile_features(kunwei_ft_driver PUBLIC cxx_std_17)
target_compile_options(kunwei_ft_driver PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(kunwei_ft_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(kunwei_ft_driver PROPERTIES
  VERSION ${PROJECT_VERSION} SOVERSION 0 EXPORT_NAME driver)
install(TARGETS kunwei_ft_driver EXPORT export_driver
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION include)
install(EXPORT export_driver NAMESPACE kunwei_ft_driver::
  DESTINATION share/${PROJECT_NAME}/cmake)

option(KUNWEI_BUILD_ROS_PUBLISHER "Build ROS 2 driver and visualization launch" ON)
if(KUNWEI_BUILD_ROS_PUBLISHER)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  add_executable(driver src/publisher.cpp)
  target_link_libraries(driver PRIVATE kunwei_ft_driver)
  ament_target_dependencies(driver PUBLIC rclcpp geometry_msgs)
  set_target_properties(driver PROPERTIES INSTALL_RPATH "$ORIGIN/..")
  install(TARGETS driver RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME}
    PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
  ament_export_targets(export_driver HAS_LIBRARY_TARGET)
  ament_package()
else()
  include(CMakePackageConfigHelpers)
  configure_package_config_file(cmake/kunwei_ft_driverConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/kunwei_ft_driverConfig.cmake
    INSTALL_DESTINATION share/${PROJECT_NAME}/cmake)
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/kunwei_ft_driverConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/kunwei_ft_driverConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/kunwei_ft_driverConfigVersion.cmake
    DESTINATION share/${PROJECT_NAME}/cmake)
endif()
