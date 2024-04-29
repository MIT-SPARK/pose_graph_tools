cmake_minimum_required(VERSION 3.16)
project(pose_graph_tools VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE $<INSTALL_INTERFACE:include>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT pose_graph_tools-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(
  EXPORT pose_graph_tools-targets
  FILE pose_graph_toolsTargets.cmake
  NAMESPACE pose_graph_tools::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pose_graph_tools)

configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/pose_graph_toolsConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/pose_graph_toolsConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pose_graph_tools)
write_basic_package_version_file(
  pose_graph_toolsConfigVersion.cmake
  VERSION ${PACKAGE_VERSION}
  COMPATIBILITY AnyNewerVersion)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pose_graph_toolsConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/pose_graph_toolsConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pose_graph_tools)
