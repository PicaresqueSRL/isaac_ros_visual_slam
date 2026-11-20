# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22.1)
project(pq_cuvslam LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message( STATUS "Architecture: ${ARCHITECTURE}" )

find_package(Boost COMPONENTS thread chrono REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include(ExternalProject)

# Ensure CUDA imported targets (e.g., CUDA::nvtx3) are available
find_package(CUDAToolkit REQUIRED)
find_package(isaac_ros_visual_slam_interfaces REQUIRED)
find_library(ISAAC_ROS_VISUAL_SLAM_INTERFACES_TYPESUPPORT_CPP_LIBRARY
  isaac_ros_visual_slam_interfaces__rosidl_typesupport_cpp REQUIRED)
get_filename_component(CUVSLAM_CUDA_ROOT "${CUDAToolkit_BIN_DIR}" DIRECTORY)

# NVTX
option(USE_NVTX "Enable NVTX markers for improved profiling (if available)" ON)
if(USE_NVTX)
    add_definitions(-DUSE_NVTX)
    link_directories("${CUDA_TOOLKIT_ROOT_DIR}/lib64")
    link_libraries("CUDA::nvtx3")
endif()

# Dependencies
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(VISUAL_SLAM_LIBRARIES
  CUDA::cudart
  ${ISAAC_ROS_VISUAL_SLAM_INTERFACES_TYPESUPPORT_CPP_LIBRARY}
)

# Zero-copy GPU image transport through Isaac ROS cuda_buffer.
#   AUTO: enabled if the cuda_buffer package is found (e.g. Isaac ROS container)
#   ON:   required, configuration fails if cuda_buffer is missing
#   OFF:  disabled, images are passed to cuVSLAM in host memory
# cuda_buffer is intentionally not listed in package.xml, so that the package
# builds on a standard ROS 2 installation.
set(USE_CUDA_BUFFER "AUTO" CACHE STRING "Use Isaac ROS cuda_buffer for zero-copy images (AUTO/ON/OFF)")
set_property(CACHE USE_CUDA_BUFFER PROPERTY STRINGS AUTO ON OFF)
set(VSLAM_WITH_CUDA_BUFFER OFF)
if(NOT USE_CUDA_BUFFER STREQUAL "OFF")
  find_package(cuda_buffer QUIET)
  if(cuda_buffer_FOUND)
    set(VSLAM_WITH_CUDA_BUFFER ON)
  elseif(USE_CUDA_BUFFER STREQUAL "ON")
    message(FATAL_ERROR "USE_CUDA_BUFFER=ON but the cuda_buffer package was not found")
  endif()
endif()
message(STATUS "cuda_buffer zero-copy images: ${VSLAM_WITH_CUDA_BUFFER}")
if(VSLAM_WITH_CUDA_BUFFER)
  list(APPEND VISUAL_SLAM_LIBRARIES ${cuda_buffer_TARGETS})
endif()

# ---------------------------------------------------------------------------------------------
# cuVSLAM version, chosen at build time:
#   -DCUVSLAM_REF=<tag|commit>     fetch that version (e.g. v17.0.1) at configure time
#   -DCUVSLAM_SOURCE_DIR=<path>    use a local checkout
#   (neither)                      use the cuvslam submodule of the repository
# Supported versions: 15, 16, 17 (PQ_CUVSLAM_ALLOW_UNSUPPORTED=ON to try others).
set(CUVSLAM_REF "" CACHE STRING "cuVSLAM git tag or commit to build (empty: submodule)")
set(CUVSLAM_GIT_URL "https://github.com/nvidia-isaac/cuVSLAM.git" CACHE STRING
  "cuVSLAM repository used with CUVSLAM_REF")
set(CUVSLAM_SOURCE_DIR "" CACHE PATH "cuVSLAM source checkout (empty: submodule)")
option(PQ_CUVSLAM_ALLOW_UNSUPPORTED "Allow cuVSLAM versions outside 15-17" OFF)

if(CUVSLAM_REF)
  include(FetchContent)
  string(MAKE_C_IDENTIFIER "${CUVSLAM_REF}" _cuvslam_ref_id)
  set(_cuvslam_fetch_dir "${CMAKE_CURRENT_BINARY_DIR}/cuvslam-src-${_cuvslam_ref_id}")
  message(STATUS "Fetching cuVSLAM ${CUVSLAM_REF} from ${CUVSLAM_GIT_URL}")
  fetchcontent_populate(cuvslam_src_${_cuvslam_ref_id}
    QUIET
    GIT_REPOSITORY "${CUVSLAM_GIT_URL}"
    GIT_TAG "${CUVSLAM_REF}"
    SOURCE_DIR "${_cuvslam_fetch_dir}"
    SUBBUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/cuvslam-subbuild-${_cuvslam_ref_id}")
  set(CUVSLAM_SRC "${_cuvslam_fetch_dir}")
  set(CUVSLAM_BUILD_ID "${_cuvslam_ref_id}")
elseif(CUVSLAM_SOURCE_DIR)
  get_filename_component(CUVSLAM_SRC "${CUVSLAM_SOURCE_DIR}" REALPATH)
  set(CUVSLAM_BUILD_ID "local")
else()
  get_filename_component(CUVSLAM_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../cuvslam" REALPATH)
  set(CUVSLAM_BUILD_ID "submodule")
endif()
if(NOT EXISTS "${CUVSLAM_SRC}/CMakeLists.txt")
  message(FATAL_ERROR
    "cuVSLAM sources missing at ${CUVSLAM_SRC}. For the submodule run "
    "`git submodule update --init` in the repository; or set CUVSLAM_REF.")
endif()

# Version, from the VERSION file of the sources.
file(READ "${CUVSLAM_SRC}/VERSION" CUVSLAM_VERSION)
string(STRIP "${CUVSLAM_VERSION}" CUVSLAM_VERSION)
string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)" _cuvslam_version_ok "${CUVSLAM_VERSION}")
if(NOT _cuvslam_version_ok)
  message(FATAL_ERROR "cannot parse the cuVSLAM version '${CUVSLAM_VERSION}'")
endif()
set(CUVSLAM_VERSION_MAJOR ${CMAKE_MATCH_1})
set(CUVSLAM_VERSION_MINOR ${CMAKE_MATCH_2})
set(CUVSLAM_VERSION_PATCH ${CMAKE_MATCH_3})
message(STATUS "cuVSLAM version: ${CUVSLAM_VERSION} (${CUVSLAM_SRC})")
if(CUVSLAM_VERSION_MAJOR LESS 15 OR CUVSLAM_VERSION_MAJOR GREATER 17)
  if(PQ_CUVSLAM_ALLOW_UNSUPPORTED)
    message(WARNING "cuVSLAM ${CUVSLAM_VERSION} is outside the supported versions 15-17")
  else()
    message(FATAL_ERROR
      "cuVSLAM ${CUVSLAM_VERSION} is not supported (15-17). "
      "Set PQ_CUVSLAM_ALLOW_UNSUPPORTED=ON to try anyway.")
  endif()
endif()

# cuNLS solver in cuVSLAM (cuVSLAM >= 17, CUDA >= 12.6, cuBLAS, cuSOLVER); needed by the
# Multisensor mode. AUTO: ON from cuVSLAM 17 (its default), OFF before (not supported there,
# and OFF in the Isaac ROS node).
set(USE_CUNLS "AUTO" CACHE STRING "Build cuVSLAM with cuNLS (AUTO/ON/OFF)")
set_property(CACHE USE_CUNLS PROPERTY STRINGS AUTO ON OFF)
if(USE_CUNLS STREQUAL "AUTO")
  if(CUVSLAM_VERSION_MAJOR GREATER_EQUAL 17)
    set(CUVSLAM_USE_CUNLS ON)
  else()
    set(CUVSLAM_USE_CUNLS OFF)
  endif()
elseif(USE_CUNLS)
  set(CUVSLAM_USE_CUNLS ON)
else()
  set(CUVSLAM_USE_CUNLS OFF)
endif()
if(CUVSLAM_USE_CUNLS AND CUVSLAM_VERSION_MAJOR LESS 17)
  message(FATAL_ERROR "USE_CUNLS=ON requires cuVSLAM >= 17.0.0 (found ${CUVSLAM_VERSION})")
endif()
message(STATUS "cuVSLAM cuNLS: ${CUVSLAM_USE_CUNLS}")

if(CMAKE_BUILD_TYPE)
  set(CUVSLAM_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
else()
  set(CUVSLAM_BUILD_TYPE Release)
endif()

set(CUVSLAM_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/cuvslam-${CUVSLAM_BUILD_ID}")
set(CUVSLAM_LIBRARY "${CUVSLAM_BINARY_DIR}/bin/libcuvslam.so")
set(CUVSLAM_API_LAUNCHER "${CUVSLAM_BINARY_DIR}/bin/cuvslam_api_launcher")
set(CUVSLAM_GIT_SAFE_DIR_ENV
  GIT_CONFIG_COUNT=1
  GIT_CONFIG_KEY_0=safe.directory
  GIT_CONFIG_VALUE_0=${CUVSLAM_SRC}
)

externalproject_add(cuvslam_external
  SOURCE_DIR "${CUVSLAM_SRC}"
  BINARY_DIR "${CUVSLAM_BINARY_DIR}"
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CUVSLAM_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX=${CUVSLAM_BINARY_DIR}/install
    -DCUDAToolkit_ROOT=${CUVSLAM_CUDA_ROOT}
    -DCUVSLAM_BUILD_SHARED_LIB=ON
    -DUSE_CUDA=ON
    -DUSE_NVTX=${USE_NVTX}
    -DUSE_RERUN=OFF
    -DUSE_CERES=OFF
    -DUSE_CUNLS=${CUVSLAM_USE_CUNLS}
  BUILD_COMMAND ${CMAKE_COMMAND} -E env
    ${CUVSLAM_GIT_SAFE_DIR_ENV}
    ${CMAKE_COMMAND} --build <BINARY_DIR> --target cuvslam_api_launcher
  INSTALL_COMMAND ""
  BUILD_BYPRODUCTS
    "${CUVSLAM_LIBRARY}"
    "${CUVSLAM_API_LAUNCHER}"
)

add_library(cuvslam SHARED IMPORTED GLOBAL)
set_target_properties(cuvslam PROPERTIES
  IMPORTED_LOCATION "${CUVSLAM_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${CUVSLAM_SRC}/libs"
)
add_dependencies(cuvslam cuvslam_external)

# visual_slam_node
ament_auto_add_library(
  visual_slam_node SHARED
  src/visual_slam_node.cpp
  src/impl/cuvslam_compat.cpp
  src/impl/cuvslam_ros_conversion.cpp
  src/impl/landmarks_vis_helper.cpp
  src/impl/latency_emulator.cpp
  src/impl/localizer_vis_helper.cpp
  src/impl/motion_compensation.cpp
  src/impl/pose_cache.cpp
  src/impl/profile.cpp
  src/impl/posegraph_vis_helper.cpp
  src/impl/qos.cpp
  src/impl/run_output.cpp
  src/impl/visual_slam_impl.cpp
  src/impl/viz_helper.cpp
)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
target_link_libraries(visual_slam_node
  cuvslam
  yaml-cpp::yaml-cpp
  opencv_core
  opencv_imgproc
  Boost::thread
  Boost::chrono
  ${VISUAL_SLAM_LIBRARIES}
)
add_dependencies(visual_slam_node cuvslam_external)
if(VSLAM_WITH_CUDA_BUFFER)
  target_compile_definitions(visual_slam_node PRIVATE VSLAM_WITH_CUDA_BUFFER)
endif()

# Commit of the wrapper (git describe), written to run_info.json. Read at every build into a
# generated header (cmake/git_version.cmake).
set(PQ_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
add_custom_target(pq_cuvslam_git_version ALL
  COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    -DOUTPUT=${PQ_GENERATED_DIR}/pq_cuvslam/git_version.hpp
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/git_version.cmake
  BYPRODUCTS ${PQ_GENERATED_DIR}/pq_cuvslam/git_version.hpp
  COMMENT "Reading the pq_cuvslam commit")
add_dependencies(visual_slam_node pq_cuvslam_git_version)
target_include_directories(visual_slam_node PRIVATE ${PQ_GENERATED_DIR})
# cuVSLAM version and features of the build, for the adaptation layer (impl/cuvslam_compat).
target_compile_definitions(visual_slam_node PUBLIC
  PQ_CUVSLAM_VERSION_MAJOR=${CUVSLAM_VERSION_MAJOR}
  PQ_CUVSLAM_VERSION_MINOR=${CUVSLAM_VERSION_MINOR}
  PQ_CUVSLAM_VERSION_PATCH=${CUVSLAM_VERSION_PATCH}
  PQ_CUVSLAM_WITH_CUNLS=$<BOOL:${CUVSLAM_USE_CUNLS}>)
rclcpp_components_register_nodes(visual_slam_node "pq::visual_slam::VisualSlamNode")
set(node_plugins "${node_plugins}pq::visual_slam::VisualSlamNode;$<TARGET_FILE:visual_slam_node>\n")

# pq_cuvslam executable
ament_auto_add_executable(${PROJECT_NAME}
  src/visual_slam_main.cpp
)
target_link_libraries(${PROJECT_NAME} visual_slam_node)

# Deterministic bag replay executable
ament_auto_add_executable(pq_cuvslam_replay
  src/replay_main.cpp
)
target_link_libraries(pq_cuvslam_replay visual_slam_node)

# API launcher executable
install(PROGRAMS
  ${CUVSLAM_API_LAUNCHER}
  DESTINATION lib/${PROJECT_NAME}
)

# Install cuVSLAM
install(FILES ${CUVSLAM_LIBRARY} DESTINATION lib)

set_target_properties(visual_slam_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test_message_stream_sequencer test/test_message_stream_sequencer.cpp)
  target_include_directories(${PROJECT_NAME}_test_message_stream_sequencer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_add_gtest(${PROJECT_NAME}_test_device_frame_synchronizer
    test/test_device_frame_synchronizer.cpp)
  target_include_directories(${PROJECT_NAME}_test_device_frame_synchronizer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_add_gtest(${PROJECT_NAME}_test_motion_compensation test/test_motion_compensation.cpp)
  target_link_libraries(${PROJECT_NAME}_test_motion_compensation visual_slam_node)

  # Tests of the vendored isaac_common headers
  foreach(_test test_message_buffer test_message_stream_synchronizer)
    ament_add_gtest(${PROJECT_NAME}_${_test} test/${_test}.cpp)
    target_include_directories(${PROJECT_NAME}_${_test} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  endforeach()
endif()

ament_auto_package(INSTALL_TO_SHARE launch profiles rviz)
