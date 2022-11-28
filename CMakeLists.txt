# Copyright 2022 Pascal 'skal' Massimino
#
# MIT License
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.5)

project(
  sklmpu9255
  LANGUAGES CXX
  VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 11)

option(USE_IMU "Use IMU hardware." ON)
option(USE_SDL "Use SDL2 for 'show_rpy' example." ON)
option(USE_THREAD "Use pthread for 'show_rpy' example." ON)
option(FAKE_I2C "Use Fake I2C code (debugging)." OFF)
option(BUILD_TESTS "build and run tests" ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "Build type: Release, Debug or RelWithDebInfo" FORCE)
endif()

include(CheckCXXCompilerFlag)

if(USE_THREAD)
  find_package(Threads)
endif()

################################################################################

add_library(sklmpu9255 OBJECT src/sklmpu9255.cpp)
target_include_directories(sklmpu9255
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

if(FAKE_I2C)
  target_compile_definitions(sklmpu9255 PUBLIC FAKE_I2C)
endif()

################################################################################
# 'show_rpy' examples (requires SDL2)

add_executable(show_rpy examples/show_rpy.cpp)
target_link_libraries (show_rpy LINK_PUBLIC sklmpu9255)
target_include_directories(show_rpy
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

if(USE_SDL)
  find_package(SDL2)
  if (FOUND_SDL)
    include_directories(${SDL2_INCLUDE_DIRS})
    target_compile_definitions(show_rpy PUBLIC HAVE_SDL)
    target_link_libraries(show_rpy LINK_PUBLIC ${SDL2_LIBRARIES})
  endif()
endif()

if(USE_THREAD)
  target_link_libraries(show_rpy LINK_PUBLIC pthread)
  target_compile_definitions(show_rpy PUBLIC USE_THREAD)
endif()

################################################################################
# tests

if(BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/58d77fa8070e8cec2dc1ed015d66b454c8d78850.zip # release 1.12.1
    DOWNLOAD_EXTRACT_TIMESTAMP true
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  include(GoogleTest)

  # test_mpu
  add_executable(test_mpu tests/test_mpu.cpp)
  target_link_libraries(test_mpu LINK_PUBLIC sklmpu9255 GTest::gtest_main)
  target_include_directories(test_mpu
                             PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME test_mpu
           COMMAND ${CMAKE_CURRENT_BINARY_DIR}/test_mpu
           WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  gtest_discover_tests(test_mpu)
endif()

################################################################################
# install

install(TARGETS sklmpu9255
        EXPORT ${PROJECT_NAME}_targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

set(ConfigPackageLocation lib/${PROJECT_NAME}/cmake/)

# Install the exports.
install(
  EXPORT ${PROJECT_NAME}_targets
  NAMESPACE ${PROJECT_NAME}::
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation})

# Create the CMake version file.
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/sklmpu9255ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)

# Create the Config file.
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation lib/${PROJECT_NAME}/cmake/)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake INSTALL_DESTINATION
  ${ConfigPackageLocation})

# Install the generated CMake files.
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/sklmpu9255ConfigVersion.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/sklmpu9255Config.cmake"
        DESTINATION ${ConfigPackageLocation})

################################################################################