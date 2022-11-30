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

# to avoid warning about DOWNLOAD_EXTRACT_TIMESTAMP:
set(CMAKE_POLICY_DEFAULT_CMP0135 NEW)

project(
  sklmpu9255
  LANGUAGES CXX
  VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 11)

option(USE_IMU "Use IMU hardware." ON)
option(USE_SDL "Use SDL2 for 'show_rpy' example." ON)
option(USE_THREAD "Use threaded code for 'show_rpy' example." ON)
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

add_library(sklmpu9255 STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/mpu.cpp
                              ${CMAKE_CURRENT_SOURCE_DIR}/src/ak8963.cpp
                              ${CMAKE_CURRENT_SOURCE_DIR}/src/mpu925x.cpp
                              ${CMAKE_CURRENT_SOURCE_DIR}/src/qfilter.cpp)
target_include_directories(sklmpu9255
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
set_target_properties(sklmpu9255
    PROPERTIES PUBLIC_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/src/sklmpu9255.h")
# Make sure the OBJECT libraries are built with position independent code (it is
# not ON by default).
set_target_properties(sklmpu9255 PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Set the version numbers.
set_target_properties(sklmpu9255 PROPERTIES VERSION ${PROJECT_VERSION}
                                 SOVERSION ${PROJECT_VERSION})
target_include_directories(
  sklmpu9255 INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
             $<INSTALL_INTERFACE:include>)

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
  if (SDL2_FOUND)
    include_directories(${SDL2_INCLUDE_DIRS})
    target_compile_definitions(show_rpy PUBLIC USE_SDL)
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
  enable_testing()

  include(GoogleTest OPTIONAL)
  if(EXISTS "${GTEST_SOURCE_DIR}/CMakeLists.txt")
    message(STATUS "Using ${GTEST_SOURCE_DIR}/CMakeLists.txt.")
    add_subdirectory(${GTEST_SOURCE_DIR} ${GTEST_BUILD_DIR} EXCLUDE_FROM_ALL)
  else()
    find_package(GTest QUIET)
    if(GTEST_FOUND)
      message(STATUS "Found installed CMake Google Test package.")
    else()
      include(FetchContent)
      # release 1.12.1
      FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/58d77fa8070e8cec2dc1ed015d66b454c8d78850.zip
      )
      # For Windows: Prevent overriding the parent project's compiler/linker settings
      set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
      FetchContent_MakeAvailable(googletest)
    endif()
  endif()

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

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Install the different headers and libraries.
install(
  TARGETS sklmpu9255
  EXPORT ${PROJECT_NAME}Targets
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

set(ConfigPackageLocation ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake/)
install(EXPORT ${PROJECT_NAME}Targets NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${ConfigPackageLocation})

# Create the CMake version file.
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)

# Create the Config file.
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation})

# Install the generated CMake files.
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        DESTINATION ${ConfigPackageLocation})

################################################################################
