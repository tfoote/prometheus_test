cmake_minimum_required(VERSION 3.8)
project(prometheus_cpp_vendor)


option(FORCE_BUILD_VENDOR_PKG
  "Build prometheus-cpp from source, even if system-installed package is available"
  OFF)

find_package(ament_cmake REQUIRED)
# ament_add_default_options()

include(ExternalProject)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


macro(build_prometheus_cpp)
  set(extra_cmake_args)

  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()
  if(NOT WIN32)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -w")
  endif()

  list(APPEND extra_cmake_args "-DENABLE_COMPRESSION=OFF")
  list(APPEND extra_cmake_args "-DENABLE_PUSH=ON")
  list(APPEND extra_cmake_args "-DBUILD_SHARED_LIBS=ON")
  #if(BUILD_SHARED_LIBS)
  #  list(APPEND extra_cmake_args "-DBUILD_SHARED_LIBS=ON")
  #endif()
  list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
  list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")

  if(WIN32 AND NOT ${CMAKE_VERBOSE_MAKEFILE})
    set(should_log ON)  # prevent warnings in Windows CI
  else()
    set(should_log OFF)
  endif()

  if(DEFINED CMAKE_TOOLCHAIN_FILE)
    list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
  endif()


  externalproject_add(prometheus-cpp
    GIT_REPOSITORY https://github.com/jupp0r/prometheus-cpp.git
    GIT_TAG 76470b3ec024c8214e1f4253fb1f4c0b28d3df94 # Tag v1.0.1
    # GIT_CONFIG advice.detachedHead=false
    # Suppress git update due to https://gitlab.kitware.com/cmake/cmake/-/issues/16419
    # See https://github.com/ament/uncrustify_vendor/pull/22 for details
    # UPDATE_COMMAND ""
    TIMEOUT 600
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/prometheus-cpp_install
      ${extra_cmake_args}
    # INSTALL_COMMAND make install
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/prometheus-cpp_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
    USE_SOURCE_PERMISSIONS
    PATTERN config.h EXCLUDE
  )
endmacro()


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

ament_export_include_directories(include)

# NO_CMAKE_PACKAGE_REGISTRY used to avoid finding the library downloaded in WORKSPACE B
# when building workspace A.
# This should only find a system installed prometheus-cpp and thus the environment hook isn't needed.
find_package(prometheus-cpp QUIET NO_CMAKE_PACKAGE_REGISTRY)
if(FORCE_BUILD_VENDOR_PKG OR NOT prometheus-cpp_FOUND)
  build_prometheus_cpp()

else()
  message(STATUS "Found prometheus-cpp ${prometheus-cpp_VERSION} in path ${prometheus-cpp_CONFIG}")
endif()

ament_package()


