# Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

cmake_minimum_required(VERSION 3.23)
project(ptcee LANGUAGES CXX VERSION 1.0.0)
set(library_name "ptcee")

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Sophus REQUIRED)

add_library(${library_name})

target_link_libraries(${library_name}
  PUBLIC Eigen3::Eigen
  PUBLIC gtsam::gtsam
  PUBLIC Sophus::Sophus
)

file(GLOB_RECURSE HEADER_FILES include/*.h)
file(GLOB_RECURSE SRC_FILES src/*.cpp)

target_sources(${library_name}
  PUBLIC FILE_SET public_headers TYPE HEADERS BASE_DIRS include FILES ${HEADER_FILES}
  PRIVATE ${SRC_FILES}
)

set_target_properties(${library_name} PROPERTIES
  PUBLIC_HEADER "${HEADER_FILES}"
  CXX_STANDARD_REQUIRED ON
  CXX_STANDARD 20
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
)

include(cmake/compiler-flags.cmake)
add_compiler_flags(${library_name})

include(cmake/coverage-flags.cmake)
add_coverage_flags(${library_name})

install(TARGETS ${library_name}
  FILE_SET public_headers
)
