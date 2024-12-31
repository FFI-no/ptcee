cmake_minimum_required(VERSION 3.15)
project(PackageTest CXX)

find_package(ptcee REQUIRED)

add_executable(example src/test-package.cpp)
target_link_libraries(example
  ptcee::ptcee
)
