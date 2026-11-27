cmake_minimum_required(VERSION 3.22)
project(reeman_direct_devices LANGUAGES CXX)

find_package(gz-sim 10 REQUIRED)
find_package(gz-transport 15 REQUIRED)

add_library(reeman_direct_devices SHARED direct_devices.cpp)
target_compile_features(reeman_direct_devices PRIVATE cxx_std_17)
target_link_libraries(reeman_direct_devices PRIVATE gz-sim::gz-sim gz-transport::gz-transport)
