cmake_minimum_required(VERSION 3.22)
project(ReemanControls LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
find_package(Qt6 REQUIRED COMPONENTS Core)
find_package(gz-gui REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz-transport REQUIRED)

add_library(ReemanControls SHARED ReemanControls.cc)
target_compile_features(ReemanControls PRIVATE cxx_std_17)
qt_add_resources(ReemanControls controls PREFIX "/ReemanControls" FILES ReemanControls.qml)
target_link_libraries(ReemanControls PRIVATE
    Qt6::Core gz-gui::gz-gui gz-msgs::gz-msgs gz-transport::gz-transport)
