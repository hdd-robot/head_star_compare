cmake_minimum_required(VERSION 3.0.2)
project(myproject)
set (CMAKE_CXX_STANDARD 17)


find_package(Boost REQUIRED COMPONENTS regex)
include_directories(${Boost_INCLUDE_DIRS})

find_package(PkgConfig)
#pkg_check_modules(GTKMM gtkmm-3.0)

#link_directories(${GTKMM_LIBRARY_DIRS})
#include_directories(include ${GTKMM_INCLUDE_DIRS})

file(GLOB SOURCES "*.cpp")

add_executable(algo_nav ${SOURCES})

#target_link_libraries(nav_platform ${GTKMM_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(nav_platform  ${Boost_LIBRARIES})

