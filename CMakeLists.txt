cmake_minimum_required(VERSION 3.16.3)

project(TSMM)

set(CMAKE_CXX_FLAGS "-std=c++17")

add_definitions(-Wfatal-errors) # stops further building at first error
add_definitions(-DQT_NO_KEYWORDS) # required for better error messages
include_directories(3rd)

add_subdirectory(src)



print_disabled_features(fas
gdsag
)