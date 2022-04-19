cmake_minimum_required(VERSION 3.16.3)

project(tsmm)

set(CMAKE_CXX_FLAGS "-std=c++14" )

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Osmium REQUIRED)

if (NOT OSMIUM_FOUND)
    message(WARNING "Libosmium not found!\n")
endif ()

# Find source files
file(GLOB SOURCES
        main.cpp
        TSMM_libcommon/src/*.cpp
        )

# Find header files
file(GLOB INCLUDES
        TSMM_libcommon/include/*.h
        )

set(GeneralLibs bz2 z expat proj pthread)

include_directories(${OSMIUM_INCLUDE_DIRS})
include_directories(TSMM_libcommon/include)

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${GeneralLibs})



