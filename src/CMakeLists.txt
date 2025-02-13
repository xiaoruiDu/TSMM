list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(Osmium REQUIRED)

if (NOT OSMIUM_FOUND)
    message(WARNING "Libosmium not found!\n")
endif ()

include_directories(${OSMIUM_INCLUDE_DIRS})

add_subdirectory(OSM)
set(GeneralLibs bz2 z expat proj pthread)
add_executable(osmsimplify osmSimplify.cpp)
target_link_libraries(osmsimplify ${GeneralLibs} osmcore)