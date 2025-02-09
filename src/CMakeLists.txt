
add_subdirectory(OSM)
set(GeneralLibs bz2 z expat proj pthread)
add_executable(osmsimplify osm_simplify.cpp)
target_link_libraries(osmsimplify ${GeneralLibs} osmcore)