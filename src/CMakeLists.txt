add_subdirectory(OSM)
set(GeneralLibs bz2 z expat proj pthread)
add_executable(osmsimplify osmSimplify.cpp)
target_link_libraries(osmsimplify ${GeneralLibs} osmcore)