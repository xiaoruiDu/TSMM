

#include <iostream>
// Allow any format of input files (XML, PBF, ...)
#include <osmium/io/any_input.hpp>
#include <osmium/visitor.hpp>

// For the location index. There are different types of indexes available.
// This will work for all input files keeping the index in memory.
#include <osmium/index/map/flex_mem.hpp>

// For the NodeLocationForWays handler

#include "OSMManager.h"
//#include "matplotlibcpp.h"

#include <osmium/handler/node_locations_for_ways.hpp>

// The type of index used. This must match the include file above
using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;

// The location handler always depends on the index type
using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;

int main(int argc, char* argv[]) {

    std::string osmPath = "/home/xiaorui/PHD/TSMM_git/TSMM/binjiang_qgis.osm";  /// change it to your osm Path
    OSMManager* osmManager = new OSMManager(osmPath);
    osmManager->initialize();
    delete osmManager;
}
