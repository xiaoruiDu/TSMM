#ifndef OSMPARSER_H
#define OSMPARSER_H

#include <iostream>
#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>

#include <map/OSMMap.h>

namespace TSMM::OSM{


class OSMParser : public osmium::handler::Handler{
    OSMMap *map_;

public:
    OSMParser(OSMMap* map): map_(map){}

    void node(const osmium::Node& node) {
        std::cout << "add node \n" ;
        map_->addNode(node.id(), node.location().lat(), node.location().lon());
    }

    void way(const osmium::Way& way) {
        std::cout << "add way \n" ;
        map_->addWay(way.id());
    }

    void relation(const osmium::Relation& relation) {
        std::cout << "add relation \n" ;
        map_->addRelation(relation.id());
    }

};


}

#endif //OSMPARSER_H
