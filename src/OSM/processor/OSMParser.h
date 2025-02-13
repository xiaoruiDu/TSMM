#ifndef OSMPARSER_H
#define OSMPARSER_H

#include <iostream>
#include <osmium/handler.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/visitor.hpp>
#include <utility>

#include <map/OSMMap.h>

namespace TSMM::OSM
{


    class OSMParser : public osmium::handler::Handler
    {
        std::shared_ptr<OSMMap> map_;

    public:
        explicit OSMParser(const std::shared_ptr<OSMMap> &map) : map_(map) {}

        void node(const osmium::Node &node)
        {
            map_->addNode(node.id(), node.location().lat(), node.location().lon());
        }

        void way(const osmium::Way &way)
        {
            map_->addWay(way.id());
        }

        void relation(const osmium::Relation &relation)
        {
            map_->addRelation(relation.id());
        }
    };


}// namespace TSMM::OSM

#endif//OSMPARSER_H
