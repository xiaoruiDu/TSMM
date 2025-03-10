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
            std::vector<osmium::object_id_type> nodeRef;
            std::unordered_map<std::string, std::string> tags;
            std::for_each(way.nodes().begin(), way.nodes().end(), [&](const auto &node_ref) {
                nodeRef.push_back(node_ref.ref());// Store only node IDs
            });

            std::for_each(way.tags().begin(), way.tags().end(), [&](const auto &tag) {
                tags.insert(std::make_pair(tag.key(), tag.value()));
            });

            map_->addWay(way.id(), nodeRef, tags);
        }

        void relation(const osmium::Relation &relation)
        {
            map_->addRelation(relation.id());
        }
    };


}// namespace TSMM::OSM

#endif//OSMPARSER_H
