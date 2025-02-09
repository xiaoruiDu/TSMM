#ifndef OSMMAP_H
#define OSMMAP_H

#include <iostream>
#include <map/OSMType.h>
#include <memory>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/handler.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/io/pbf_output.hpp>
#include <osmium/io/xml_output.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/visitor.hpp>
#include <unordered_map>

namespace TSMM::OSM
{

    class OSMMap
    {

        std::unordered_map<long long, std::unique_ptr<OSMType::Node>> nodes_;
        std::unordered_map<long long, std::unique_ptr<OSMType::Way>> ways_;
        std::unordered_map<long long, std::unique_ptr<OSMType::Relation>> relations_;
        osmium::memory::Buffer buffer_{10240, osmium::memory::Buffer::auto_grow::yes};


    public:
        void addNode(long long id, double lat, double lon)
        {
            nodes_[id] = std::make_unique<OSMType::Node>(id, lat, lon);
        }

        void addWay(long long id)
        {
            ways_[id] = std::make_unique<OSMType::Way>(id);
        }

        void addRelation(long long id)
        {
            relations_[id] = std::make_unique<OSMType::Relation>(id);
        }

        OSMType::Node *getNode(long long id)
        {
            return nodes_.count(id) ? nodes_[id].get() : nullptr;
        }

        OSMType::Way *getWay(long long id)
        {
            return ways_.count(id) ? ways_[id].get() : nullptr;
        }

        OSMType::Relation *getRelation(long long id)
        {
            return relations_.count(id) ? relations_[id].get() : nullptr;
        }

        void saveToOSM(const std::string &path)
        {
            // build node
            for (const auto &node: nodes_)
            {
                osmium::builder::NodeBuilder node_builder(buffer_);
                node_builder.set_id(node.second->id_);
                node_builder.set_location(osmium::Location{node.second->lon_, node.second->lat_});
            }
            // build way
            for (const auto &way: ways_)
            {
                osmium::builder::WayBuilder way_builder(buffer_);
                way_builder.set_id(way.second->id_);
            }

            // build relation
            for (const auto &relation: relations_)
            {
                osmium::builder::RelationBuilder relation_builder(buffer_);
                relation_builder.set_id(relation.second->id_);
            }

            osmium::io::Header header;
            header.set("test_key", "test_val");

            osmium::io::File file(path, "osm");
            osmium::io::Writer writer(file, header);
            writer(std::move(buffer_));// Write memory buffer to file
            writer.close();
        }
    };
}// namespace TSMM::OSM


#endif//OSMMAP_H
