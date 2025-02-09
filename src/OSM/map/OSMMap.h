#ifndef OSMMAP_H
#define OSMMAP_H

#include <iostream>
#include <map/OSMType.h>
#include <memory>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/pbf_output.hpp>
#include <osmium/io/xml_output.hpp>
#include <osmium/memory/buffer.hpp>
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
            std::size_t bufferSize = 10240;
            osmium::memory::Buffer nodeBuffer{bufferSize, osmium::memory::Buffer::auto_grow::yes};

            for (const auto &node: nodes_)
            {
                {
                    osmium::builder::NodeBuilder builder{nodeBuffer};
                    builder.set_user("tsmm");
                    osmium::Node &obj = builder.object();

                    obj.set_id(node.second->id_);
                    obj.set_uid(node.second->id_);
                    obj.set_location(osmium::Location{node.second->lon_, node.second->lat_});
                }
                nodeBuffer.commit();
            }

            osmium::io::File file{path};
            osmium::io::Writer writer{file};
            writer(std::move(nodeBuffer));
            writer.close();

            std::cout << "OSM file created: output.osm.pbf\n";
        }
    };
}// namespace TSMM::OSM


#endif//OSMMAP_H
