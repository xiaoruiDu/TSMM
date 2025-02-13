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

        std::unordered_map<long long, std::shared_ptr<OSMType::Node>> nodes_;
        std::unordered_map<long long, std::shared_ptr<OSMType::Way>> ways_;
        std::unordered_map<long long, std::shared_ptr<OSMType::Relation>> relations_;
        osmium::memory::Buffer buffer_{10240, osmium::memory::Buffer::auto_grow::yes};

        void buildNodes(osmium::memory::Buffer &buffer)
        {
            for (const auto &node: nodes_)
            {
                {
                    osmium::builder::NodeBuilder builder{buffer};
                    builder.set_user("tsmm");
                    osmium::Node &obj = builder.object();

                    obj.set_id(node.second->id_);
                    obj.set_uid(node.second->id_);
                    obj.set_location(osmium::Location{node.second->lon_, node.second->lat_});
                }
                buffer.commit();
            }
        }

        void buildWays(osmium::memory::Buffer &buffer)
        {
            for (const auto &way: ways_)
            {
                {
                    osmium::builder::WayBuilder builder{buffer};
                    builder.set_user("tsmm");
                    osmium::Way &obj = builder.object();

                    obj.set_id(way.second->id_);
                    obj.set_uid(way.second->id_);
                }
                buffer.commit();
            }
        }

        void buildRelations(osmium::memory::Buffer &buffer)
        {}

    public:
        void addNode(long long id, double lat, double lon)
        {
            nodes_[id] = std::make_shared<OSMType::Node>(id, lat, lon);
        }

        void addWay(long long id)
        {
            ways_[id] = std::make_shared<OSMType::Way>(id);
        }

        void addRelation(long long id)
        {
            relations_[id] = std::make_shared<OSMType::Relation>(id);
        }


        void save(const std::string &path)
        {
            std::size_t bufferSize = 10240;
            osmium::memory::Buffer nodeBuffer{bufferSize, osmium::memory::Buffer::auto_grow::yes};
            osmium::memory::Buffer wayBuffer{bufferSize, osmium::memory::Buffer::auto_grow::yes};

            buildNodes(nodeBuffer);
            buildWays(wayBuffer);


            osmium::io::File file{path};
            osmium::io::Writer writer{file};
            writer(std::move(nodeBuffer));
            writer(std::move(wayBuffer));
            writer.close();

            std::cout << "OSM file created: output.osm.pbf\n";
        }
    };
}// namespace TSMM::OSM


#endif//OSMMAP_H
