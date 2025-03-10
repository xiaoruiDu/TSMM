#ifndef OSMMAP_H
#define OSMMAP_H

#include <iostream>
#include <map/OSMNode.h>
#include <map/OSMRelation.h>
#include <map/OSMWay.h>
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

        std::unordered_map<OSMNode::id_t, std::shared_ptr<OSMNode>> nodes_;
        std::unordered_map<OSMWay::id_t, std::shared_ptr<OSMWay>> ways_;
        std::unordered_map<OSMRelation::id_t, std::shared_ptr<OSMRelation>> relations_;
        osmium::memory::Buffer buffer_{10240, osmium::memory::Buffer::auto_grow::yes};

        void buildNodes(osmium::memory::Buffer &buffer)
        {
            std::for_each(nodes_.begin(), nodes_.end(), [&](const auto &node) {
                {
                    osmium::builder::NodeBuilder builder{buffer};
                    builder.set_user("tsmm");
                    osmium::Node &obj = builder.object();

                    obj.set_id(node.second->id());
                    obj.set_uid(node.second->id());
                    obj.set_location(osmium::Location{node.second->lon(), node.second->lat()});
                }
                buffer.commit();
            });
        }

        void buildWays(osmium::memory::Buffer &buffer)
        {
            std::for_each(ways_.begin(), ways_.end(), [&](const auto &way) {
                if (way.second->isActive())
                {
                    {
                        osmium::builder::WayBuilder builder{buffer};
                        builder.set_user("tsmm");
                        osmium::Way &obj = builder.object();

                        obj.set_id(way.second->id());
                        obj.set_uid(way.second->id());

                        {
                            ///< add node ref
                            osmium::builder::WayNodeListBuilder wayNodeListBuilder{buffer, &builder};
                            std::for_each(way.second->nodeRefs().begin(), way.second->nodeRefs().end(), [&](const auto &nId) {
                                osmium::Location node(nodes_[nId]->lon(), nodes_[nId]->lat());
                                osmium::NodeRef nodeRef{nId, node};
                                wayNodeListBuilder.add_node_ref(nodeRef);
                            });
                        }

                        {
                            ///< add way tags
                            osmium::builder::TagListBuilder tl_builder{buffer, &builder};
                            // add road level tag
                            std::for_each(way.second->tags().begin(), way.second->tags().end(), [&](const auto &tags) {
                                tl_builder.add_tag(tags.first, tags.second);
                            });
                        }
                    }
                    buffer.commit();
                }
            });
        }


        void buildRelations(osmium::memory::Buffer &buffer)
        {}

    public:
        void addNode(OSMWay::id_t id, double lat, double lon)
        {
            nodes_[id] = std::make_shared<OSMNode>(id, lat, lon);
        }

        void addWay(OSMWay::id_t id, std::vector<osmium::object_id_type> &nodeRef, std::unordered_map<std::string, std::string> &tags)
        {

            ways_[id] = std::make_shared<OSMWay>(id, nodeRef, tags);
        }

        void addRelation(OSMRelation::id_t id)
        {
            relations_[id] = std::make_shared<OSMRelation>(id);
        }


        std::unordered_map<OSMWay::id_t, std::shared_ptr<OSMWay>> ways() const
        {
            return ways_;
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
