#ifndef OSMMAP_H
#define OSMMAP_H

#include <iostream>
#include <map/OSMType.h>
#include <memory>
#include <unordered_map>

namespace TSMM::OSM
{

    class OSMMap
    {

        std::unordered_map<long long, std::unique_ptr<OSMType::Node>> nodes_;
        std::unordered_map<long long, std::unique_ptr<OSMType::Way>> ways_;
        std::unordered_map<long long, std::unique_ptr<OSMType::Relation>> relations_;

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

        void store(const std::string &path) {}
    };
}// namespace TSMM::OSM


#endif//OSMMAP_H
