#ifndef OSMWAY_H
#define OSMWAY_H
#include <memory>
#include <osmium/builder/osm_object_builder.hpp>
#include <unordered_map>
namespace TSMM::OSM
{
    class OSMWay
    {

    public:
        typedef std::int64_t id_t;

        OSMWay(id_t id, std::vector<osmium::object_id_type> &nodeRef, std::unordered_map<std::string, std::string> &tags)
            : id_(id), isActive_(true), nodeRefs_(std::move(nodeRef)), tags_(std::move(tags)) {}

        void addNode(long long nodeId)
        {
            nodeRefs_.push_back(nodeId);
        }

        void addTag(const std::string &key, const std::string &value)
        {
            tags_[key] = value;
        }

        id_t id() const { return id_; }

        void deactive() { isActive_ = false; }
        bool isActive() const { return isActive_; }

        const std::unordered_map<std::string, std::string> &tags() const
        {
            return tags_;
        }

        const std::vector<osmium::object_id_type>& nodeRefs() const
        {
            return nodeRefs_;
        }

    private:
        id_t id_;
        bool isActive_;
        std::vector<osmium::object_id_type> nodeRefs_;// Stores references to OSMNode IDs
        std::unordered_map<std::string, std::string> tags_;
    };
}// namespace TSMM::OSM

#endif//OSMWAY_H
