#ifndef OSMNODE_H
#define OSMNODE_H
#include <memory>
#include <unordered_map>
namespace TSMM::OSM
{

    class OSMNode
    {

    public:
        typedef std::int64_t id_t;


        OSMNode(id_t id, double lat, double lon) : id_(id), lat_(lat), lon_(lon), isActive_(true) {}

        void addTag(const std::string &key, const std::string &value)
        {
            tags_[key] = value;
        }

        id_t id() const { return id_; }
        double lat() const { return lat_; }
        double lon() const { return lon_; }


    private:
        id_t id_;
        double lat_, lon_;
        bool isActive_;
        std::unordered_map<std::string, std::string> tags_;
    };

}// namespace TSMM::OSM

#endif//OSMNODE_H
