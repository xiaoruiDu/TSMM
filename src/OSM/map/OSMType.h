#ifndef TYPE_H
#define TYPE_H
#include<unordered_map>
#include<vector>


namespace TSMM::OSMType
{
    // OSM Node: Represents a point with latitude, longitude, and tags
    struct Node {
        long long id_;
        double lat_, lon_;
        std::unordered_map<std::string, std::string> tags_;

        Node(long long id, double lat, double lon) : id_(id), lat_(lat), lon_(lon) {}

        void addTag(const std::string &key, const std::string &value) {
            tags_[key] = value;
        }
    };

    // OSM Way: Represents a series of nodes (roads, rivers, buildings, etc.)
    struct Way {
        long long id_;
        std::vector<long long> nodeRefs_; // Stores references to OSMNode IDs
        std::unordered_map<std::string, std::string> tags_;

        explicit Way(long long id) : id_(id) {}

        void addNode(long long nodeId) {
            nodeRefs_.push_back(nodeId);
        }

        void addTag(const std::string &key, const std::string &value) {
            tags_[key] = value;
        }
    };

    // OSM Relation: A group of nodes, ways, or other relations
    struct Relation {
        long long id_;
        std::vector<std::pair<std::string, long long>> members_; // Type ("node", "way", "relation") and ID
        std::unordered_map<std::string, std::string> tags_;

        explicit Relation(long long id) : id_(id) {}

        void addMember(const std::string &type, long long refId) {
            members_.emplace_back(type, refId);
        }

        void addTag(const std::string &key, const std::string &value) {
            tags_[key] = value;
        }
    };

}


#endif //TYPE_H
