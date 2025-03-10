#ifndef OSMRELATION_H
#define OSMRELATION_H
#include <memory>
#include <random>
#include <unordered_map>
namespace TSMM::OSM
{

    class OSMRelation
    {
    public:
        typedef std::int64_t id_t;


        explicit OSMRelation(id_t id) : id_(id), isActive_(true) {}
        void addMember(const std::string &type, long long refId)
        {
            members_.emplace_back(type, refId);
        }

        void addTag(const std::string &key, const std::string &value)
        {
            tags_[key] = value;
        }

        id_t id() const { return id_; }

    private:
        id_t id_;
        bool isActive_;
        std::vector<std::pair<std::string, long long>> members_;// Type ("node", "way", "relation") and ID
        std::unordered_map<std::string, std::string> tags_;
    };
}// namespace TSMM::OSM


#endif//OSMRELATION_H
