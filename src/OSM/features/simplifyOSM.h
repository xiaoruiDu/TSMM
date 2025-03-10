#ifndef SIMPLIFY_OSM_H
#define SIMPLIFY_OSM_H

#include <features/conf.h>
#include <map/OSMMap.h>
#include <memory>
#include <processor/OSMParser.h>
#include <processor/OSMProcessorBase.h>
#include <processor/bufferWays.h>
#include <processor/filterWays.h>
#include <processor/linkWays.h>
#include <unordered_set>

namespace TSMM::OSM
{
    static inline std::unordered_set<std::string> leftWayTags{"motorway", "trunk", "primary", "secondary", "tertiary", "residential"};

    bool isWayFiltered(std::shared_ptr<OSMWay> &way)
    {
        std::string keyString = "highway";
        const auto waytags = way->tags();
        if (waytags.count(keyString))
        {
            if (leftWayTags.count(waytags.at(keyString)))
                return false;
            return true;
        }
        return true;
    }

    class SimplifyOSM
    {
        std::shared_ptr<OSMMap> map_;
        Conf conf_;


    public:
        explicit SimplifyOSM(Conf &conf) : map_(std::make_shared<OSMMap>()), conf_(std::move(conf))
        {
            try
            {
                osmium::io::Reader reader(conf_.inPath_, osmium::osm_entity_bits::node | osmium::osm_entity_bits::way | osmium::osm_entity_bits::relation);
                auto osmParser = OSMParser(map_);
                osmium::apply(reader, osmParser);
                reader.close();
            } catch (const std::exception &e)
            {
                std::cerr << "Error: Failed to open OSM file '" << conf_.inPath_ << "'.\n";
                std::cerr << "Exception: " << e.what() << "\n";
                exit(1);// Exit with error code
            }
        }

        void process()
        {
            // specific ways to process osmmap
            // for example: linkways --> buffer ways --> store the map
            std::vector<OSMProcessorBase *> processPipeLine;
            OSMProcessorBase *wayFilter = new FilterWays(isWayFiltered);
            OSMProcessorBase *linkWays = new LinkWays();
            OSMProcessorBase *bufferWays = new BufferWays();
            processPipeLine.push_back(wayFilter);
            processPipeLine.push_back(linkWays);
            processPipeLine.push_back(bufferWays);

            std::for_each(processPipeLine.begin(), processPipeLine.end(), [&](auto &pipe) { pipe->process(*map_); });
            map_->save(conf_.outPath_);
        }
    };

}// namespace TSMM::OSM

#endif//SIMPLIFY_OSM_H
