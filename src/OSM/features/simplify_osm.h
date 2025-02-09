#ifndef SIMPLIFY_OSM_H
#define SIMPLIFY_OSM_H

#include <features/conf.h>
#include <map/OSMMap.h>
#include <processor/OSMProcessorBase.h>
#include <processor/bufferWays.h>
#include <processor/linkWays.h>


namespace TSMM::OSM
{

    class SimplifyOSM
    {
        OSMMap *map_;
        Conf *conf_;

    public:
        explicit SimplifyOSM(Conf &conf) : map_(nullptr), conf_(nullptr)
        {
            // load osm file with osmium lib, and generate OSMMap
            // load config for specific tweak while processing
        }

        ~SimplifyOSM()
        {
            delete map_;
            delete conf_;
        }

        void process()
        {
            // specific ways to process osmmap
            // for example: linkways --> buffer ways --> store the map
            std::vector<OSMProcessorBase *> processPipeLine;
            OSMProcessorBase *linkWays = new LinkWays();
            OSMProcessorBase *bufferWays = new BufferWays();
            processPipeLine.push_back(linkWays);
            processPipeLine.push_back(bufferWays);

            for (auto &pipe: processPipeLine)
                pipe->process(*map_);

            map_->store(conf_->outPath_);
        }
    };

}// namespace TSMM::OSM

#endif//SIMPLIFY_OSM_H
