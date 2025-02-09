#ifndef SIMPLIFY_OSM_H
#define SIMPLIFY_OSM_H

#include <features/conf.h>
#include <map/OSMMap.h>
#include <processor/OSMParser.h>
#include <processor/OSMProcessorBase.h>
#include <processor/bufferWays.h>
#include <processor/linkWays.h>


namespace TSMM::OSM
{

    class SimplifyOSM
    {
        OSMMap *map_;
        Conf conf_;


    public:
        explicit SimplifyOSM(Conf &conf) : map_(nullptr), conf_(std::move(conf))
        {
            map_ = new OSMMap();
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

        ~SimplifyOSM()
        {
            delete map_;
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

            map_->saveToOSM(conf_.outPath_);
        }
    };

}// namespace TSMM::OSM

#endif//SIMPLIFY_OSM_H
