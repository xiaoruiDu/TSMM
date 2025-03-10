#ifndef OSMPROCESSORBASE_H
#define OSMPROCESSORBASE_H


#include <map/OSMMap.h>

namespace TSMM::OSM
{

    class OSMProcessorBase
    {

    public:
        OSMProcessorBase() = default;
        virtual ~OSMProcessorBase() = default;
        virtual void process(OSMMap &map) = 0;
    };
}// namespace TSMM::OSM

#endif//OSMPROCESSORBASE_H
