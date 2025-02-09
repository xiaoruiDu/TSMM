#ifndef NODEPROCESSOR_H
#define NODEPROCESSOR_H

#include <processor/OSMProcessorBase.h>

namespace TSMM::OSM
{

    class LinkWays : public OSMProcessorBase
    {

    private:
        // process node, ways and relation
        void nodeProcess(OSMMap &map);
        void wayProcess(OSMMap &map);
        void relationProcess(OSMMap &map);

    public:
        void process(OSMMap &map) override {}
    };

}// namespace TSMM::OSM


#endif//NODEPROCESSOR_H
