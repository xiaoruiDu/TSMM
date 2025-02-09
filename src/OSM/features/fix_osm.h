#ifndef FIX_OSM_H
#define FIX_OSM_H


#include <map/OSMMap.h>
#include <features/conf.h>
#include <processor/linkWays.h>
#include <processor/bufferWays.h>

namespace TSMM::OSM{

    class FixOSM{


    public:
        explicit FixOSM(const TSMM::OSM::Conf &conf)  {}

        void process(){
            // specific ways to process osmmap

        }


    };

}

#endif //FIX_OSM_H
