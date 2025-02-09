#include <features/conf.h>
#include <features/simplify_osm.h>
#include <iostream>


int main()
{
    auto conf =TSMM::OSM::Conf{.inPath_ = "/home/amie/Project/TSMM/test/data/test_map_1.osm", .outPath_ = "", .disTolerance_ = 1.0};
    auto simplifyOSM = TSMM::OSM::SimplifyOSM(conf);
    simplifyOSM.process();
    std::cout << "test success!" << std::endl;
    return 0;
}