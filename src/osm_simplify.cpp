#include <features/conf.h>
#include <features/simplify_osm.h>
#include <iostream>


int main()
{
    // note: please use absolute path otherwise osmium can't parse the file.
    auto conf =TSMM::OSM::Conf{.inPath_ = "/home/amie/Project/TSMM/test/data/test_map_1.osm", .outPath_ = "/home/amie/Project/TSMM/test/output/test_map_1_output.osm", .disTolerance_ = 1.0,};
    auto simplifyOSM = TSMM::OSM::SimplifyOSM(conf);
    simplifyOSM.process();
    std::cout << "test success!" << std::endl;
    return 0;
}