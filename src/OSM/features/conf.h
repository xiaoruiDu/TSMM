#ifndef CONF_H
#define CONF_H

#include <iostream>

namespace TSMM::OSM
{
    struct Conf {
        std::string inPath_;
        std::string outPath_;
        double disTolerance_;
    };

}// namespace TSMM::OSM


#endif//CONF_H
