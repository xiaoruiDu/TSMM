//
// Created by xiaorui on 25.02.22.
//

#ifndef BENCHMARK_OSMGENERATEGLOBALID_H
#define BENCHMARK_OSMGENERATEGLOBALID_H
#include "OSMCoreTypes.h"

#include <iostream>
class OSMGenerateGlobalId
{
public:
    OSMGenerateGlobalId(OSMGenerateGlobalId const &) = delete;
    OSMGenerateGlobalId &operator=(OSMGenerateGlobalId const &) = delete;

    static OSMGenerateGlobalId *getInstance();
    void deleteInstance();
    Road::Node::id_t getNewNodeId();
    Road::SubWay::id_t getNewSubwayId();

private:
    OSMGenerateGlobalId();
    static OSMGenerateGlobalId *instance;
    Road::Node::id_t newNodeId;
    Road::SubWay::id_t newSubwayId;
};


#endif// BENCHMARK_OSMGENERATEGLOBALID_H
