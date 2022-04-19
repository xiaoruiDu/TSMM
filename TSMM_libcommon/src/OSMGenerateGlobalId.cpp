//
// Created by xiaorui on 25.02.22.
//

#include "OSMGenerateGlobalId.h"
OSMGenerateGlobalId *OSMGenerateGlobalId::instance = nullptr;

OSMGenerateGlobalId::OSMGenerateGlobalId()
{
    this->newNodeId = 0;
    this->newSubwayId = 0;
}

OSMGenerateGlobalId *OSMGenerateGlobalId::getInstance()
{
    if (!instance)
    {
        instance = new OSMGenerateGlobalId();
    }
    return instance;
}

Road::Node::id_t OSMGenerateGlobalId::getNewNodeId()
{
    newNodeId += 1;
    return newNodeId;
}

Road::SubWay::id_t OSMGenerateGlobalId::getNewSubwayId()
{
    newSubwayId += 1;
    return newSubwayId;
}

void OSMGenerateGlobalId::deleteInstance()
{
    if (instance)
    {
        delete instance;
    }
}