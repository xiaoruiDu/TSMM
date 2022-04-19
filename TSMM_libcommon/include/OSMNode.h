//
// Created by xiaorui on 16.02.22.
//

#ifndef BENCHMARK_OSMNODE_H
#define BENCHMARK_OSMNODE_H

#include "OSMCoreTypes.h"
#include "SxyPosition.h"

#include <iostream>
#include <memory>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/visitor.hpp>
#include <set>

class OSMNode {
public:
    typedef std::shared_ptr<OSMNode> ptr;

    OSMNode();

    ~OSMNode();

    /**
     * @brief constructs a new SRoadNode object, by assigning the ID and a position in the network
     *
     * @param nodeID
     * @param position
     */
    OSMNode(const Road::Node::id_t nodeId, const SlonlatPosition::ptr position);

    void setNodeTags(std::map<std::string, std::string> nodeTagsMap);

    void setIntersectionLevel(std::set<std::string> intersectionLevel);

    void insertIntersectionLevel(const std::set<std::string>& intersectionLevelSet);

    void setSubWayParentId(Road::SubWay::id_t subwayGlobalId);

    ///< get methods
    const std::map<std::string, std::string>& getNodeTags();

    const SlonlatPosition::ptr getSlonlatPosition();

    const double getNodeLat();

    const double getNodeLon();

    const Road::Node::id_t getNodeId();

    const osmium::Location getOsmLocation();

    const std::set<std::string>& getIntersectionLevel();

    const std::set<Road::SubWay::id_t>& getSubWayParentId();

    ///< set methods

    ///< update methods
    void removeParentSubWayId(Road::SubWay::id_t parentId);
    void updateParentSubWayId(Road::Way::id_t parentId);

    /// TODO < add methods
    void addNodeTag(std::map<std::string, std::string> nodeTagsMap);

    ///< others
public:
    int mmTest = 0;

private:
    std::map<std::string, std::string> m_nodeTagsMap;
    Road::Node::id_t m_nodeId;
    double lat;
    double lon;
    SlonlatPosition::ptr m_lonlat;
    osmium::Location m_osmNodeLocation;
    std::set<std::string> m_intersectionLevel; /// only calculated intersection nodes have this attribute.
    std::set<Road::SubWay::id_t> m_parentId;
};

#endif // BENCHMARK_OSMNODE_H
