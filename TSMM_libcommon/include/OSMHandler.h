//
// Created by xiaorui on 09.11.21.
//

#ifndef BENCHMARK_OSMHANDLER_H
#define BENCHMARK_OSMHANDLER_H

#include <iostream>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/visitor.hpp>
#include <string>

// For the location index. There are different types of indexes available.
// This will work for all input files keeping the index in memory.
#include <osmium/index/map/flex_mem.hpp>
// Allow any format of input files (XML, PBF, ...)
#include "OSMCoreTypes.h"
#include "OSMGenerateGlobalId.h"
#include "OSMLine.h"
#include "OSMMath.h"
#include "OSMNode.h"
#include "OSMVariableName.h"
#include "OSMWay.h"
#include "Utility.h"

#include <osmium/io/any_input.hpp>

class OSMHandler : public osmium::handler::Handler {
public:
    typedef std::shared_ptr<OSMHandler> ptr;

    OSMHandler();

    explicit OSMHandler(const std::string& keyString, const std::set<std::string>& remainedLayerValuesSet);

    ///< buffer methods
    polygonBuffer_t generateCompleteLineBuffer(boost::geometry::model::linestring<boostPoint_t>& ls) const;

    bool isLineInPolygon(const polygonBuffer_t& polygon, Road::Line::id_t lineId) const;

    ///< create method
    void createLine();
    void createWay(const osmium::Way& way);
    OSMSubWay::ptr createSubWay(Road::Way::id_t parentId, OSMNode::ptr fromNode, OSMNode::ptr toNode, std::map<std::string, std::string> wayTagsMap);
    OSMNode::ptr createNode(const osmium::Node& node);
    OSMNode::ptr createNode(double lon, double lat);
    ///< get methods
    const std::map<Road::Line::id_t, OSMLine::ptr>& getAllLineId2LineObj();

    const std::map<Road::Node::id_t, OSMNode::ptr>& getAllNodeId2NodeObj();

    const std::map<std::string, std::map<Road::Line::id_t, OSMLine::ptr>>& getAggregatedLineInfoMap();

    const std::map<std::string, std::map<Road::Way::id_t, OSMWay::ptr>>& getAggregatedWayInfoMap();

    const std::map<std::string, std::map<Road::Node::id_t, OSMNode::ptr>>& getAggregatedNodeInfoMap();

    const std::map<Road::SubWay::id_t, OSMSubWay::ptr>& getAllSubWayId2SubWayObjs();

    const std::set<Road::Node::id_t>& getAllIntersectionNodeIdSet();

    int getLocalWayIndexInLine(const Road::Way::id_t wayId);
    int getLocalSubWayIndexInWay(const Road::SubWay::id_t subWayId);

    ///< update methods
    void addNodeObjToAggregatedNodeInfoMap(std::string layerName, Road::Node::id_t nodeId, OSMNode::ptr nodeObj);

    OSMLine::ptr getLineObjById(const Road::Line::id_t& lineId);
    OSMWay::ptr getWayObjById(const Road::Way::id_t& wayId);
    OSMSubWay::ptr getSubwayObjById(const Road::SubWay::id_t& subWayId);
    OSMNode::ptr getNodeObjById(const Road::Node::id_t& nodeId);

    ///< check result methods
    void printLineNodesRef(OSMLine::ptr line);

    void printLineWaysRef(OSMLine::ptr line);

    void printSimplifiedNetwork();

    void printAllIntersection();

    void way(const osmium::Way& way);

    void node(const osmium::Node& node);

    void nodeFilter(const osmium::Node& node);

    void wayFilter(const osmium::Way& way, const std::string& tagKey, const std::set<std::string>& remainedLayers);

    ///< core methods
    void linkWays2Lines();

    void generateSimplifiedLines(); ///< buffer method
                                    //    void initMaxNodeId();
                                    //    void initMaxWayId();
    void orderWayNodeRef();

    void reBufferAllLayerLines();

    void straightenLineIfTurnCurveDetected(OSMLine::ptr line);
    void detectAndStraightenCurveInHead(OSMLine::ptr line);
    void detectAndStraightenCurveInTail(OSMLine::ptr line);
    void extendLines();

    std::map<Road::SubWay::id_t, std::vector<Road::Node::id_t>> calIntersectionPoints();

    void insertIntersectionPoints(const std::map<Road::SubWay::id_t, std::vector<Road::Node::id_t>>& info);

    //    void generateIntersection2WayLevelsDic();
    void deleteRedundantRoad();

    void mergeNeighborIntersections();

    void cleanNetwork();

    //    void generateIntersectionGraph();
    void checkNetworkConnectivity();

    void buildRamp();

    void outputOsmFile();


private:
    //    const int getAUsableNodeId();
    void updateSubWayOrder(Road::Way::id_t wayId, std::vector<OSMNode::ptr>& nodeObjs);
    bool isTwoWayShareSameNode(Road::SubWay::id_t way1FrontSubwayId,
                               Road::SubWay::id_t way1BackSubwayId,
                               Road::SubWay::id_t way2FrontSubwayId,
                               Road::SubWay::id_t way2BackSubwayId);

    std::tuple<int, double> calAngleOfConnectedSubWay(Road::SubWay::id_t way1FrontSubwayId,
                                                      Road::SubWay::id_t way1BackSubwayId,
                                                      Road::SubWay::id_t way2FrontSubwayId,
                                                      Road::SubWay::id_t way2BackSubwayId);

public:
    OSMLine::ptr pLine;

private:
    std::string m_remainedLayerKey;
    std::set<std::string> m_remainedLayerValuesSet;

    ///< obj  (global id  --> obj)
    std::map<Road::Node::id_t, OSMNode::ptr> m_allNodeId2NodeObj;
    std::map<Road::Way::id_t, OSMWay::ptr> m_allWayId2WayObj;
    std::map<Road::Line::id_t, OSMLine::ptr> m_allLineId2LineObj;
    std::map<Road::SubWay::id_t, OSMSubWay::ptr> m_allSubWayId2SubWayObjs;
    std::set<Road::Node::id_t> m_allIntersections;
    ///< cluster layer info
    std::map<std::string, std::map<Road::Line::id_t, OSMLine::ptr>>
        m_simplifiedNetwork; /// simplified Line info is stored here. <"trunk": <lineInfo> >

    double m_min_lon = DBL_MAX;
    double m_min_lat = DBL_MAX;
    double m_max_lon = -DBL_MAX;
    double m_max_lat = -DBL_MAX;

    ///< threshold
    std::map<std::string, double> m_roadLevel2extendedLength;
    double m_detectCurveLength;
    double m_curveThreshold;
    double m_parallelTolerance;
    double m_bufferDistance;      ///< for generating buffer
    int m_bufferPointsPerCircle;  ///< for generating buffer
    double m_redundantRoadLength; ///< for deleteRedundantRoad()  unit: m
    double m_mergeIntersectionDis;
};

#endif // BENCHMARK_OSMHANDLER_H
