//
// Created by xiaorui on 16.02.22.
//

#ifndef BENCHMARK_OSMVARIABLENAME_H
#define BENCHMARK_OSMVARIABLENAME_H

#include "OSMCoreTypes.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/util/math.hpp>
#include <iostream>
#include <map>
#include <osmium/geom/factory.hpp>
#include <osmium/geom/haversine.hpp>
#include <osmium/geom/wkb.hpp>
#include <osmium/geom/wkt.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/visitor.hpp>
#include <vector>

typedef int wayType_t; ///< 1: oneway type, 2:twoway type

typedef double coordinate_type;
typedef boost::geometry::model::d2::point_xy<coordinate_type> boostPoint_t;
typedef boost::geometry::model::polygon<boostPoint_t> boostPolygon_t;

typedef std::map<int64_t, std::tuple<int64_t, int64_t, int64_t, int64_t, wayType_t>> segmentDic_t; ///< {wayId: (start node,end node)}
typedef std::map<int64_t, std::deque<int64_t>> mergedWayDic_t;                                     ///< {newWayId: <wayId,wayId,wayId>}
typedef std::map<int64_t, osmium::Location> nodeId2LocationDic_t;                                  ///< {nodeId: {lat,lon}}
typedef std::map<int64_t, double> wayId2LengthDic_t;                                               ///< {wayId: length[unit: m]}
typedef std::map<int64_t, std::vector<int64_t>> wayId2NodeRefDic_t;                                ///< {wayId: [nodeRef,]}
typedef boost::geometry::model::multi_polygon<boostPolygon_t> polygonBuffer_t;
typedef std::map<int64_t, std::map<std::string, std::string>> wayId2Tags_t;
typedef std::map<int64_t, std::set<std::string>> intersection2WayLevels_t;
typedef std::map<int64_t, std::vector<int64_t>> nodeId2WayIds_t;
typedef std::map<int64_t, std::vector<int64_t>> mergedRoadId2WayIds;

typedef boost::geometry::model::linestring<boostPoint_t> boostLine_t;
typedef boost::geometry::model::d2::point_xy<double> point_type;

typedef int64_t wayId_t;
typedef int64_t nodeRef_t;
typedef int64_t interSectionId_t;
typedef int64_t roadId_t;
// recording detailed info of a line(edge in a way segment). it contains info of which way segment(wayID) is this line belonged to.
// and the start(nodeRef_t) and end(nodeRef_t) nodeRef() of this line, and its line info data structure.
// interSectionId_t is a placeholder which is used to generate global ID in OSMManager.cpp
typedef std::vector<std::tuple<wayId_t, nodeRef_t, nodeRef_t, boostLine_t, interSectionId_t>> intersectionPackage_t;

typedef int64_t mergedWayId_t;
typedef int64_t oriWaysId_t;
typedef int64_t oriNodesRef_t;

typedef std::deque<oriWaysId_t> oriWaysIdList_t;
typedef std::vector<oriNodesRef_t> oriNodesRefList_t;
typedef std::vector<int64_t> removedMergedWayIdList_t;
typedef std::deque<oriWaysId_t> removedOriWaysIdList_t;
typedef std::vector<oriNodesRef_t> removedOriNodesRefList_t;
typedef std::deque<oriWaysId_t> modifiedWaysIdList_t;
typedef std::vector<nodeRef_t> modifiedNodesIdList_t;
typedef double mergedWayLength_t;
typedef std::vector<std::string> roadLevel_t;

enum RoadInfo {
    mergedWayId = 0,
    oriWaysIdList,
    oriNodesRefList,
    removedMergedWayIdList,
    removedOriWaysIdList,
    removedOriNodesRefList,
    mergedWayLength,
    roadLevel,
    modifiedWaysIdList,
    modifiedNodesIdList
};

enum RoadInfoExceptMergedWayId {
    oriWaysIdListN = 0,
    oriNodesRefListN,
    removedMergedWayIdListN,
    removedOriWaysIdListN,
    removedOriNodesRefListN,
    mergedWayLengthN,
    roadLevelN,
    modifiedWaysIdListN
};

enum Restrictions { no_right_turn = 0, no_left_turn, no_u_turn, no_straight_on, only_right_turn, only_straight_on };
enum RampNodes { left = 0, right };
enum RampNodeWayLocation { nearIntersection = 0, farIntersection };
// simplified wayID(merged ways' ID),
// corresponding original wayID list,
// corresponding original NodeRef list.
// simplified wayID list that contains eliminated MergedWayIds by the corresponding road.
// original wayID list(corresponding eliminated MergedWayIds)
// original  NodeRef list(corresponding eliminated MergedWayIds)
// MergedWay length
typedef wayId_t fromWayId;
typedef nodeRef_t viaNodeId;
typedef wayId_t toWayId;
typedef std::string restrictionsType;
typedef std::tuple<fromWayId, viaNodeId, toWayId> restrictionsMember;
typedef std::tuple<mergedWayId_t,
                   oriWaysIdList_t,
                   oriNodesRefList_t,
                   removedMergedWayIdList_t,
                   removedOriWaysIdList_t,
                   removedOriNodesRefList_t,
                   mergedWayLength_t,
                   roadLevel_t,
                   modifiedWaysIdList_t,
                   modifiedNodesIdList_t>
    simplifiedRoadInfo_t;
typedef std::vector<simplifiedRoadInfo_t> simplifiedRoadInfoList_t;

typedef std::map<oriWaysId_t, std::map<std::string, oriNodesRef_t>> extraConnection_t;

typedef std::map<oriWaysId_t, std::vector<long>> modifiedWays_t;
typedef std::vector<std::vector<oriNodesRef_t>> intersectionGraph_t;

typedef std::map<mergedWayId_t,
                 std::tuple<oriWaysIdList_t,
                            oriNodesRefList_t,
                            removedMergedWayIdList_t,
                            removedOriWaysIdList_t,
                            removedOriNodesRefList_t,
                            mergedWayLength_t,
                            roadLevel_t,
                            modifiedWaysIdList_t>>
    roadInfoDicType_t;

#endif // BENCHMARK_OSMVARIABLENAME_H
