//
// Created by xiaorui on 09.11.21.
//

#include "../include/OSMHandler.h"

OSMHandler::OSMHandler()
{
}

OSMHandler::OSMHandler(const std::string &keyString, const std::set<std::string> &remainedLayerValuesSet)
    : m_remainedLayerKey(keyString), m_remainedLayerValuesSet(remainedLayerValuesSet)
{
    //    this->m_newNodeId = 0;

    ///< Place all threshold value here
    this->m_parallelTolerance = 30.0;
    this->m_bufferDistance = 0.0008;
    this->m_bufferPointsPerCircle = 36;
    this->m_detectCurveLength = 80.;
    this->m_roadLevel2extendedLength["trunk"] = 30.;
    this->m_roadLevel2extendedLength["tertiary"] = 30.;
    this->m_roadLevel2extendedLength["secondary"] = 30.;
    this->m_roadLevel2extendedLength["primary"] = 30.;
    this->m_roadLevel2extendedLength["residential"] = 30.;
    this->m_roadLevel2extendedLength["motorway"] = 50.;
    this->m_curveThreshold = 1.01;
    this->m_redundantRoadLength = 100;
    this->m_mergeIntersectionDis = 20;
}

polygonBuffer_t OSMHandler::generateCompleteLineBuffer(boost::geometry::model::linestring<boostPoint_t> &ls) const
{
    boost::geometry::strategy::buffer::distance_symmetric<coordinate_type> distance_strategy(this->m_bufferDistance);
    boost::geometry::strategy::buffer::join_round join_strategy(this->m_bufferPointsPerCircle);
    boost::geometry::strategy::buffer::end_round end_strategy(this->m_bufferPointsPerCircle);
    boost::geometry::strategy::buffer::point_circle circle_strategy(this->m_bufferPointsPerCircle);
    boost::geometry::strategy::buffer::side_straight side_strategy;

    // Declare output
    boost::geometry::model::multi_polygon<boostPolygon_t> result;

    boost::geometry::buffer(ls, result, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);

    return result;
}

bool OSMHandler::isLineInPolygon(const polygonBuffer_t &polygon, Road::Line::id_t lineId) const
{
    typedef boost::geometry::model::d2::point_xy<double> point_type;
    typedef boost::geometry::model::polygon<point_type> polygon_type;
    boost::geometry::model::multi_point<boostPoint_t> mp;

    ///< collect Points Of a line
    for (const auto &wayObj: this->m_allLineId2LineObj.at(lineId)->getWayObjs())
    {
        //                wayObj.second->getWayObjs().
        for (const auto &nodeObj: wayObj->getNodeRefObjs())
        {
            boost::geometry::append(mp, boostPoint_t(nodeObj->getNodeLat(), nodeObj->getNodeLon()));
        }
    }

    bool isAllNodesInPolygon = false;

    double insideCounter = 0;
    double outsideCounter = 0;
    for (auto &it: mp)
    {
        if (boost::geometry::within(it, polygon))
        {
            insideCounter += 1;
        } else
        {
            outsideCounter += 1;
        }
    }
    if (insideCounter / (insideCounter + outsideCounter) > 0.7)
    {
        isAllNodesInPolygon = true;
    }

    return isAllNodesInPolygon;
}

const std::map<Road::Line::id_t, OSMLine::ptr> &OSMHandler::getAllLineId2LineObj()
{
    return this->m_allLineId2LineObj;
}

const std::map<Road::Node::id_t, OSMNode::ptr> &OSMHandler::getAllNodeId2NodeObj()
{
    return this->m_allNodeId2NodeObj;
}

const std::map<std::string, std::map<Road::Line::id_t, OSMLine::ptr>> &OSMHandler::getAggregatedLineInfoMap()
{
    return this->m_simplifiedNetwork;
}

const std::map<Road::SubWay::id_t, OSMSubWay::ptr> &OSMHandler::getAllSubWayId2SubWayObjs()
{
    return this->m_allSubWayId2SubWayObjs;
}


void OSMHandler::printLineNodesRef(OSMLine::ptr line)
{
    for (const auto &it: line->getWayObjs())
    {
        for (const auto &it2: it->getNodeRefObjs())
        {
            std::cout << "nodeRef: " << it2->getNodeId() << '\n';
        }
    }
}

void OSMHandler::printLineWaysRef(OSMLine::ptr line)
{
    for (const auto &it: line->getWayObjs())
    {
        std::cout << "wayRef: " << it->getWayId() << '\n';
    }
}

void OSMHandler::node(const osmium::Node &node)
{
    this->nodeFilter(node);
}

void OSMHandler::way(const osmium::Way &way)
{
    this->wayFilter(way, this->m_remainedLayerKey, this->m_remainedLayerValuesSet);
}


void OSMHandler::nodeFilter(const osmium::Node &node)
{

    this->createNode(node);
    // update min and max boundary
    this->m_min_lon = std::min(m_min_lon, node.location().lon());
    this->m_max_lon = std::max(m_max_lon, node.location().lon());
    this->m_min_lat = std::min(m_min_lat, node.location().lat());
    this->m_max_lat = std::max(m_max_lat, node.location().lat());
}


void OSMHandler::wayFilter(const osmium::Way &way, const std::string &tagKey, const std::set<std::string> &remainedLayers)
{
    if (way.tags().has_key(tagKey.c_str()))
    {
        if (remainedLayers.count(std::string(way.tags().get_value_by_key(tagKey.c_str()))))
        {
            assert(way.nodes().size() >= 2);
            std::tuple<int64_t, int64_t, int64_t, int64_t, wayType_t> refs;
            if (way.tags().has_key("oneway"))
            {
                if (std::string(way.tags().get_value_by_key("oneway")) == "yes")
                {
                    refs = std::make_tuple(way.nodes().front().ref(),
                                           way.nodes()[1].ref(),
                                           way.nodes()[way.nodes().size() - 2].ref(),
                                           way.nodes().back().ref(),
                                           1);
                } else
                {
                    refs = std::make_tuple(way.nodes().front().ref(),
                                           way.nodes()[1].ref(),
                                           way.nodes()[way.nodes().size() - 2].ref(),
                                           way.nodes().back().ref(),
                                           2);
                }
            } else
            {
                refs = std::make_tuple(way.nodes().front().ref(),
                                       way.nodes()[1].ref(),
                                       way.nodes()[way.nodes().size() - 2].ref(),
                                       way.nodes().back().ref(),
                                       2);
            }
            if (way.tags().has_key("highway"))
            {
                std::map<std::string, std::string> tagRoadLevel;
                tagRoadLevel["highway"] = way.tags().get_value_by_key("highway");
            }
            if (way.tags().has_key("lanes"))
            {
                //                this->m_wayId2Tags[way.id()]["lanes"] = way.tags().get_value_by_key("lanes");
                //                this->m_allLayerWayId2Tags[way.id()]["lanes"] = way.tags().get_value_by_key("lanes");
            }

            // collect way Info
            std::vector<int64_t> nodeRefs;
            for (auto &refs: way.nodes())
            {
                nodeRefs.push_back(refs.ref());
            }

            // add tags to way and subway obj
            std::map<std::string, std::string> wayTagsMap;
            for (const auto &it: way.tags())
            {
                wayTagsMap[it.key()] = it.value();
            }
            // create subway obj
            std::vector<OSMSubWay::ptr> subwayObj;
            int nodePos = 0;
            long fromNodeId = 0;
            long toNodeId = 0;
            for (auto &refs: way.nodes())
            {
                if (nodePos == 0)
                {
                    fromNodeId = Road::Node::id_t(refs.ref());
                    nodePos += 1;
                    continue;
                } else
                {
                    toNodeId = refs.ref();
                }
                /// generate subwayObj
                OSMSubWay::ptr newSubwayObj =
                        this->createSubWay(way.id(), this->m_allNodeId2NodeObj.at(fromNodeId), this->m_allNodeId2NodeObj.at(toNodeId), wayTagsMap);

                subwayObj.push_back(newSubwayObj);
                fromNodeId = toNodeId;
                nodePos += 1;
            }

            OSMWay::ptr newWay = std::make_shared<OSMWay>(way.id(), subwayObj);

            newWay->setWayTags(wayTagsMap);
            newWay->setWayLength(osmium::geom::haversine::distance(way.nodes()));

            this->m_allSubWayId2SubWayObjs.insert(newWay->getSubWayId2SubWayObjs().begin(), newWay->getSubWayId2SubWayObjs().end());

            this->m_allWayId2WayObj[way.id()] = newWay;///< for handler
        }
    }
}


void OSMHandler::linkWays2Lines()
{
    int line_id = 0;

    ///< cluster ways by way level
    std::map<std::string, std::map<Road::Way::id_t, OSMWay::ptr>> aggregatedWayInfoMap;
    for (const auto &it: this->m_allWayId2WayObj)
    {
        std::string wayLevel = it.second->getWayTags().at("highway");
        aggregatedWayInfoMap[wayLevel].insert(std::pair<Road::Way::id_t, OSMWay::ptr>(it.first, it.second));
    }

    ///< mergeWays layer by layer
    for (const auto &layer: aggregatedWayInfoMap)
    {
        std::map<Road::Way::id_t, OSMWay::ptr> dic = layer.second;

        int numOfWays = dic.size();
        int numOfTraversedWays = 0;
        int pct = 0;

        while (not dic.empty())
        {
            std::deque<int64_t> completeWayIds;
            //            std::vector<int64_t> visitedWayIds;

            /// step1: pick up a random way as the seed to generate a line, and push it into complete and visitedWayIds;
            Road::Way::id_t way1Id;
            for (const auto &it: dic)
            {
                way1Id = it.first;
                break;
            }

            completeWayIds.push_back(way1Id);
            //            visitedWayIds.push_back(way1Id);
            Road::SubWay::id_t way1SubwayFrontId = this->m_allWayId2WayObj.at(way1Id)->getSubWayObjs().front()->getSubwayGlobalId();
            Road::SubWay::id_t way1SubwayBackId = this->m_allWayId2WayObj.at(way1Id)->getSubWayObjs().back()->getSubwayGlobalId();
            Road::SubWay::id_t way2SubwayFrontId;
            Road::SubWay::id_t way2SubwayBackId;

            bool findNextWay = true;
            /// stop while loop till find a completed line;
            while (findNextWay)
            {
                findNextWay = false;
                /// step2:
                for (const auto &way2Obj: dic)
                {
                    if (std::find(completeWayIds.begin(), completeWayIds.end(), way2Obj.second->getWayId()) != completeWayIds.end())
                    {
                        continue;
                    }
                    way2SubwayFrontId = way2Obj.second->getSubWayObjs().front()->getSubwayGlobalId();
                    way2SubwayBackId = way2Obj.second->getSubWayObjs().back()->getSubwayGlobalId();
                    ///< step3: way1 way2 share same point?
                    bool isShare = this->isTwoWayShareSameNode(way1SubwayFrontId, way1SubwayBackId, way2SubwayFrontId, way2SubwayBackId);
                    ///< if yes, calculate angle of two subways who share the same point;
                    if (isShare)
                    {
                        std::tuple<int, double> angleMap =
                                this->calAngleOfConnectedSubWay(way1SubwayFrontId, way1SubwayBackId, way2SubwayFrontId, way2SubwayBackId);

                        if (std::get<1>(angleMap) < m_parallelTolerance)
                        {
                            ///< if way2 is close to way1's front subway then add way2 to way1's left, vice versa
                            if (std::get<0>(angleMap) == 1)
                            {
                                completeWayIds.push_front(way2Obj.second->getWayId());
                                std::map<std::string, std::string> wayTagsMap;
                                OSMSubWay::ptr newSubway =
                                        this->createSubWay(-1,
                                                           this->m_allSubWayId2SubWayObjs.at(way2SubwayBackId)->getToNodeRefObj(),
                                                           this->m_allSubWayId2SubWayObjs.at(way2SubwayBackId)->getFromNodeRefObj(),
                                                           wayTagsMap);
                                way1SubwayFrontId = newSubway->getSubwayGlobalId();

                            } else if (std::get<0>(angleMap) == 2)
                            {
                                completeWayIds.push_front(way2Obj.second->getWayId());
                                way1SubwayFrontId = way2SubwayFrontId;
                                findNextWay = true;
                                break;

                            } else if (std::get<0>(angleMap) == 3)
                            {
                                completeWayIds.push_back(way2Obj.second->getWayId());
                                way1SubwayBackId = way2SubwayBackId;
                                findNextWay = true;
                                break;

                            } else if (std::get<0>(angleMap) == 4)
                            {
                                completeWayIds.push_back(way2Obj.second->getWayId());
                                std::map<std::string, std::string> wayTagsMap;
                                OSMSubWay::ptr newSubway =
                                        this->createSubWay(-1,
                                                           this->m_allSubWayId2SubWayObjs.at(way2SubwayFrontId)->getToNodeRefObj(),
                                                           this->m_allSubWayId2SubWayObjs.at(way2SubwayFrontId)->getFromNodeRefObj(),
                                                           wayTagsMap);
                                way1SubwayBackId = newSubway->getSubwayGlobalId();
                                findNextWay = true;
                                break;
                            }
                        }
                    }
                }
            }

            /// step: delete ways in completeWayIds;
            for (auto &visitedId: completeWayIds)
            {
                dic.erase(visitedId);
            }

            ///< create new Line object
            std::vector<OSMWay::ptr> wayObjs;
            ///< placeholder
            std::map<std::string, std::string> lineTags;
            lineTags.insert(std::pair<std::string, std::string>("highway", layer.first));
            for (const auto &wayPtrId: completeWayIds)
            {
                this->m_allWayId2WayObj.at(wayPtrId)->setWayParentLineId(line_id);
                wayObjs.push_back(this->m_allWayId2WayObj.at(wayPtrId));
                numOfTraversedWays++;
            }

            OSMLine::ptr lineObj = std::make_shared<OSMLine>(line_id, wayObjs, lineTags);
            this->m_allLineId2LineObj[lineObj->getLineId()] = lineObj;
            this->m_simplifiedNetwork[layer.first].insert(
                    std::pair<Road::Line::id_t, OSMLine::ptr>(line_id, this->m_allLineId2LineObj.at(line_id)));///< for handler

            line_id += 1;

            if (int(std::trunc(numOfTraversedWays * 100. / numOfWays * 1.)) / 10 >= pct)
            {
                std::cout << "Step 1/12 [" << layer.first << "] link ways to lines is processed with......" << int(std::trunc(numOfTraversedWays * 100. / numOfWays * 1.)) / 10 << "0%" << std::endl;
                pct += 1;
            }
        }
    }
}

void OSMHandler::generateSimplifiedLines()
{
    ///< simplify the map layer by layer
    for (const auto &layerLineInfo: this->m_simplifiedNetwork)
    {
        assert(layerLineInfo.second.size() > 0);

        ///< collect lineId2lengthDic
        std::map<int64_t, double> lineId2lengthDic;
        for (const auto &lineId2lengthDicItem: layerLineInfo.second)
        {
            lineId2lengthDic.insert(std::pair<int64_t, double>(lineId2lengthDicItem.first, lineId2lengthDicItem.second->getLineLength()));
        }

        std::vector<int64_t> toShowIds;
        std::vector<int64_t> toDelIds;

        int numOfLines = lineId2lengthDic.size();
        int numOfProcessedLines = 0;
        int pct = 0;

        while (not lineId2lengthDic.empty())
        {
            ///< step1: get longest lineId
            auto val = get_max(lineId2lengthDic);
            toShowIds.push_back(val.first);
            toDelIds.push_back(val.first);

            ///< step2: generate polygon along the line
            boost::geometry::model::linestring<boostPoint_t> ls;
            for (const auto &wayObj: this->m_allLineId2LineObj.at(val.first)->getWayObjs())
            {
                //                wayObj.second->getWayObjs().
                for (const auto &nodeObj: wayObj->getNodeRefObjs())
                {
                    boost::geometry::append(ls, boostPoint_t(nodeObj->getNodeLat(), nodeObj->getNodeLon()));
                }
            }
            boost::geometry::model::multi_polygon<boostPolygon_t> result = this->generateCompleteLineBuffer(ls);

            ///< step3: merge lines which 70% points are inside the buffer:
            for (auto &it: lineId2lengthDic)
            {
                if ((val.first != it.first) and (this->isLineInPolygon(result, it.first)))
                {
                    ///< add swallowed line to remained line;
                    this->m_allLineId2LineObj.at(val.first)->addSwallowedLine(it.first, this->m_allLineId2LineObj.at(it.first));
                    toDelIds.push_back(it.first);
                }
            }

            for (auto &del: toDelIds)
            {
                ///< the first line is the line to be stored
                if (del != toDelIds.front())
                {
                    for (const auto &way: this->m_simplifiedNetwork.at(layerLineInfo.first).at(del)->getWayObjs())
                    {
                        ///< delete old way's parent
                        way->removeParentLineId();
                    }

                    this->m_simplifiedNetwork[layerLineInfo.first].erase(del);
                }

                lineId2lengthDic.erase(del);
                numOfProcessedLines++;
            }
            toDelIds.clear();

            if (int(std::trunc(numOfProcessedLines * 100. / numOfLines * 1.)) / 10 >= pct)
            {
                std::cout << "Step 2/12 [" << layerLineInfo.first << "] use buffer to simplify lines is processed with......" << int(std::trunc(numOfProcessedLines * 100. / numOfLines * 1.)) / 10 << "0%" << std::endl;
                pct += 1;
            }
        }
    }
}

void OSMHandler::orderWayNodeRef()
{

    int numOfLines = 0;
    int numOfProcessedLines = 0;
    int pct = 0;

    for (auto &layerLineInfo: this->m_simplifiedNetwork)
    {
        numOfLines += layerLineInfo.second.size();
    }

    int count = 0;
    for (auto &layerLineInfo: this->m_simplifiedNetwork)
    {
        for (auto &lineObj: layerLineInfo.second)
        {
            numOfProcessedLines++;
            for (int i = 0, j = 1; j < lineObj.second->getWayObjs().size(); i++, j++)
            {
                Road::Way::id_t preWayId = i;
                Road::Way::id_t nextWayId = j;

                Road::Node::id_t preWayIdStartNodeRef = lineObj.second->getWayObjs().at(preWayId)->getNodeRefObjs().front()->getNodeId();
                Road::Node::id_t preWayIdEndNodeRef = lineObj.second->getWayObjs().at(preWayId)->getNodeRefObjs().back()->getNodeId();
                Road::Node::id_t nextWayIdStartNodeRef = lineObj.second->getWayObjs().at(nextWayId)->getNodeRefObjs().front()->getNodeId();
                Road::Node::id_t nextWayIdEndNodeRef = lineObj.second->getWayObjs().at(nextWayId)->getNodeRefObjs().back()->getNodeId();

                std::vector<OSMNode::ptr> nodePreObjs = lineObj.second->getWayObjs().at(preWayId)->getNodeRefObjs();
                std::vector<OSMNode::ptr> nodeNextObjs = lineObj.second->getWayObjs().at(nextWayId)->getNodeRefObjs();
                bool printFlag = false;

                if (preWayIdStartNodeRef == nextWayIdStartNodeRef)
                {

                    printFlag = true;
                    count += 1;
                    std::reverse(nodePreObjs.begin(), nodePreObjs.end());

                } else if (preWayIdStartNodeRef == nextWayIdEndNodeRef)
                {
                    printFlag = true;
                    count += 1;
                    std::reverse(nodePreObjs.begin(), nodePreObjs.end());
                    std::reverse(nodeNextObjs.begin(), nodeNextObjs.end());
                } else if (preWayIdEndNodeRef == nextWayIdEndNodeRef)
                {
                    printFlag = true;
                    count += 1;
                    std::reverse(nodeNextObjs.begin(), nodeNextObjs.end());
                }
                this->updateSubWayOrder(lineObj.second->getWayObjs().at(preWayId)->getWayId(), nodePreObjs);
                this->updateSubWayOrder(lineObj.second->getWayObjs().at(nextWayId)->getWayId(), nodeNextObjs);

                if (int(std::trunc(numOfProcessedLines * 100. / numOfLines * 1.)) / 10 >= pct)
                {
                    std::cout << "Step 4/12 sort nodes oder within one line is processed with......" << int(std::trunc(numOfProcessedLines * 100. / numOfLines * 1.)) / 10 << "0%" << std::endl;
                    pct += 1;
                }
            }
        }
    }
}

void OSMHandler::reBufferAllLayerLines()
{
    ///< collect lineId2lengthDic
    std::map<int64_t, double> lineId2lengthDic;
    for (const auto &lineInfo: this->m_simplifiedNetwork)
    {
        for (const auto &lineId2lengthDicItem: lineInfo.second)
        {
            lineId2lengthDic.insert(std::pair<int64_t, double>(lineId2lengthDicItem.first, lineId2lengthDicItem.second->getLineLength()));
        }
    }

    std::vector<int64_t> toShowIds;
    std::vector<int64_t> toDelIds;

    int numOfLines = lineId2lengthDic.size();
    int numOfProcessedLines = 0;
    int pct = 0;

    while (not lineId2lengthDic.empty())
    {
        ///< step1: get longest lineId
        auto val = get_max(lineId2lengthDic);
        toShowIds.push_back(val.first);
        toDelIds.push_back(val.first);

        ///< step2: generate polygon along the line
        boost::geometry::model::linestring<boostPoint_t> ls;
        for (const auto &wayObj: this->m_allLineId2LineObj.at(val.first)->getWayObjs())
        {
            //                wayObj.second->getWayObjs().
            for (const auto &nodeObj: wayObj->getNodeRefObjs())
            {
                boost::geometry::append(ls, boostPoint_t(nodeObj->getNodeLat(), nodeObj->getNodeLon()));
            }
        }
        boost::geometry::model::multi_polygon<boostPolygon_t> result = this->generateCompleteLineBuffer(ls);

        ///< step3: merge lines which 70% points are inside the buffer:
        for (auto &it: lineId2lengthDic)
        {
            if ((val.first != it.first) and (this->isLineInPolygon(result, it.first)))
            {
                ///< add swallowed line to remained line;
                this->m_allLineId2LineObj.at(val.first)->addSwallowedLine(it.first, this->m_allLineId2LineObj.at(it.first));
                toDelIds.push_back(it.first);
            }
        }

        for (auto &del: toDelIds)
        {
            ///< the first line is the line to be stored
            if (del != toDelIds.front())
            {
                std::string layerName = this->m_allLineId2LineObj.at(del)->getLineTags().at("highway");

                for (const auto &way: this->m_simplifiedNetwork.at(layerName).at(del)->getWayObjs())
                {
                    ///< delete old way's parent
                    way->removeParentLineId();
                }

                this->m_simplifiedNetwork[layerName].erase(del);
            }

            lineId2lengthDic.erase(del);
            numOfProcessedLines++;
        }
        toDelIds.clear();

        if (int(std::trunc(numOfProcessedLines * 100. / numOfLines * 1.)) / 10 >= pct)
        {
            std::cout << "Step 3/12 use rebuffer to merge side road is processed with......" << int(std::trunc(numOfProcessedLines * 100. / numOfLines * 1.)) / 10 << "0%" << std::endl;
            pct += 1;
        }
    }
}

void OSMHandler::extendLines()
{

    int numOfLines = 0;
    int numOfProcessedLines = 0;
    int pct = 0;

    for (auto &layerLineInfo: this->m_simplifiedNetwork)
    {
        numOfLines += layerLineInfo.second.size();
    }

    for (const auto &it: this->m_simplifiedNetwork)
    {
        for (const auto &line: it.second)
        {
            numOfProcessedLines++;
            this->straightenLineIfTurnCurveDetected(line.second);
            double extendedLength = 0;
            if (this->m_roadLevel2extendedLength.count(it.first))
            {
                extendedLength = this->m_roadLevel2extendedLength.at(it.first);
            } else
            {
                return;
            }
            // extend in Head
            {
                auto const &wayInHead = line.second->getWayObjs().front();
                auto const &subWayInHead = wayInHead->getSubWayObjs().front();

                const auto &newSlonlat = extendWithDistanceToNewPosition(subWayInHead->getToNodeRefObj()->getSlonlatPosition(),
                                                                         subWayInHead->getFromNodeRefObj()->getSlonlatPosition(),
                                                                         extendedLength);

                const auto nodeId = OSMGenerateGlobalId::getInstance()->getNewNodeId();
                OSMNode::ptr newNode = std::make_shared<OSMNode>(nodeId, newSlonlat);
                this->m_allNodeId2NodeObj[nodeId] = newNode;
                subWayInHead->updateFromNode(newNode);
            }
            // extend in Tail
            {
                auto const &wayInTail = line.second->getWayObjs().back();
                auto const &subWayInTail = wayInTail->getSubWayObjs().back();

                const auto &newSlonlat = extendWithDistanceToNewPosition(subWayInTail->getFromNodeRefObj()->getSlonlatPosition(),
                                                                         subWayInTail->getToNodeRefObj()->getSlonlatPosition(),
                                                                         extendedLength);
                const auto nodeId = OSMGenerateGlobalId::getInstance()->getNewNodeId();
                OSMNode::ptr newNode = std::make_shared<OSMNode>(nodeId, newSlonlat);
                this->m_allNodeId2NodeObj[nodeId] = newNode;
                subWayInTail->updateToNode(newNode);
            }

            if (int(std::trunc(numOfProcessedLines * 100. / numOfLines * 1.)) / 10 >= pct)
            {
                std::cout << "Step 5/12 extend lines is processed with......" << int(std::trunc(numOfProcessedLines * 100. / numOfLines * 1.)) / 10 << "0%" << std::endl;
                pct += 1;
            }
        }
    }
}

std::map<Road::SubWay::id_t, std::vector<Road::Node::id_t>> OSMHandler::calIntersectionPoints()
{
    std::map<Road::SubWay::id_t, std::vector<Road::Node::id_t>> intersectionInfoMap;
    std::map<Road::Line::id_t, OSMLine::ptr> tmpData;

    for (const auto &it: this->m_simplifiedNetwork)
    {
        tmpData.insert(it.second.begin(), it.second.end());
    }

    Road::Line::id_t visitedLine;

    int numOfLines = tmpData.size();
    int numOfProcessedLines = 0;
    int pct = 0;

    while (not tmpData.empty())
    {
        for (const auto &line1: tmpData)
        {
            visitedLine = line1.first;

            for (const auto &wayObj: line1.second->getWayObjs())
            {
                for (const auto &subwayObj: wayObj->getSubWayObjs())
                {
                    ///< generate all subways for line1
                    double point1Y = subwayObj->getFromNodeRefObj()->getNodeLat();
                    double point1X = subwayObj->getFromNodeRefObj()->getNodeLon();
                    double point2Y = subwayObj->getToNodeRefObj()->getNodeLat();
                    double point2X = subwayObj->getToNodeRefObj()->getNodeLon();
                    boostPoint_t boostPoint1(point1X, point1Y);
                    boostPoint_t boostPoint2(point2X, point2Y);
                    boostLine_t subwaysLine;
                    boost::geometry::append(subwaysLine, boostPoint1);
                    boost::geometry::append(subwaysLine, boostPoint2);

                    /// get subwaysLine from sub
                    for (const auto &line2: tmpData)
                    {
                        if (line1.first != line2.first)
                        {
                            ///< generate all subways for line2
                            for (const auto &wayObj2: line2.second->getWayObjs())
                            {
                                for (const auto &subwayObj2: wayObj2->getSubWayObjs())
                                {
                                    ///< generate all subways for line1
                                    double point1Y = subwayObj2->getFromNodeRefObj()->getNodeLat();
                                    double point1X = subwayObj2->getFromNodeRefObj()->getNodeLon();
                                    double point2Y = subwayObj2->getToNodeRefObj()->getNodeLat();
                                    double point2X = subwayObj2->getToNodeRefObj()->getNodeLon();
                                    boostPoint_t boostPoint1(point1X, point1Y);
                                    boostPoint_t boostPoint2(point2X, point2Y);
                                    boostLine_t subwaysLine2;
                                    boost::geometry::append(subwaysLine2, boostPoint1);
                                    boost::geometry::append(subwaysLine2, boostPoint2);

                                    ///< calIntersection points
                                    bool isIntersectionExist = boost::geometry::intersects(subwaysLine, subwaysLine2);
                                    if (!isIntersectionExist)
                                    {
                                        continue;
                                    }

                                    std::vector<point_type> output;
                                    boost::geometry::intersection(subwaysLine, subwaysLine2, output);
                                    if (!output.empty())
                                    {
                                        assert(output.size() == 1);// two straight lines can only have 1 intersection.

                                        ///< output can obtain multiple elements
                                        for (const auto &intersectionPoint: output)
                                        {
                                            ///< generate new Node Obj
                                            OSMNode::ptr newNode = this->createNode(intersectionPoint.x(), intersectionPoint.y());
                                            std::set<std::string> intersectionLevel;
                                            intersectionLevel.insert(line1.second->getLineTags().at("highway"));
                                            intersectionLevel.insert(line2.second->getLineTags().at("highway"));
                                            newNode->setIntersectionLevel(
                                                    intersectionLevel);///< only intersection nodes have valid value for this variable.
                                            intersectionInfoMap[subwayObj->getSubwayGlobalId()].push_back(newNode->getNodeId());
                                            intersectionInfoMap[subwayObj2->getSubwayGlobalId()].push_back(newNode->getNodeId());
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            break;
        }
        ///< remove line1 from tmpData
        tmpData.erase(visitedLine);
        numOfProcessedLines++;

        if (int(std::trunc(numOfProcessedLines * 100. / numOfLines * 1.)) / 10 >= pct)
        {
            std::cout << "Step 6/12 calculate intersections' coordinates is processed with......" << int(std::trunc(numOfProcessedLines * 100. / numOfLines * 1.)) / 10 << "0%" << std::endl;
            pct += 1;
        }
    }
    return intersectionInfoMap;
}

void OSMHandler::insertIntersectionPoints(const std::map<Road::SubWay::id_t, std::vector<Road::Node::id_t>> &info)
{
    int numOfLines = info.size();
    int numOfProcessedLines = 0;
    int pct = 0;


    for (const auto &it: info)
    {
        numOfProcessedLines ++;
        std::vector<std::pair<Road::Node::id_t, double>> id2distance;
        const auto &subway = this->m_allSubWayId2SubWayObjs.at(it.first);

        for (const auto &nodeId: it.second)
        {
            const auto &node = this->m_allNodeId2NodeObj.at(nodeId);
            double dis = distance(subway->getFromNodeRefObj()->getNodeLat(),
                                  subway->getFromNodeRefObj()->getNodeLon(),
                                  node->getNodeLat(),
                                  node->getNodeLon());
            if (dis <= 0.1)
            {
                subway->getFromNodeRefObj()->insertIntersectionLevel(node->getIntersectionLevel());
                this->m_allIntersections.insert(subway->getFromNodeRefObj()->getNodeId());
            } else if (dis >= subway->getLength() - 0.1)
            {
                subway->getToNodeRefObj()->insertIntersectionLevel(node->getIntersectionLevel());
                this->m_allIntersections.insert(subway->getToNodeRefObj()->getNodeId());
            } else
            {
                id2distance.push_back(std::make_pair(nodeId, dis));
                this->m_allIntersections.insert(nodeId);
            }
        }
        if (id2distance.size() == 0) continue;
        std::sort(id2distance.begin(), id2distance.end(), [=](std::pair<Road::Node::id_t, double> &a, std::pair<Road::Node::id_t, double> &b) {
            return a.second < b.second;
        });

        std::vector<OSMSubWay::ptr> newSubways;

        for (int i = 0; i <= id2distance.size(); i++)
        {
            OSMSubWay::ptr newSubwayObj = nullptr;
            if (i == 0)
            {
                newSubwayObj = std::make_shared<OSMSubWay>(subway->getWayParentId(),
                                                           subway->getFromNodeRefObj(),
                                                           this->m_allNodeId2NodeObj.at(id2distance.at(i).first),
                                                           subway->getSubWayTagsMap());
            } else if (i == id2distance.size())
            {
                newSubwayObj = std::make_shared<OSMSubWay>(subway->getWayParentId(),
                                                           this->m_allNodeId2NodeObj.at(id2distance.at(i - 1).first),
                                                           subway->getToNodeRefObj(),
                                                           subway->getSubWayTagsMap());
            } else
            {
                newSubwayObj = std::make_shared<OSMSubWay>(subway->getWayParentId(),
                                                           this->m_allNodeId2NodeObj.at(id2distance.at(i - 1).first),
                                                           this->m_allNodeId2NodeObj.at(id2distance.at(i).first),
                                                           subway->getSubWayTagsMap());
            }
            this->m_allSubWayId2SubWayObjs[newSubwayObj->getSubwayGlobalId()] = newSubwayObj;
            newSubways.push_back(newSubwayObj);
        }
        auto &way = this->m_allWayId2WayObj.at(subway->getWayParentId());
        std::vector<OSMSubWay::ptr> subways;
        for (const auto &sub: way->getSubWayObjs())
        {
            if (sub->getSubwayGlobalId() == subway->getSubwayGlobalId())
            {
                subways.insert(subways.end(), newSubways.begin(), newSubways.end());
            } else
            {
                subways.push_back(sub);
            }
        }
        way->updateSubWayObj(subways);

        if (int(std::trunc(numOfProcessedLines * 100. / numOfLines * 1.)) / 10 >= pct)
        {
            std::cout << "Step 7/12 insert intersections into ways is processed with......" << int(std::trunc(numOfProcessedLines * 100. / numOfLines * 1.)) / 10 << "0%" << std::endl;
            pct += 1;
        }
    }
}

void OSMHandler::deleteRedundantRoad()
{
    int numOfLines = 0;
    int numOfProcessedLines = 0;
    int pct = 0;

    for (auto &layerLineInfo: this->m_simplifiedNetwork)
    {
        numOfLines += layerLineInfo.second.size();
    }


    for (const auto &lineDic: this->m_simplifiedNetwork)
    {
        for (const auto &lineObj: lineDic.second)
        {
            numOfProcessedLines ++;

            if (int(std::trunc(numOfProcessedLines * 100. / numOfLines * 1.)) / 10 >= pct)
            {
                std::cout << "Step 8/12 delete redundant part of lines is processed with......" << int(std::trunc(numOfProcessedLines * 100. / numOfLines * 1.)) / 10 << "0%" << std::endl;
                pct += 1;
            }

            OSMWay::ptr startWayObj = lineObj.second->getWayObjs().at(0);
            OSMWay::ptr endWayObj = lineObj.second->getWayObjs().at(lineObj.second->getWayObjs().size() - 1);

            ///< process startWayObj
            double redundantLengthStart = 0.0;
            int deleteFromStartPos = 0;
            bool deleteStartFlag = false;
            for (const auto &subwayObj: startWayObj->getSubWayObjs())
            {
                redundantLengthStart += subwayObj->getLength();
                ///< keep redundant part
                if (redundantLengthStart >= this->m_redundantRoadLength)
                {
                    deleteFromStartPos = 0;
                    break;
                } else
                {
                    if (this->m_allIntersections.count(subwayObj->getFromNodeRefObj()->getNodeId()))
                    {
                        deleteStartFlag = true;
                        break;

                    } else if (this->m_allIntersections.count(subwayObj->getToNodeRefObj()->getNodeId()))
                    {
                        deleteFromStartPos += 1;
                        deleteStartFlag = true;
                        break;
                    }
                }
                deleteFromStartPos += 1;
            }
            if (deleteStartFlag)
            {
                ///< delete redundant from start end;
                startWayObj->deleteSubWayObjFromHead(deleteFromStartPos);
            }

            ///< process endWayObj
            if (!endWayObj->getSubWayObjs().empty())
            {
                double redundantLengthEnd = 0.0;
                int deleteFromEndPos = 0;
                bool deleteEndFlag = false;
                for (int i = endWayObj->getSubWayObjs().size() - 1; i >= 0; i--)
                {
                    const auto &subwayObj = endWayObj->getSubWayObjs().at(i);
                    redundantLengthEnd += subwayObj->getLength();
                    ///< keep redundant part
                    if (redundantLengthEnd >= this->m_redundantRoadLength)
                    {
                        deleteFromEndPos = 0;
                        break;
                    } else
                    {
                        if (this->m_allIntersections.count(subwayObj->getToNodeRefObj()->getNodeId()))
                        {
                            deleteEndFlag = true;
                            break;

                        } else if (this->m_allIntersections.count(subwayObj->getFromNodeRefObj()->getNodeId()))
                        {
                            deleteFromEndPos += 1;
                            deleteEndFlag = true;
                            break;
                        }
                    }
                    deleteFromEndPos += 1;
                }
                if (deleteEndFlag)
                {
                    ///< delete redundant from start end;
                    endWayObj->deleteSubWayObjFromTail(deleteFromEndPos);
                }
            }
        }
    }
}

void OSMHandler::mergeNeighborIntersections()
{
    std::set<Road::Node::id_t> intersections;
    std::set<Road::Node::id_t> visited;
    std::vector<std::vector<Road::Node::id_t>> allClusters;

    ///< step1: get all intersection points;
    std::vector<Road::Node::id_t> intersectionVector(this->m_allIntersections.begin(), this->m_allIntersections.end());

    ///< step1: cluster intersection points
    for (int i = 0; i < intersectionVector.size(); i++)
    {
        std::vector<nodeRef_t> cluster;
        const bool is_in = visited.find(intersectionVector.at(i)) != visited.end();
        cluster.push_back(intersectionVector.at(i));
        visited.insert(intersectionVector.at(i));

        if (not is_in)
        {
            for (int j = i + 1; j < intersectionVector.size(); j++)
            {
                Road::Node::id_t node1 = intersectionVector.at(i);
                Road::Node::id_t node2 = intersectionVector.at(j);
                double dis = distance(this->getNodeObjById(node1)->getNodeLat(),
                                      this->getNodeObjById(node1)->getNodeLon(),
                                      this->getNodeObjById(node2)->getNodeLat(),
                                      this->getNodeObjById(node2)->getNodeLon());
                if (dis <= this->m_mergeIntersectionDis)
                {
                    cluster.push_back(intersectionVector.at(j));
                    visited.insert(intersectionVector.at(j));
                }
            }
            allClusters.push_back(cluster);
        }
    }
    ///< step2: create a new node for a cluster, and map all nodes inside the cluster to the new node;

    int numOfClusters = allClusters.size();
    int numOfProcessedClusters = 0;
    int pct = 0;

    for (const auto &clusterItem: allClusters)
    {
        numOfProcessedClusters++;

        if (int(std::trunc(numOfProcessedClusters * 100. / numOfClusters * 1.)) / 10 >= pct)
        {
            std::cout << "Step 9/12 merge neighbor intersections is processed with......" << int(std::trunc(numOfProcessedClusters * 100. / numOfClusters * 1.)) / 10 << "0%" << std::endl;
            pct += 1;
        }

        if (clusterItem.size() == 1)
        {
            intersections.insert(clusterItem.at(0));
            continue;
        }
        ///< step3: get average pos of clustered nodes;
        double lon = 0.0;
        double lat = 0.0;
        for (const auto &nodeId: clusterItem)
        {
            lon += this->getNodeObjById(nodeId)->getNodeLon();
            lat += this->getNodeObjById(nodeId)->getNodeLat();
        }
        lon /= clusterItem.size();
        lat /= clusterItem.size();

        const auto &newNode = this->createNode(lon, lat);
        intersections.insert(newNode->getNodeId());
        /// find line which need to replace with newNode
        std::map<Road::Line::id_t, std::set<Road::Node::id_t>> line2closedIntersection;
        for (const auto &nodeId: clusterItem)
        {
            const auto &node = this->getNodeObjById(nodeId);
            newNode->setIntersectionLevel(node->getIntersectionLevel());
            for (const auto &subWayParentId: this->getNodeObjById(nodeId)->getSubWayParentId())
            {
                const auto &wayId = this->getSubwayObjById(subWayParentId)->getWayParentId();
                const auto &lineId = this->getWayObjById(wayId)->getLineParentId();
                if (!line2closedIntersection.count(lineId)) line2closedIntersection[lineId] = {};
                line2closedIntersection.at(lineId).insert(nodeId);
            }
        }
        ///< update subwayParentIds of new node;
        for (const auto &it: line2closedIntersection)
        {
            auto line = this->getLineObjById(it.first);
            /// only one intersection needs to be replaced
            if (it.second.size() == 1)
            {
                bool replaced = false;
                for (const auto &way: line->getWayObjs())
                {
                    if (replaced) break;
                    for (const auto &subWay: way->getSubWayObjs())
                    {
                        if (replaced) break;
                        if (it.second.count(subWay->getToNodeRefObj()->getNodeId()))
                        {
                            subWay->updateToNode(newNode);
                        } else if (it.second.count(subWay->getFromNodeRefObj()->getNodeId()))
                        {
                            subWay->updateFromNode(newNode);
                            replaced = true;
                        }
                    }
                }
                /// more than one intersections need to be replaced
            } else
            {
                /// identify whether all closed intersections are in one way
                std::vector<int> intersectionLocatedWayIndex;
                for (const auto &nodeId: it.second)
                {
                    for (const auto &subwayIds: this->getNodeObjById(nodeId)->getSubWayParentId())
                    {
                        const auto &wayId = this->getSubwayObjById(subwayIds)->getWayParentId();
                        if (this->getWayObjById(wayId)->getLineParentId() == line->getLineId())
                        {
                            intersectionLocatedWayIndex.push_back(this->getLocalWayIndexInLine(wayId));
                        }
                    }
                }
                std::sort(intersectionLocatedWayIndex.begin(), intersectionLocatedWayIndex.end(), std::less<int>());
                auto wayInFront = line->getWayObjs().at(intersectionLocatedWayIndex.front());
                auto wayInBack = line->getWayObjs().at(intersectionLocatedWayIndex.back());
                std::vector<OSMSubWay::ptr> subwaysInFront;
                std::vector<OSMSubWay::ptr> subwaysInBack;
                for (int i = 0; i < wayInFront->getSubWayObjs().size(); i++)
                {
                    const auto &subway = wayInFront->getSubWayObjs().at(i);
                    if (it.second.count(subway->getFromNodeRefObj()->getNodeId()))
                    {
                        /// first node in this line need to be replaced then, do nothing
                        break;
                    }
                    if (it.second.count(subway->getToNodeRefObj()->getNodeId()))
                    {
                        subway->updateToNode(newNode);
                        subwaysInFront.push_back(subway);
                        break;
                    } else
                    {
                        subwaysInFront.push_back(subway);
                    }
                }
                /// go from back to front, if meets intersections need to be replaced, stop
                for (int i = wayInBack->getSubWayObjs().size() - 1; i >= 0; i--)
                {
                    const auto &subway = wayInBack->getSubWayObjs().at(i);
                    if (it.second.count(subway->getToNodeRefObj()->getNodeId()))
                    {
                        break;
                    } else if (it.second.count(subway->getFromNodeRefObj()->getNodeId()))
                    {
                        subway->updateFromNode(newNode);
                        subwaysInBack.insert(subwaysInBack.begin(), subway);
                        break;
                    } else
                    {
                        subwaysInBack.insert(subwaysInBack.begin(), subway);
                    }
                }
                if (intersectionLocatedWayIndex.front() != intersectionLocatedWayIndex.back())
                {
                    wayInFront->updateSubWayObj(subwaysInFront);
                    wayInBack->updateSubWayObj(subwaysInBack);
                } else
                {
                    subwaysInFront.insert(subwaysInFront.end(), subwaysInBack.begin(), subwaysInBack.end());
                    wayInFront->updateSubWayObj(subwaysInFront);
                    subwaysInBack.clear();
                }
                std::vector<OSMWay::ptr> updatedWaysVec;
                for (int wayIndex = 0; wayIndex < intersectionLocatedWayIndex.front(); wayIndex++)
                {
                    updatedWaysVec.push_back(line->getWayObjs().at(wayIndex));
                }
                if (!subwaysInFront.empty())
                {
                    updatedWaysVec.push_back(wayInFront);
                }
                if (!subwaysInBack.empty())
                {
                    updatedWaysVec.push_back(wayInBack);
                }
                for (int wayIndex = intersectionLocatedWayIndex.back() + 1; wayIndex < line->getWayObjs().size(); wayIndex++)
                {
                    updatedWaysVec.push_back(line->getWayObjs().at(wayIndex));
                }
                if (updatedWaysVec.empty())
                {
                    for (const auto &way: this->m_simplifiedNetwork.at(line->getLineTags().at("highway")).at(line->getLineId())->getWayObjs())
                    {
                        ///< delete old way's parent
                        way->removeParentLineId();
                    }

                    this->m_simplifiedNetwork.at(line->getLineTags().at("highway")).erase(line->getLineId());
                } else
                {
                    line->updateWayObjs(updatedWaysVec);
                }
            }
        }
    }
    //    }
    this->m_allIntersections = intersections;
}
void OSMHandler::straightenLineIfTurnCurveDetected(OSMLine::ptr line)
{

    double detectCurveLength = this->m_detectCurveLength;
    if (line->getLineLength() < detectCurveLength or (line->getWayObjs().size() == 1 and line->getWayObjs().at(0)->getSubWayObjs().size() == 1) or
        line->getWayObjs().front()->getNodeRefObjs().front()->getNodeId() == line->getWayObjs().back()->getNodeRefObjs().back()->getNodeId())
    {
        return;
    }
    this->detectAndStraightenCurveInHead(line);
    this->detectAndStraightenCurveInTail(line);
}

void OSMHandler::detectAndStraightenCurveInHead(OSMLine::ptr line)
{
    double detectCurveLength = this->m_detectCurveLength;
    double curveThreshold = this->m_curveThreshold;
    OSMSubWay::ptr subWayInHead = line->getWayObjs().front()->getSubWayObjs().front();
    double length = 0;
    OSMSubWay::ptr subWayAfterCurve = nullptr;
    OSMWay::ptr wayAfterCurve = nullptr;
    int wayPosition = 0;
    int subWayPosition = 0;
    if (subWayInHead->getLength() >= detectCurveLength)
    {
        return;
    } else
    {
        for (int i = 0; i < line->getWayObjs().size(); i++)
        {
            if (subWayAfterCurve != nullptr) break;
            const auto &way = line->getWayObjs().at(i);
            for (int j = 0; j < way->getSubWayObjs().size(); j++)
            {
                const auto &subway = way->getSubWayObjs().at(j);
                length += subway->getLength();
                if (length > detectCurveLength)
                {
                    subWayAfterCurve = subway;
                    wayAfterCurve = way;
                    wayPosition = i;
                    subWayPosition = j;
                    break;
                }
            }
        }
    }
    assert(subWayAfterCurve != nullptr);
    double beelineDistance = distance(subWayInHead->getFromNodeRefObj()->getNodeLat(),
                                      subWayInHead->getFromNodeRefObj()->getNodeLon(),
                                      subWayAfterCurve->getFromNodeRefObj()->getNodeLat(),
                                      subWayAfterCurve->getFromNodeRefObj()->getNodeLon());
    if ((length - subWayAfterCurve->getLength()) / beelineDistance > curveThreshold)
    {
        OSMSubWay::ptr nextSubWay = nullptr;
        if (wayAfterCurve->getSubWayObjs().size() - 1 > subWayPosition)
        {
            nextSubWay = wayAfterCurve->getSubWayObjs().at(subWayPosition + 1);
        } else if (line->getWayObjs().size() - 1 > wayPosition)
        {
            nextSubWay = line->getWayObjs().at(wayPosition + 1)->getSubWayObjs().front();
        } else
        {
            return;
        }
        const auto &newSlonlat = extendWithDistanceToNewPosition(nextSubWay->getToNodeRefObj()->getSlonlatPosition(),
                                                                 nextSubWay->getFromNodeRefObj()->getSlonlatPosition(),
                                                                 length);
        const auto nodeId = OSMGenerateGlobalId::getInstance()->getNewNodeId();
        OSMNode::ptr newNode = std::make_shared<OSMNode>(nodeId, newSlonlat);
        this->m_allNodeId2NodeObj[nodeId] = newNode;

        subWayAfterCurve->updateFromNode(newNode);
        wayAfterCurve->deleteSubWayObjFromHead(subWayPosition);
        line->deleteWayFromHead(wayPosition);
    }
}
void OSMHandler::detectAndStraightenCurveInTail(OSMLine::ptr line)
{
    double detectCurveLength = this->m_detectCurveLength;
    double curveThreshold = this->m_curveThreshold;
    OSMSubWay::ptr subWayInTail = line->getWayObjs().back()->getSubWayObjs().back();
    double length = 0;

    OSMSubWay::ptr subWayBeforeCurve = nullptr;
    OSMWay::ptr wayBeforeCurve = nullptr;
    int wayPosition = 0;
    int subWayPosition = 0;
    if (subWayInTail->getLength() >= detectCurveLength)
    {
        return;
    } else
    {
        for (int i = line->getWayObjs().size() - 1; i >= 0; i--)
        {
            if (subWayBeforeCurve != nullptr) break;
            const auto &way = line->getWayObjs().at(i);
            for (int j = way->getSubWayObjs().size() - 1; j >= 0; j--)
            {
                const auto &subway = way->getSubWayObjs().at(j);
                length += subway->getLength();
                if (length > detectCurveLength)
                {
                    subWayBeforeCurve = subway;
                    wayBeforeCurve = way;
                    wayPosition = i;
                    subWayPosition = j;
                    break;
                }
            }
        }
    }
    assert(subWayBeforeCurve != nullptr);
    double beelineDistance = distance(subWayInTail->getToNodeRefObj()->getNodeLat(),
                                      subWayInTail->getToNodeRefObj()->getNodeLon(),
                                      subWayBeforeCurve->getToNodeRefObj()->getNodeLat(),
                                      subWayBeforeCurve->getToNodeRefObj()->getNodeLon());
    if ((length - subWayBeforeCurve->getLength()) / beelineDistance > curveThreshold)
    {
        OSMSubWay::ptr previSubWay = nullptr;
        if (subWayPosition > 0)
        {
            previSubWay = wayBeforeCurve->getSubWayObjs().at(subWayPosition - 1);
        } else if (wayPosition > 0)
        {
            previSubWay = line->getWayObjs().at(wayPosition - 1)->getSubWayObjs().back();
        } else
        {
            return;
        }
        const auto &newSlonlat = extendWithDistanceToNewPosition(previSubWay->getFromNodeRefObj()->getSlonlatPosition(),
                                                                 previSubWay->getToNodeRefObj()->getSlonlatPosition(),
                                                                 length);
        const auto nodeId = OSMGenerateGlobalId::getInstance()->getNewNodeId();
        OSMNode::ptr newNode = std::make_shared<OSMNode>(nodeId, newSlonlat);
        this->m_allNodeId2NodeObj[nodeId] = newNode;

        subWayBeforeCurve->updateToNode(newNode);
        wayBeforeCurve->deleteSubWayObjFromTail(wayBeforeCurve->getSubWayObjs().size() - (subWayPosition + 1));
        line->deleteWayFromTail(line->getWayObjs().size() - (wayPosition + 1));
    }
}

void OSMHandler::cleanNetwork()
{
    std::map<Road::Line::id_t, std::vector<Road::Node::id_t>> intersectionGraph;
    for (const auto &intersectionId: this->m_allIntersections)
    {
        const auto &intersection = this->getNodeObjById(intersectionId);
        assert(intersection->getSubWayParentId().size() > 0);
        for (const auto &subwayId: intersection->getSubWayParentId())
        {
            const auto &subway = this->getSubwayObjById(subwayId);
            const auto &wayId = subway->getWayParentId();
            const auto &lineId = this->getWayObjById(wayId)->getLineParentId();
            if (!intersectionGraph.count(lineId)) intersectionGraph[lineId] = {};
            intersectionGraph.at(lineId).push_back(intersectionId);
        }
    }

    std::map<Road::Node::id_t, std::set<Road::Node::id_t>> connectionGraph;

    for (const auto &intersections: intersectionGraph)
    {
        for (const auto &node: intersections.second)
        {
            if (!connectionGraph.count(node))
            {
                connectionGraph[node] = {};
            }
            for (const auto &node2: intersections.second)
            {
                if (node2 != node)
                {
                    connectionGraph.at(node).insert(node2);
                }
            }
        }
    }
    std::map<Road::Node::id_t, std::set<Road::Node::id_t>> clusters;
    Road::Node::id_t mainClusterKey = -1;
    int mainClusterSize = -1;
    std::set<Road::Node::id_t> visitedNode;

    for (const auto &nodes: connectionGraph)
    {
        // find new vertices
        if (not visitedNode.count(nodes.first))
        {
            std::set<Road::Node::id_t> cluster;

            std::vector<Road::Node::id_t> queue;
            queue.push_back(nodes.first);
            while (!queue.empty())
            {
                auto currentNode = queue.front();
                cluster.insert(currentNode);
                visitedNode.insert(currentNode);
                queue.erase(queue.begin());
                if (connectionGraph.count(currentNode))
                {
                    for (auto node: connectionGraph.at(currentNode))
                    {
                        if (not visitedNode.count(node))
                        {
                            queue.push_back(node);
                        }
                    }
                }
            }
            clusters[nodes.first] = cluster;
            int a = cluster.size();
            if (a > mainClusterSize)
            {
                mainClusterSize = cluster.size();
                mainClusterKey = nodes.first;
            }
        }
    }
    std::cout << "step 10/12 clean road network is done" << std::endl;
    std::cout << "step 10/12 total clusters: " << clusters.size() << std::endl;
    std::cout << "step 10/12 main cluster size: " << mainClusterSize << " " << ((double) mainClusterSize / (double) this->m_allIntersections.size()) * 100 << "%"
              << std::endl;
    const auto &mainCluster = clusters.at(mainClusterKey);

    std::set<Road::Line::id_t> mainClustersLineId;
    for (const auto &nodeId: mainCluster)
    {
        for (const auto &subWayId: this->getNodeObjById(nodeId)->getSubWayParentId())
        {
            mainClustersLineId.insert(this->getWayObjById(this->getSubwayObjById(subWayId)->getWayParentId())->getLineParentId());
        }
    }
    std::map<std::string, std::map<Road::Line::id_t, OSMLine::ptr>> simplifiedNetwork;
    for (const auto &it: this->m_simplifiedNetwork)
    {
        for (const auto &it2: it.second)
        {
            if (mainClustersLineId.count(it2.first))
            {
                if (!simplifiedNetwork.count(it.first)) simplifiedNetwork[it.first] = {};
                simplifiedNetwork.at(it.first).insert(it2);
            }
        }
    }
    this->m_simplifiedNetwork = simplifiedNetwork;
}

int OSMHandler::getLocalWayIndexInLine(const Road::Way::id_t wayId)
{
    const auto &way = this->m_allWayId2WayObj.at(wayId);
    const auto &parentLine = this->m_allLineId2LineObj.at(way->getLineParentId());
    for (int i = 0; i < parentLine->getWayObjs().size(); i++)
    {
        if (parentLine->getWayObjs().at(i)->getWayId() == wayId)
        {
            return i;
        }
    }
    return -1;
}
int OSMHandler::getLocalSubWayIndexInWay(const Road::SubWay::id_t subWayId)
{
    const auto &subWay = this->m_allSubWayId2SubWayObjs.at(subWayId);
    const auto &parentWay = this->m_allWayId2WayObj.at(subWay->getWayParentId());
    for (int i = 0; i < parentWay->getSubWayObjs().size(); i++)
    {
        if (parentWay->getSubWayObjs().at(i)->getSubwayGlobalId() == subWayId)
        {
            return i;
        }
    }
    return -1;
}

OSMLine::ptr OSMHandler::getLineObjById(const Road::Line::id_t &lineId)
{
    return this->m_allLineId2LineObj.at(lineId);
}
OSMWay::ptr OSMHandler::getWayObjById(const Road::Way::id_t &wayId)
{
    return this->m_allWayId2WayObj.at(wayId);
}
OSMSubWay::ptr OSMHandler::getSubwayObjById(const Road::SubWay::id_t &subWayId)
{
    return this->m_allSubWayId2SubWayObjs.at(subWayId);
}
OSMNode::ptr OSMHandler::getNodeObjById(const Road::Node::id_t &nodeId)
{
    return this->m_allNodeId2NodeObj.at(nodeId);
}

void OSMHandler::createLine()
{
}
OSMNode::ptr OSMHandler::createNode(const osmium::Node &node)
{
    // create node obj
    SlonlatPosition::ptr newSlonlat = std::make_shared<SlonlatPosition>(node.location().lon(), node.location().lat());
    OSMNode::ptr newNode = std::make_shared<OSMNode>(node.id(), newSlonlat);
    // add tags to node obj
    std::map<std::string, std::string> nodeTagsMap;
    for (const auto &it: node.tags())
    {
        if (it.key() == "oneway")
        {
            nodeTagsMap[it.key()] = it.value();
        }
    }
    newNode->setNodeTags(nodeTagsMap);

    return this->m_allNodeId2NodeObj[node.id()] = newNode;
}
OSMNode::ptr OSMHandler::createNode(double lon, double lat)
{
    // create node obj
    SlonlatPosition::ptr newSlonlat = std::make_shared<SlonlatPosition>(lon, lat);
    const auto &id = OSMGenerateGlobalId::getInstance()->getNewNodeId();
    OSMNode::ptr newNode = std::make_shared<OSMNode>(id, newSlonlat);
    return this->m_allNodeId2NodeObj[id] = newNode;
}
void OSMHandler::createWay(const osmium::Way &way)
{
}
OSMSubWay::ptr OSMHandler::createSubWay(Road::Way::id_t parentId,
                                        OSMNode::ptr fromNode,
                                        OSMNode::ptr toNode,
                                        std::map<std::string, std::string> wayTagsMap)
{
    OSMSubWay::ptr newSubwayObj = std::make_shared<OSMSubWay>(parentId, fromNode, toNode, wayTagsMap);
    this->m_allSubWayId2SubWayObjs[newSubwayObj->getSubwayGlobalId()] = newSubwayObj;
    return newSubwayObj;
}
void OSMHandler::updateSubWayOrder(Road::Way::id_t wayId, std::vector<OSMNode::ptr> &nodeObjs)
{
    /// update subwayObjs for preWayId;
    std::vector<OSMSubWay::ptr> subwayPreObjs;
    int nodePos = 0;
    OSMNode::ptr fromNodeObjPre;
    OSMNode::ptr toNodeObjPre;
    for (const auto &it: nodeObjs)
    {
        if (nodePos == 0)
        {
            fromNodeObjPre = it;
            nodePos += 1;
            continue;
        } else
        {
            toNodeObjPre = it;
        }
        /// generate subwayObj
        OSMSubWay::ptr newSubway = this->createSubWay(wayId, fromNodeObjPre, toNodeObjPre, this->m_allWayId2WayObj.at(wayId)->getWayTags());
        subwayPreObjs.push_back(newSubway);

        fromNodeObjPre = toNodeObjPre;
        nodePos += 1;
    }

    this->m_allWayId2WayObj.at(wayId)->updateSubWayObj(subwayPreObjs);
}

bool OSMHandler::isTwoWayShareSameNode(Road::SubWay::id_t way1FrontSubwayId,
                                       Road::SubWay::id_t way1BackSubwayId,
                                       Road::SubWay::id_t way2FrontSubwayId,
                                       Road::SubWay::id_t way2BackSubwayId)
{
    Road::Node::id_t way1FrontSubWayFromNode = this->m_allSubWayId2SubWayObjs.at(way1FrontSubwayId)->getFromNodeRefObj()->getNodeId();
    Road::Node::id_t way1BackSubWayToNode = this->m_allSubWayId2SubWayObjs.at(way1BackSubwayId)->getToNodeRefObj()->getNodeId();

    Road::Node::id_t way2FrontSubWayFromNode = this->m_allSubWayId2SubWayObjs.at(way2FrontSubwayId)->getFromNodeRefObj()->getNodeId();
    Road::Node::id_t way2BackSubWayToNode = this->m_allSubWayId2SubWayObjs.at(way2BackSubwayId)->getToNodeRefObj()->getNodeId();

    if (way1FrontSubWayFromNode == way2FrontSubWayFromNode or way1FrontSubWayFromNode == way2BackSubWayToNode or
        way1BackSubWayToNode == way2FrontSubWayFromNode or way1BackSubWayToNode == way2BackSubWayToNode)
    {
        return true;
    }
    return false;
}

std::tuple<int, double> OSMHandler::calAngleOfConnectedSubWay(Road::SubWay::id_t way1FrontSubwayId,
                                                              Road::SubWay::id_t way1BackSubwayId,
                                                              Road::SubWay::id_t way2FrontSubwayId,
                                                              Road::SubWay::id_t way2BackSubwayId)
{
    Road::Node::id_t way1FrontSubWayFromNode = this->m_allSubWayId2SubWayObjs.at(way1FrontSubwayId)->getFromNodeRefObj()->getNodeId();
    Road::Node::id_t way1BackSubWayToNode = this->m_allSubWayId2SubWayObjs.at(way1BackSubwayId)->getToNodeRefObj()->getNodeId();

    Road::Node::id_t way2FrontSubWayFromNode = this->m_allSubWayId2SubWayObjs.at(way2FrontSubwayId)->getFromNodeRefObj()->getNodeId();
    Road::Node::id_t way2BackSubWayToNode = this->m_allSubWayId2SubWayObjs.at(way2BackSubwayId)->getToNodeRefObj()->getNodeId();

    std::tuple<int, double> angleMap;

    if (way1FrontSubWayFromNode == way2FrontSubWayFromNode)
    {// 180

        double angle = calAngleOfTwoVectors(this->m_allSubWayId2SubWayObjs.at(way1FrontSubwayId)->getFromNodeRefObj()->getSlonlatPosition(),
                                            this->m_allSubWayId2SubWayObjs.at(way1FrontSubwayId)->getToNodeRefObj()->getSlonlatPosition(),
                                            this->m_allSubWayId2SubWayObjs.at(way2FrontSubwayId)->getFromNodeRefObj()->getSlonlatPosition(),
                                            this->m_allSubWayId2SubWayObjs.at(way2FrontSubwayId)->getToNodeRefObj()->getSlonlatPosition());
        angle = abs(180 - angle);

        angleMap = std::make_tuple(1, angle);

        return angleMap;

    } else if (way1FrontSubWayFromNode == way2BackSubWayToNode)
    {
        double angle = calAngleOfTwoVectors(this->m_allSubWayId2SubWayObjs.at(way1FrontSubwayId)->getFromNodeRefObj()->getSlonlatPosition(),
                                            this->m_allSubWayId2SubWayObjs.at(way1FrontSubwayId)->getToNodeRefObj()->getSlonlatPosition(),
                                            this->m_allSubWayId2SubWayObjs.at(way2BackSubwayId)->getFromNodeRefObj()->getSlonlatPosition(),
                                            this->m_allSubWayId2SubWayObjs.at(way2BackSubwayId)->getToNodeRefObj()->getSlonlatPosition());

        angleMap = std::make_tuple(2, angle);

        return angleMap;

    } else if (way1BackSubWayToNode == way2FrontSubWayFromNode)
    {
        double angle = calAngleOfTwoVectors(this->m_allSubWayId2SubWayObjs.at(way1BackSubwayId)->getFromNodeRefObj()->getSlonlatPosition(),
                                            this->m_allSubWayId2SubWayObjs.at(way1BackSubwayId)->getToNodeRefObj()->getSlonlatPosition(),
                                            this->m_allSubWayId2SubWayObjs.at(way2FrontSubwayId)->getFromNodeRefObj()->getSlonlatPosition(),
                                            this->m_allSubWayId2SubWayObjs.at(way2FrontSubwayId)->getToNodeRefObj()->getSlonlatPosition());
        angleMap = std::make_tuple(3, angle);

        return angleMap;

    } else if (way1BackSubWayToNode == way2BackSubWayToNode)
    {
        double angle = calAngleOfTwoVectors(this->m_allSubWayId2SubWayObjs.at(way1BackSubwayId)->getFromNodeRefObj()->getSlonlatPosition(),
                                            this->m_allSubWayId2SubWayObjs.at(way1BackSubwayId)->getToNodeRefObj()->getSlonlatPosition(),
                                            this->m_allSubWayId2SubWayObjs.at(way2BackSubwayId)->getFromNodeRefObj()->getSlonlatPosition(),
                                            this->m_allSubWayId2SubWayObjs.at(way2BackSubwayId)->getToNodeRefObj()->getSlonlatPosition());
        angle = abs(180 - angle);

        angleMap = std::make_tuple(4, angle);

        return angleMap;
    }
}
const std::set<Road::Node::id_t> &OSMHandler::getAllIntersectionNodeIdSet()
{
    return this->m_allIntersections;
}
