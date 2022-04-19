//
// Created by xiaorui on 16.02.22.
//

#ifndef BENCHMARK_OSMWAY_H
#define BENCHMARK_OSMWAY_H

#include "OSMGenerateGlobalId.h"
#include "OSMNode.h"
#include "OSMVariableName.h"

#include <iostream>
#include <map>
#include <memory>

class OSMSubWay {
public:
    typedef std::shared_ptr<OSMSubWay> ptr;
    OSMSubWay();
    ~OSMSubWay();
    ///< if just create temporary subwayObjs, then set their parentId = -1
    OSMSubWay(Road::Way::id_t parentId, OSMNode::ptr fromNodeRefObj, OSMNode::ptr toNodeRefObj, const std::map<std::string, std::string>& subWayTagsMap);
    ///< get method
    const OSMNode::ptr getFromNodeRefObj();
    const OSMNode::ptr getToNodeRefObj();
    //    const Road::SubWay::id_t getSubwayLocalId();
    const Road::SubWay::id_t getSubwayGlobalId();
    const std::map<std::string, std::string>& getSubWayTagsMap();
    const Road::Way::id_t getWayParentId();


    const double getLength();

    ///< update info
    void removeParentWayId();
    void updateParentWayId(Road::Way::id_t parentId);
    void updateFromNode(const std::shared_ptr<OSMNode> fromNodeRefObj);
    void updateToNode(const std::shared_ptr<OSMNode> toNodeRefObj);

private:
    void calSubwayLength();

private:
    std::map<std::string, std::string> m_subWayTagsMap;
    //    std::vector<OSMNode::ptr> m_nodeRefObjs;
    OSMNode::ptr m_fromNodeRefObj;
    OSMNode::ptr m_toNodeRefObj;
    Road::SubWay::id_t m_globalId;
    //    Road::SubWay::id_t m_localId;
    Road::Way::id_t m_parentId;
    double m_length;
};

class OSMWay {
public:
    typedef std::shared_ptr<OSMWay> ptr;

    OSMWay();

    OSMWay(Road::Way::id_t wayId, std::vector<OSMSubWay::ptr>& subWayObjVector);

    ~OSMWay();

    ///< set methods
    void setWayTags(const std::map<std::string, std::string>& wayTagsMap);

    void setWayLength(const double length);

    void setWayParentLineId(const Road::Line::id_t parentId);

    ///< update methods
    void updateSubWayObj(const std::vector<OSMSubWay::ptr> subWayObjs);

    void insertNodeObj(int insertPos, OSMNode::ptr nodeObj);
    void insertSubWayObj(const OSMSubWay::ptr& subway, int pos = 0);
    void addOrUpdateTags(std::string key, std::string value);

    void removeParentLineId();
    void updateParentLineId(Road::Line::id_t parentId);

    void deleteNodeObj(int insertPos);

    ///< get methods
    const double getWayLength();

    const int getLocalIndexOfLine();

    const Road::Way::id_t getWayId();

    const std::map<std::string, std::string> getWayTags();

    const std::vector<OSMNode::ptr> getNodeRefObjs();

    const std::vector<OSMSubWay::ptr>& getSubWayObjs();

    const Road::Line::id_t getLineParentId() const;

    const std::map<Road::SubWay::id_t, OSMSubWay::ptr>& getSubWayId2SubWayObjs() const;

    ///< add methods

    ///< others
    //    void updateSubWayObjs();
    void deleteSubWayObjFromHead(int elementNum);
    void deleteSubWayObjFromTail(int elementNum);
    ///< TODO others

private:
    std::map<std::string, std::string> m_wayTagsMap;
    //    std::vector<OSMNode::ptr> m_nodeRefObjs;
    std::vector<OSMSubWay::ptr> m_subWayObjs;
    std::map<Road::SubWay::id_t, OSMSubWay::ptr> m_subWayId2SubWayObjs;
    double m_length;
    Road::Way::id_t m_id;
    Road::Line::id_t m_parentId;
};

#endif // BENCHMARK_OSMWAY_H
