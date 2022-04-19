//
// Created by xiaorui on 16.02.22.
//

#include "OSMNode.h"

OSMNode::OSMNode() {
}

OSMNode::OSMNode(const Road::Node::id_t nodeId, const SlonlatPosition::ptr position) {
    this->m_nodeId = nodeId;
    this->lat      = position->lat;
    this->lon      = position->lon;
    this->m_lonlat = position;
}

OSMNode::~OSMNode() {
}

void OSMNode::setNodeTags(std::map<std::string, std::string> nodeTagsMap) {
    this->m_nodeTagsMap = nodeTagsMap;
}

void OSMNode::setIntersectionLevel(std::set<std::string> intersectionLevel) {
    this->m_intersectionLevel = intersectionLevel;
}

void OSMNode::setSubWayParentId(Road::SubWay::id_t subwayGlobalId) {
    this->m_parentId.insert(subwayGlobalId);
}

const std::map<std::string, std::string>& OSMNode::getNodeTags() {
    return this->m_nodeTagsMap;
}

const SlonlatPosition::ptr OSMNode::getSlonlatPosition() {
    return this->m_lonlat;
}

const double OSMNode::getNodeLat() {
    return this->lat;
}

const double OSMNode::getNodeLon() {
    return this->lon;
}

const Road::Node::id_t OSMNode::getNodeId() {
    return this->m_nodeId;
}

const osmium::Location OSMNode::getOsmLocation() {
    osmium::Location node(this->lon, this->lat);
    this->m_osmNodeLocation = node;
    return this->m_osmNodeLocation;
}

const std::set<std::string>& OSMNode::getIntersectionLevel() {
    return this->m_intersectionLevel;
}

const std::set<Road::SubWay::id_t>& OSMNode::getSubWayParentId() {
    return this->m_parentId;
}
void OSMNode::insertIntersectionLevel(const std::set<std::string>& intersection) {
    this->m_intersectionLevel.insert(intersection.begin(), intersection.end());
}

void OSMNode::removeParentSubWayId(Road::SubWay::id_t parentId) {
    assert(this->m_parentId.count(parentId));

    this->m_parentId.erase(parentId);
}

void OSMNode::updateParentSubWayId(Road::Way::id_t parentId) {

    this->m_parentId.insert(parentId);

}