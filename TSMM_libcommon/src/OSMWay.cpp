//
// Created by xiaorui on 16.02.22.
//

#include "OSMWay.h"

OSMSubWay::OSMSubWay() {
}

OSMSubWay::~OSMSubWay() {
}

OSMSubWay::OSMSubWay(Road::Way::id_t parentId,
                     OSMNode::ptr fromNodeRefObj,
                     OSMNode::ptr toNodeRefObj,
                     const std::map<std::string, std::string>& subWayTagsMap) {
    this->m_fromNodeRefObj = fromNodeRefObj;
    this->m_toNodeRefObj   = toNodeRefObj;
    this->m_subWayTagsMap  = subWayTagsMap;
    //    this->m_localId        = localId;
    this->m_parentId = parentId;
    this->m_globalId = OSMGenerateGlobalId::getInstance()->getNewSubwayId();
    ///< if just create temporary subwayObjs, then set their parentId = -1
    if (parentId != -1) {
        this->m_fromNodeRefObj->setSubWayParentId(this->m_globalId);
        this->m_toNodeRefObj->setSubWayParentId(this->m_globalId);
    }

    //    this->m_subWayId2SubWayObj.insert(std::pair<Road::SubWay::id_t, OSMSubWay::ptr>(this->m_globalId, this));
    this->calSubwayLength();
}

const OSMNode::ptr OSMSubWay::getFromNodeRefObj() {
    return this->m_fromNodeRefObj;
}

const OSMNode::ptr OSMSubWay::getToNodeRefObj() {
    return this->m_toNodeRefObj;
}


const Road::SubWay::id_t OSMSubWay::getSubwayGlobalId() {
    return this->m_globalId;
}

const std::map<std::string, std::string>& OSMSubWay::getSubWayTagsMap() {
    return this->m_subWayTagsMap;
}

const Road::Way::id_t OSMSubWay::getWayParentId() {
    return this->m_parentId;
}

const double OSMSubWay::getLength() {
    return this->m_length;
}

void OSMSubWay::removeParentWayId() {
    this->m_parentId = -1;

    this->m_fromNodeRefObj->removeParentSubWayId(this->m_globalId);
    this->m_toNodeRefObj->removeParentSubWayId(this->m_globalId);
}

void OSMSubWay::updateParentWayId(Road::Way::id_t parentId) {
    this->m_parentId = parentId;

    this->m_fromNodeRefObj->updateParentSubWayId(this->m_globalId);
    this->m_toNodeRefObj->updateParentSubWayId(this->m_globalId);
}

void OSMSubWay::calSubwayLength() {
    double earthRadius = 6371000; // in meters
    double dLat        = SMath::toRadians(this->m_fromNodeRefObj->getNodeLat() - this->m_toNodeRefObj->getNodeLat());
    double dLng        = SMath::toRadians(this->m_fromNodeRefObj->getNodeLon() - this->m_toNodeRefObj->getNodeLon());

    double a = SMath::sin(dLat / 2) * SMath::sin(dLat / 2) + SMath::cos(SMath::toRadians(this->m_toNodeRefObj->getNodeLat())) *
                                                                 SMath::cos(SMath::toRadians(this->m_fromNodeRefObj->getNodeLat())) *
                                                                 SMath::sin(dLng / 2) * SMath::sin(dLng / 2);
    double c       = 2 * SMath::atan2(SMath::sqrt(a), SMath::sqrt(1 - a));
    this->m_length = earthRadius * c;
}
void OSMSubWay::updateFromNode(const OSMNode::ptr fromNodeRefObj) {
    this->m_fromNodeRefObj->removeParentSubWayId(this->m_globalId);

    this->m_fromNodeRefObj = fromNodeRefObj;
    this->m_fromNodeRefObj->setSubWayParentId(this->m_globalId);
    this->calSubwayLength();
}
void OSMSubWay::updateToNode(const OSMNode::ptr toNodeRefObj) {
    this->m_toNodeRefObj->removeParentSubWayId(this->m_globalId);

    this->m_toNodeRefObj = toNodeRefObj;
    this->m_toNodeRefObj->setSubWayParentId(this->m_globalId);
    this->calSubwayLength();
}

OSMWay::OSMWay() {
}

OSMWay::OSMWay(Road::Way::id_t wayId, std::vector<OSMSubWay::ptr>& subWayObjVector) {
    this->m_id         = wayId;
    this->m_subWayObjs = subWayObjVector;
    this->m_parentId   = -1; ///< means having no parent line;
    //    this->updateSubWayObjs();
}

OSMWay::~OSMWay() {
}

void OSMWay::setWayTags(const std::map<std::string, std::string>& wayTagsMap) {
    this->m_wayTagsMap = wayTagsMap;
}

void OSMWay::setWayLength(const double length) {
    this->m_length = length;
}

void OSMWay::setWayParentLineId(const Road::Line::id_t parentId) {
    this->m_parentId = parentId;
}

void OSMWay::updateSubWayObj(const std::vector<OSMSubWay::ptr> subWayObjs) {
    for (const auto& subway : this->m_subWayObjs) {
        subway->removeParentWayId();
    }
    for (const auto& subway : subWayObjs) {
        subway->updateParentWayId(this->m_id);
    }
    this->m_subWayObjs = subWayObjs;
}

void OSMWay::insertSubWayObj(const OSMSubWay::ptr& subway, int pos) {
    if (pos > 0) {
        assert(subway->getFromNodeRefObj()->getNodeId() == this->m_subWayObjs.at(pos - 1)->getToNodeRefObj()->getNodeId());
    }
    if (pos < this->m_subWayObjs.size() - 1) {
        assert(subway->getToNodeRefObj()->getNodeId() == this->m_subWayObjs.at(pos + 1)->getFromNodeRefObj()->getNodeId());
    }
    this->m_subWayObjs.insert(this->getSubWayObjs().begin() + pos, subway);
}

const Road::Way::id_t OSMWay::getWayId() {
    return this->m_id;
}

const std::map<std::string, std::string> OSMWay::getWayTags() {
    return this->m_wayTagsMap;
}

const double OSMWay::getWayLength() {
    return this->m_length;
}

const std::vector<OSMNode::ptr> OSMWay::getNodeRefObjs() {
    std::vector<OSMNode::ptr> nodeRefObjs;
    int count = 0;
    for (const auto& it : this->m_subWayObjs) {
        if (count == 0) {
            nodeRefObjs.push_back(it->getFromNodeRefObj());
            nodeRefObjs.push_back(it->getToNodeRefObj());
        } else {
            nodeRefObjs.push_back(it->getToNodeRefObj());
        }
        count += 1;
    }
    return nodeRefObjs;
}

const std::vector<OSMSubWay::ptr>& OSMWay::getSubWayObjs() {
    return this->m_subWayObjs;
}

const Road::Line::id_t OSMWay::getLineParentId() const {
    return this->m_parentId;
}

const std::map<Road::SubWay::id_t, OSMSubWay::ptr>& OSMWay::getSubWayId2SubWayObjs() const {
    return this->m_subWayId2SubWayObjs;
}
void OSMWay::deleteSubWayObjFromHead(int elementNum) {
    assert(elementNum <= this->m_subWayObjs.size());
    for (int i = 0; i < elementNum; i++) {
        this->m_subWayObjs.at(i)->removeParentWayId();
    }
    this->m_subWayObjs.erase(this->m_subWayObjs.begin(), this->m_subWayObjs.begin() + elementNum);
}
void OSMWay::deleteSubWayObjFromTail(int elementNum) {
    assert(elementNum <= this->m_subWayObjs.size());
    int size = this->m_subWayObjs.size();

    for (int i = size - 1; i >= (size - elementNum); i--) {
        auto m = this->m_subWayObjs.size();
        this->m_subWayObjs.at(i)->removeParentWayId();
    }

    this->m_subWayObjs.erase(this->m_subWayObjs.end() - elementNum, this->m_subWayObjs.end());
}
void OSMWay::addOrUpdateTags(std::string key, std::string value) {
    this->m_wayTagsMap.insert(std::make_pair(key, value));
}

// void OSMWay::updateSubWayObjs() {
//     ///< clear vector first.
//     this->m_subWayObjs.clear();
//
//     for (int pre = 0, next = 1; next < this->m_nodeRefObjs.size(); pre++, next++) {
//         OSMSubWay::ptr newSubWay =
//             std::make_shared<OSMSubWay>(this->m_id, next, this->m_nodeRefObjs.at(pre), this->m_nodeRefObjs.at(next), this->m_wayTagsMap);
//         this->m_subWayObjs.push_back(newSubWay);
//         this->m_subWayId2SubWayObjs.insert(std::pair<Road::SubWay::id_t, OSMSubWay::ptr>(newSubWay->getSubwayGlobalId(), newSubWay));
//     }
//     int m = 1;
// }

// void OSMWay::insertNodeObj(int insertPos, OSMNode::ptr nodeObj) {
//     // Create Iterator pointing to insertPos_th Position
//     auto itPos = this->m_nodeRefObjs.begin() + insertPos;
//     // Insert element with value nodeObj at insertPos_th Position in vector
//     this->m_nodeRefObjs.insert(itPos, nodeObj);
// }
//
// void OSMWay::deleteNodeObj(int insertPos) {
//     this->m_nodeRefObjs.erase(this->m_nodeRefObjs.begin() + insertPos);
// }

void OSMWay::removeParentLineId() {
    this->m_parentId = -1;
    for (auto& subway : this->m_subWayObjs) {
        subway->removeParentWayId();
    }
}

void OSMWay::updateParentLineId(Road::Line::id_t parentId) {
    this->m_parentId = parentId;
    for (auto& subway : this->m_subWayObjs) {
        subway->updateParentWayId(this->m_id);
    }
}