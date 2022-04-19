//
// Created by xiaorui on 21.02.22.
//

#include "OSMLine.h"

OSMLine::OSMLine() {
}

OSMLine::~OSMLine() {
}

// bool OSMLines::operator<(const OSMLines &rhs) const {
//     if(this->m_length < rhs.m_length){
//         return true;
//     }
//     return false;
//
// }

OSMLine::OSMLine(Road::Line::id_t id, std::vector<OSMWay::ptr>& wayObjs, std::map<std::string, std::string>& lineTags) {
    this->m_id       = id;
    this->m_wayObjs  = wayObjs;
    this->m_lineTags = lineTags;
    this->calLineLength();
}

const Road::Line::id_t OSMLine::getLineId() {
    return this->m_id;
}

const std::vector<OSMWay::ptr>& OSMLine::getWayObjs() {
    return this->m_wayObjs;
}

const std::map<std::string, std::string>& OSMLine::getLineTags() {
    return this->m_lineTags;
}

const double OSMLine::getLineLength() {
    return this->m_length;
}

const std::map<Road::Line::id_t, OSMLine::ptr>& OSMLine::getSwallowedLines() {
    return this->m_swallowedLines;
}

void OSMLine::addSwallowedLine(Road::Line::id_t lineId, OSMLine::ptr lineObj) {
    this->m_swallowedLines.insert(std::pair<Road::Line::id_t, OSMLine::ptr>(lineId, lineObj));
}
void OSMLine::updateWayObjs(const std::vector<OSMWay::ptr>& ways) {
    for (const auto& way : this->m_wayObjs) {
        ///< delete old way's parent
        way->removeParentLineId();
    }

    ///< add parent info back
    for (const auto& way : ways) {
        way->updateParentLineId(this->m_id);
    }
    this->m_wayObjs = ways;
}
void OSMLine::calLineLength() {
    this->m_length = 0.0;
    for (const auto& wayObj : this->m_wayObjs) {
        this->m_length += wayObj->getWayLength();
    }
}
void OSMLine::deleteWayFromHead(int elementNum) {
    assert(elementNum <= this->m_wayObjs.size());
    for (int i = 0; i < elementNum; i++) {
        this->m_wayObjs.at(i)->removeParentLineId();
    }

    this->m_wayObjs.erase(this->m_wayObjs.begin(), this->m_wayObjs.begin() + elementNum);
}
void OSMLine::deleteWayFromTail(int elementNum) {
    assert(elementNum <= this->m_wayObjs.size());

    for (int i = this->m_wayObjs.size() - 1; i >= this->m_wayObjs.size() - elementNum; i--) {
        this->m_wayObjs.at(i)->removeParentLineId();
    }
    this->m_wayObjs.erase(this->m_wayObjs.end() - elementNum, this->m_wayObjs.end());
}