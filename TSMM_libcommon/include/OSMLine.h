//
// Created by xiaorui on 21.02.22.
//

#ifndef BENCHMARK_OSMLINE_H
#define BENCHMARK_OSMLINE_H

#include "OSMCoreTypes.h"
#include "OSMWay.h"

#include <memory>

class OSMLine {
public:
    typedef std::shared_ptr<OSMLine> ptr;

    OSMLine(Road::Line::id_t id, std::vector<OSMWay::ptr>& wayObjs, std::map<std::string, std::string>& lineTags);

    OSMLine();

    ~OSMLine();

    ///< get methods
    const Road::Line::id_t getLineId();

    const std::vector<OSMWay::ptr>& getWayObjs();

    const std::map<std::string, std::string>& getLineTags();

    const double getLineLength();

    const std::map<Road::Line::id_t, OSMLine::ptr>& getSwallowedLines();

    ///< set methods

    ///< update methods
    void updateWayObjs(const std::vector<OSMWay::ptr>&);
    ///< add methods
    void addSwallowedLine(Road::Line::id_t lineId, OSMLine::ptr lineObj);

    /// TODO < others
    void deleteWayFromHead(int elementNum);
    void deleteWayFromTail(int elementNum);


    void insertWayObj(int insertPos, OSMWay::ptr wayObj);

    void deleteWayObj(int insertPos, OSMWay::ptr wayObj);

private:
    void calLineLength();

private:
    Road::Line::id_t m_id;
    std::vector<OSMWay::ptr> m_wayObjs;
    std::map<std::string, std::string> m_lineTags;
    double m_length;

    std::map<Road::Line::id_t, OSMLine::ptr> m_swallowedLines;
};

#endif // BENCHMARK_OSMLINE_H
