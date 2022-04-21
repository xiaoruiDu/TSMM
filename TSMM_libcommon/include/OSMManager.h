//
// Created by xiaorui on 16.02.22.
//

#ifndef BENCHMARK_OSMMANAGER_H
#define BENCHMARK_OSMMANAGER_H
#include <memory>
#include <string>
// We want to use the handler interface
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/handler.hpp>
#include <osmium/io/writer.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>
// Allow any format of input files (XML, PBF, ...)
#include "OSMGenerateGlobalId.h"
#include "OSMHandler.h"

#include <osmium/io/any_input.hpp>
#include <osmium/io/any_output.hpp>
#include <osmium/visitor.hpp>

class OSMManager
{
protected:
private:
public:
    typedef std::shared_ptr<OSMManager> ptr;
    OSMManager();
    OSMManager(std::string osmFilePath);
    ~OSMManager();
    void initialize();
    void run();

    void generateOsmFile(std::string osmFileNamePrefix);
    void generateFinialOsmFile(std::string osmFileNamePrefix);

    void writeLayerByLayer(std::string writeItem,
                           const char *outputPath,
                           osmium::memory::Buffer &tmp_nodeBuffer,
                           osmium::memory::Buffer &tmp_wayBuffer,
                           osmium::memory::Buffer &tmp_relationBuffer);

private:
    const std::size_t buffer_size = 10240;
    osmium::memory::Buffer m_nodeBuffer{buffer_size, osmium::memory::Buffer::auto_grow::yes};
    osmium::memory::Buffer m_wayBuffer{buffer_size, osmium::memory::Buffer::auto_grow::yes};
    osmium::memory::Buffer m_relationBuffer{buffer_size, osmium::memory::Buffer::auto_grow::yes};

    OSMHandler::ptr m_osmHandler;
    std::string m_osmFilePath;
    std::set<std::string> m_remainedLayerValuesSet;
    OSMGenerateGlobalId *m_ptrGlobalIdGenerator;

    std::map<std::string, int> m_defaultLaneNum;

private:
    void generateNodes(osmium::memory::Buffer &tmp_nodeBuffer);
    void generateWays(osmium::memory::Buffer &tmp_wayBuffer);
    void generateRelation(osmium::memory::Buffer &tmp_relationBuffer);
    void generateNodesForEachLayer(const std::string &layerName, osmium::memory::Buffer &tmp_nodeBuffer);
    void generateWaysForEachLayer(const std::string &layerName, osmium::memory::Buffer &tmp_wayBuffer);
    void generateRelationForEachLayer(const std::string &layerName, osmium::memory::Buffer &tmp_relationBuffer);
    void addWayTagsForEachLayer(osmium::memory::Buffer &tmp_wayBuffer,
                                osmium::builder::Builder *builder,
                                const std::map<std::string, std::string> &wayTagsMap);
    void addWayNodeRefsForEachLayer(osmium::memory::Buffer &tmp_wayBuffer,
                                    osmium::builder::Builder *builder,
                                    const std::vector<OSMNode::ptr> &nodeRefsObjs);

    ///< final osm file needs to change remained ways' tags;
    void generateFinalNodes(osmium::memory::Buffer &tmp_nodeBuffer);
    void generateFinalWays(osmium::memory::Buffer &tmp_wayBuffer);
    void generateFinalRelation(osmium::memory::Buffer &tmp_relationBuffer);
    void addFinalNodeTags(osmium::memory::Buffer &buffer, osmium::builder::Builder *builder);
    void addFinalWayTagsForEachLayer(osmium::memory::Buffer &tmp_wayBuffer,
                                     osmium::builder::Builder *builder,
                                     const std::map<std::string, std::string> &wayTagsMap);
};

#endif// BENCHMARK_OSMMANAGER_H
