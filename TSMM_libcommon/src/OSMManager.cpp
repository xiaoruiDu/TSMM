//
// Created by xiaorui on 16.02.22.
//

#include "OSMManager.h"

OSMManager::OSMManager() {
}

OSMManager::OSMManager(std::string osmFilePath): m_osmFilePath(osmFilePath) {
    std::string keyString = "highway";
    std::set<std::string> remainedLayerValuesSe;
    remainedLayerValuesSe.insert("motorway");
    remainedLayerValuesSe.insert("trunk");
    remainedLayerValuesSe.insert("primary");
    remainedLayerValuesSe.insert("secondary");
    remainedLayerValuesSe.insert("tertiary");
    remainedLayerValuesSe.insert("residential");

    this->m_defaultLaneNum.insert(std::make_pair("motorway", 4));
    this->m_defaultLaneNum.insert(std::make_pair("trunk", 4));
    this->m_defaultLaneNum.insert(std::make_pair("primary", 4));
    this->m_defaultLaneNum.insert(std::make_pair("secondary", 2));
    this->m_defaultLaneNum.insert(std::make_pair("tertiary", 2));
    this->m_defaultLaneNum.insert(std::make_pair("residential", 1));

    this->m_remainedLayerValuesSet = remainedLayerValuesSe;
    this->m_osmHandler             = std::make_shared<OSMHandler>(keyString, this->m_remainedLayerValuesSet);

    this->m_ptrGlobalIdGenerator = OSMGenerateGlobalId::getInstance();
}

OSMManager::~OSMManager() {
    this->m_ptrGlobalIdGenerator->deleteInstance();
}

void OSMManager::initialize() {
    /// TODO Step1:  config parameters

    /// Step2:  extract road Info

    // The type of index used. This must match the include file above
    using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
    // The location handler always depends on the index type
    using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;

    // The index to hold node locations.
    index_type index;
    // The location handler will add the node locations to the index and then to the ways
    location_handler_type location_handler{index};

    osmium::io::File input_file(this->m_osmFilePath);
    auto otypes = osmium::osm_entity_bits::node | osmium::osm_entity_bits::way;

    osmium::io::Reader readRoadInfo{input_file, otypes};

    osmium::apply(readRoadInfo, location_handler, *this->m_osmHandler);
    readRoadInfo.close();

    ///< start processing osmFile
    this->run();
}

void OSMManager::run() {
    ///< step1: mergeWaysToLine (and output osm file)
    this->m_osmHandler->linkWays2Lines();
    this->generateOsmFile("../outputOSM/01_link_ways_");

    ///< step2: buffer method (process lines layer by layer)
    this->m_osmHandler->generateSimplifiedLines();
    this->generateOsmFile("../outputOSM/02_first_buffer_");
    this->m_osmHandler->printSimplifiedNetwork();

    ///< step3: buffer method (process all layer lines)
    this->m_osmHandler->reBufferAllLayerLines();
    this->generateOsmFile("../outputOSM/03_re_buffer_");

    ///< step4: order node refs.
    this->m_osmHandler->orderWayNodeRef();

    /// step5: extend lines at lines' tail and head end
    this->m_osmHandler->extendLines();
    this->generateOsmFile("../outputOSM/05_extend_");

    ///<step6: calculate Intersection Info
    std::map<Road::SubWay::id_t, std::vector<Road::Node::id_t>> interSectionInfo = this->m_osmHandler->calIntersectionPoints();

    ///< step7: insert intersection points to subways
    this->m_osmHandler->insertIntersectionPoints(interSectionInfo);
    this->generateOsmFile("../outputOSM/07_insert_");
    this->m_osmHandler->printAllIntersection();

    ///< step8: delete all Redundant road
    this->m_osmHandler->deleteRedundantRoad();
    this->generateOsmFile("../outputOSM/08_del_redundant_");

    ///< step9: merge close intersections
    this->m_osmHandler->mergeNeighborIntersections();
    this->generateOsmFile("../outputOSM/09_merge_");

    ///< step10: cluster
    this->m_osmHandler->cleanNetwork();
    this->generateOsmFile("../outputOSM/10_clean_");

    /// TODO < step11: add ramp template intersections
    void buildRamp();

    ///< step12: generate final osmFile
    this->generateFinialOsmFile("../outputOSM/12_final_");

}

void OSMManager::generateOsmFile(std::string osmFileNamePrefix) {
    const std::size_t buffer_size = 10240;
    osmium::memory::Buffer tmp_nodeBuffer{buffer_size, osmium::memory::Buffer::auto_grow::yes};
    osmium::memory::Buffer tmp_wayBuffer{buffer_size, osmium::memory::Buffer::auto_grow::yes};
    osmium::memory::Buffer tmp_relationBuffer{buffer_size, osmium::memory::Buffer::auto_grow::yes};

    this->generateNodes(tmp_nodeBuffer);

    // generate WayInfo
    this->generateWays(tmp_wayBuffer);

    //    // generate RelationInfo
    this->generateRelation(tmp_relationBuffer);

    std::string path = osmFileNamePrefix + "_allLayer" + ".osm";
    this->writeLayerByLayer("all", path.c_str(), tmp_nodeBuffer, tmp_wayBuffer, tmp_relationBuffer);
}

void OSMManager::generateOsmFileForEachLayer(std::string osmFileNamePrefix) {
    for (const auto& layerName : this->m_remainedLayerValuesSet) {
        const std::size_t buffer_size = 10240;
        osmium::memory::Buffer tmp_nodeBuffer{buffer_size, osmium::memory::Buffer::auto_grow::yes};
        osmium::memory::Buffer tmp_wayBuffer{buffer_size, osmium::memory::Buffer::auto_grow::yes};
        osmium::memory::Buffer tmp_relationBuffer{buffer_size, osmium::memory::Buffer::auto_grow::yes};

        this->generateNodesForEachLayer(layerName, tmp_nodeBuffer);

        this->generateWaysForEachLayer(layerName, tmp_wayBuffer);

        this->generateRelationForEachLayer(layerName, tmp_relationBuffer);

        std::string path = osmFileNamePrefix + layerName + ".osm";
        this->writeLayerByLayer("all", path.c_str(), tmp_nodeBuffer, tmp_wayBuffer, tmp_relationBuffer);
    }
}

void OSMManager::generateNodes(osmium::memory::Buffer& tmp_nodeBuffer) {
    for (const auto& nodeInfo : this->m_osmHandler->getAllNodeId2NodeObj()) {
        {
            osmium::builder::NodeBuilder builder{tmp_nodeBuffer};
            builder.add_user("xiaorui");
            osmium::Node& obj = builder.object();

            obj.set_id(nodeInfo.first);
            obj.set_uid(nodeInfo.first);
            obj.set_location(nodeInfo.second->getOsmLocation());
        }
        tmp_nodeBuffer.commit();
    }
}

void OSMManager::generateWays(osmium::memory::Buffer& tmp_wayBuffer) {
    for (const auto& lineLayerInfoDic : this->m_osmHandler->getAggregatedLineInfoMap()) {
        for (const auto& line : lineLayerInfoDic.second) {
            for (const auto& way : line.second->getWayObjs()) {
                {
                    osmium::builder::WayBuilder builder{tmp_wayBuffer};
                    builder.add_user("xiaorui");
                    osmium::Way& obj = builder.object();

                    obj.set_id(way->getWayId());
                    obj.set_uid(way->getWayId());

                    way->addOrUpdateTags("line", std::to_string(way->getLineParentId()));

                    this->addWayNodeRefsForEachLayer(tmp_wayBuffer, &builder, way->getNodeRefObjs());
                    this->addWayTagsForEachLayer(tmp_wayBuffer, &builder, way->getWayTags());
                }
                tmp_wayBuffer.commit();
            }
        }
    }
}

void OSMManager::generateRelation(osmium::memory::Buffer& tmp_relationBuffer) {
}

void OSMManager::generateNodesForEachLayer(const std::string& layerName, osmium::memory::Buffer& tmp_nodeBuffer) {
    for (const auto& nodeInfo : this->m_osmHandler->getAllNodeId2NodeObj()) {
        {
            osmium::builder::NodeBuilder builder{tmp_nodeBuffer};
            builder.add_user("xiaorui");
            osmium::Node& obj = builder.object();

            obj.set_id(nodeInfo.first);
            obj.set_uid(nodeInfo.first);
            obj.set_location(nodeInfo.second->getOsmLocation());
        }
        tmp_nodeBuffer.commit();
    }

    //    ///< process node layer by layer
    //    for (const auto& nodeLayerInfoDic : this->m_osmHandler->getAggregatedNodeInfoMap()) {
    //        if (nodeLayerInfoDic.first != layerName) {
    //            continue;
    //        }
    //        ///< some points will be added multiple times since they are belonded to multiple layer
    //        for (const auto& nodeInfo : nodeLayerInfoDic.second) {
    //            {
    //                osmium::builder::NodeBuilder builder{tmp_nodeBuffer};
    //                builder.add_user("xiaorui");
    //                osmium::Node& obj = builder.object();
    //
    //                obj.set_id(nodeInfo.first);
    //                obj.set_uid(nodeInfo.first);
    //                obj.set_location(nodeInfo.second->getOsmLocation());
    //            }
    //            tmp_nodeBuffer.commit();
    //        }
    //    }
}

void OSMManager::generateWaysForEachLayer(const std::string& layerName, osmium::memory::Buffer& tmp_wayBuffer) {
    for (const auto& lineLayerInfoDic : this->m_osmHandler->getAggregatedLineInfoMap()) {
        if (lineLayerInfoDic.first != layerName) {
            continue;
        }

        for (const auto& line : lineLayerInfoDic.second) {
            for (const auto& way : line.second->getWayObjs()) {
                {
                    osmium::builder::WayBuilder builder{tmp_wayBuffer};
                    builder.add_user("xiaorui");
                    osmium::Way& obj = builder.object();

                    obj.set_id(way->getWayId());
                    obj.set_uid(way->getWayId());
                    this->addWayNodeRefsForEachLayer(tmp_wayBuffer, &builder, way->getNodeRefObjs());

                    this->addWayTagsForEachLayer(tmp_wayBuffer, &builder, way->getWayTags());
                }
                tmp_wayBuffer.commit();
            }
        }
    }
}

void OSMManager::generateRelationForEachLayer(const std::string& layerName, osmium::memory::Buffer& tmp_relationBuffer) {
}

void OSMManager::addWayTagsForEachLayer(osmium::memory::Buffer& tmp_wayBuffer,
                                        osmium::builder::Builder* builder,
                                        const std::map<std::string, std::string>& wayTagsMap) {
    // add way nodeTags
    osmium::builder::TagListBuilder tl_builder{tmp_wayBuffer, builder};
    // add road level tag
    for (const auto& tags : wayTagsMap) {
        tl_builder.add_tag(tags.first, tags.second);
    }

    //    tl_builder.add_tag(tagRoaldLevelKey, tagRoaldLevelValue);
    //    tl_builder.add_tag(tagLaneKey, tagLaneValue);
    //    if (this->m_oneWayIds.count(wayId)) {
    //        tl_builder.add_tag("oneway", "yes");
    //    }
}

void OSMManager::addWayNodeRefsForEachLayer(osmium::memory::Buffer& tmp_wayBuffer,
                                            osmium::builder::Builder* builder,
                                            const std::vector<OSMNode::ptr>& nodeRefsObjs) {
    osmium::builder::WayNodeListBuilder wayNodeListBuilder{tmp_wayBuffer, builder};
    for (const auto& nodeObj : nodeRefsObjs) {
        osmium::Location node   = nodeObj->getOsmLocation();
        Road::Node::id_t nodeId = nodeObj->getNodeId();
        osmium::NodeRef nodeRef{long(nodeId), node};
        wayNodeListBuilder.add_node_ref(nodeRef);
    }
}

void OSMManager::write(const std::string writeItem, const char* outputPath) {
    if (remove(outputPath) != 0)
        perror("Error deleting file");
    else
        puts("File successfully deleted");

    osmium::io::File output_file{outputPath};
    osmium::io::Writer writer{output_file};
    if (writeItem == "nodes") {
        writer(std::move(this->m_nodeBuffer));

    } else if (writeItem == "ways") {
        writer(std::move(this->m_wayBuffer));
    } else if (writeItem == "all") {
        writer(std::move(this->m_nodeBuffer));
        //        writer(std::move(this->m_wayBuffer));
        //        writer(std::move(this->m_relationBuffer));
    }
    writer.close();
}

void OSMManager::writeLayerByLayer(std::string writeItem,
                                   const char* outputPath,
                                   osmium::memory::Buffer& tmp_nodeBuffer,
                                   osmium::memory::Buffer& tmp_wayBuffer,
                                   osmium::memory::Buffer& tmp_relationBuffer) {
    if (remove(outputPath) != 0)
        perror("Error deleting file");
    else
        puts("File successfully deleted");

    osmium::io::File output_file{outputPath};
    osmium::io::Writer writer{output_file};
    if (writeItem == "nodes") {
        writer(std::move(tmp_nodeBuffer));

    } else if (writeItem == "ways") {
        writer(std::move(tmp_wayBuffer));
    } else if (writeItem == "all") {
        writer(std::move(tmp_nodeBuffer));
        writer(std::move(tmp_wayBuffer));
        writer(std::move(tmp_relationBuffer));
    }
    writer.close();
}
void OSMManager::generateFinialOsmFile(std::string osmFileNamePrefix) {
    const std::size_t buffer_size = 10240;
    osmium::memory::Buffer tmp_nodeBuffer{buffer_size, osmium::memory::Buffer::auto_grow::yes};
    osmium::memory::Buffer tmp_wayBuffer{buffer_size, osmium::memory::Buffer::auto_grow::yes};
    osmium::memory::Buffer tmp_relationBuffer{buffer_size, osmium::memory::Buffer::auto_grow::yes};

    this->generateFinalNodes(tmp_nodeBuffer);

    // generate WayInfo
    this->generateFinalWays(tmp_wayBuffer);

    //    // generate RelationInfo
    this->generateFinalRelation(tmp_relationBuffer);

    std::string path = osmFileNamePrefix + "_allLayer" + ".osm";
    this->writeLayerByLayer("all", path.c_str(), tmp_nodeBuffer, tmp_wayBuffer, tmp_relationBuffer);
}
void OSMManager::generateFinalNodes(osmium::memory::Buffer& tmp_nodeBuffer) {
    for (const auto& nodeInfo : this->m_osmHandler->getAllNodeId2NodeObj()) {
        {
            osmium::builder::NodeBuilder builder{tmp_nodeBuffer};
            builder.add_user("xiaorui");
            osmium::Node& obj = builder.object();

            obj.set_id(nodeInfo.first);
            obj.set_uid(nodeInfo.first);
            obj.set_location(nodeInfo.second->getOsmLocation());

            if (this->m_osmHandler->getAllIntersectionNodeIdSet().count(nodeInfo.first)) {
                if (this->m_osmHandler->getAllNodeId2NodeObj().at(nodeInfo.first)->getIntersectionLevel().size() == 1 and
                    this->m_osmHandler->getAllNodeId2NodeObj().at(nodeInfo.first)->getIntersectionLevel().count("residential")) {
                    continue;
                } else {
                    osmium::builder::TagListBuilder tl_builder{tmp_nodeBuffer, &builder};
                    tl_builder.add_tag("highway", "traffic_signals");
                    tl_builder.add_tag("traffic_signals", "traffic_lights");
                    tl_builder.add_tag("traffic_signals:direction", "traffic_signals:direction");
                }
            }
        }
        tmp_nodeBuffer.commit();
    }
}
void OSMManager::generateFinalWays(osmium::memory::Buffer& tmp_wayBuffer) {
    for (const auto& lineLayerInfoDic : this->m_osmHandler->getAggregatedLineInfoMap()) {
        for (const auto& line : lineLayerInfoDic.second) {
            for (const auto& way : line.second->getWayObjs()) {
                {
                    osmium::builder::WayBuilder builder{tmp_wayBuffer};
                    builder.add_user("xiaorui");
                    osmium::Way& obj = builder.object();

                    obj.set_id(way->getWayId());
                    obj.set_uid(way->getWayId());

                    way->addOrUpdateTags("line", std::to_string(way->getLineParentId()));

                    this->addWayNodeRefsForEachLayer(tmp_wayBuffer, &builder, way->getNodeRefObjs());
                    this->addFinalWayTagsForEachLayer(tmp_wayBuffer, &builder, way->getWayTags());
                }
                tmp_wayBuffer.commit();
            }
        }
    }
}
void OSMManager::generateFinalRelation(osmium::memory::Buffer& tmp_relationBuffer) {
}
void OSMManager::addFinalWayTagsForEachLayer(osmium::memory::Buffer& tmp_wayBuffer,
                                             osmium::builder::Builder* builder,
                                             const std::map<std::string, std::string>& wayTagsMap) {
    // add way nodeTags
    osmium::builder::TagListBuilder tl_builder{tmp_wayBuffer, builder};

    std::map<std::string, std::string> tmpWayTagsMap;
    tmpWayTagsMap = wayTagsMap;
    ///< extend way tags according to swallowed lines;
    if (tmpWayTagsMap.count("lanes")) {
        std::string laneNum    = tmpWayTagsMap.at("lanes");
        tmpWayTagsMap["lanes"] = std::to_string(std::atoi(laneNum.c_str()) * 2);
    } else {
        ///< add default laneNum
        tmpWayTagsMap.insert(std::make_pair("lanes", std::to_string(this->m_defaultLaneNum.at(tmpWayTagsMap.at("highway")) * 2)));
    }
    ///< if there is no "oneway" tag that means the way is a bidirectional way.
    if (tmpWayTagsMap.count("oneway")) {
        tmpWayTagsMap["oneway"] = "no";
    }

    // add road level tag
    for (const auto& tags : tmpWayTagsMap) {
        tl_builder.add_tag(tags.first, tags.second);
    }
}
void OSMManager::addFinalNodeTags(osmium::memory::Buffer& buffer, osmium::builder::Builder* builder) {
    osmium::builder::TagListBuilder tl_builder{this->m_nodeBuffer, builder};
    tl_builder.add_tag("highway", "traffic_signals");
    tl_builder.add_tag("traffic_signals", "traffic_lights");
    tl_builder.add_tag("traffic_signals:direction", "traffic_signals:direction");
}
