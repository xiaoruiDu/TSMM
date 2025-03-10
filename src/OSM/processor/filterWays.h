
#ifndef FILTERWAYS_H
#define FILTERWAYS_H

#include <functional>
#include <processor/OSMProcessorBase.h>

namespace TSMM::OSM
{

    class FilterWays : public OSMProcessorBase
    {

    public:
        using CallBack = std::function<bool(std::shared_ptr<OSMWay> &)>;
        explicit FilterWays(CallBack call_back) : OSMProcessorBase(), callBack_(std::move(call_back))
        {
        }

        void process(OSMMap &map) override
        {
            std::for_each(map.ways().begin(), map.ways().end(), [&](auto &pair) {
                if (callBack_(pair.second))
                    pair.second->deactivate();
            });
        }

    private:
        CallBack callBack_;
    };

}// namespace TSMM::OSM


#endif//FILTERWAYS_H
