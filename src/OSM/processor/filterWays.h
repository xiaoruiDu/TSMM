
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

            for (auto &way: map.ways())
            {
                if (callBack_(way.second))
                    way.second->deactive();
            }
        }

    private:
        std::vector<std::string> targetTags_;
        CallBack callBack_;
    };

}// namespace TSMM::OSM


#endif//FILTERWAYS_H
