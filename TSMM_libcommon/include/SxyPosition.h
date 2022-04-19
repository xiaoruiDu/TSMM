//
// Created by xiaorui on 21.02.22.
//

#ifndef BENCHMARK_SXYPOSITION_H
#define BENCHMARK_SXYPOSITION_H

#include <assert.h>
#include <string>
//#include <proj_api.h>
#include "OSMMath.h"

#include <iostream>
#include <memory>

class SlonlatPosition {
public:
    typedef std::shared_ptr<SlonlatPosition> ptr;
    long double lon;
    long double lat;
    /**
     * @brief Constructor
     */
    SlonlatPosition();

    /**
     * @brief Constructor that initializes the member
     * @param lon
     * @param lat
     */
    SlonlatPosition(double lon, double lat);

    /**
     * @brief Constructor that initializes the member
     * @param pos
     */
    SlonlatPosition(SlonlatPosition& pos);

    void operator=(SlonlatPosition& pos);

    /**
     * @brief Returns the distance from this position to another in [meter]
     * @param position the other point
     * @return distance
     */
    double distance(SlonlatPosition::ptr position);

    /**
     * @brief Returns the mid point between this position and another
     * @param position
     * @return midpoint
     */
    SlonlatPosition midpoint(const SlonlatPosition& position);
};

#endif // BENCHMARK_SXYPOSITION_H
