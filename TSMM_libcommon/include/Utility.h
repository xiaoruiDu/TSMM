//
// Created by xiaorui on 22.02.22.
//

#ifndef BENCHMARK_UTILITY_H
#define BENCHMARK_UTILITY_H

#define PI 3.14159265358979323846
#define EARTH_RADIUS_IN_METERS 6372797.56085

#include "SxyPosition.h"

#include <map>
#include <math.h>
template <typename KeyType, typename ValueType>
std::pair<KeyType, ValueType> get_max(const std::map<KeyType, ValueType>& x) {
    using pairtype = std::pair<KeyType, ValueType>;
    return *std::max_element(x.begin(), x.end(), [](const pairtype& p1, const pairtype& p2) { return p1.second < p2.second; });
}

inline double deg_to_rad(double degree) noexcept {
    return degree * (PI / 180.0);
}

inline double distance(double lat1, double lon1, double lat2, double lon2) {
    double lonh = std::sin(deg_to_rad(lon1 - lon2) * 0.5);
    lonh *= lonh;
    double lath = std::sin(deg_to_rad(lat1 - lat2) * 0.5);
    lath *= lath;
    const double tmp = std::cos(deg_to_rad(lat1)) * std::cos(deg_to_rad(lat2));
    return 2.0 * EARTH_RADIUS_IN_METERS * std::asin(std::sqrt(lath + tmp * lonh));
}

inline double distanceForAngle(SlonlatPosition::ptr start, SlonlatPosition::ptr end) {
    // Calculating distance
    return sqrt(pow(end->lat - start->lat, 2) + pow(end->lon - start->lon, 2));
}

inline double calAngleOfTwoVectors(SlonlatPosition::ptr start1,
                                   SlonlatPosition::ptr end1,

                                   SlonlatPosition::ptr start2,
                                   SlonlatPosition::ptr end2) {
    double vector1X = end1->lat - start1->lat;
    double vector1Y = end1->lon - start1->lon;

    double vector2X = end2->lat - start2->lat;
    double vector2Y = end2->lon - start2->lon;

    double dotResult = vector1X * vector2X + vector1Y * vector2Y;

    double value = (distanceForAngle(start1, end1) * distanceForAngle(start2, end2));

    assert(value != 0);

    double cosTheta = dotResult / value;
    double angle    = acos(cosTheta) * 180 / M_PI;
    //    std::cout << angle << '\n';
    return angle;
}
inline SlonlatPosition::ptr extendWithDistanceToNewPosition(const SlonlatPosition::ptr& from,
                                                            const SlonlatPosition::ptr& to,
                                                            double extendedDistance) {
    double dis   = distance(from->lat, from->lon, to->lat, to->lon);
    double ratio = extendedDistance / (extendedDistance + dis);
    double lon   = (to->lon - ratio * from->lon) / (1 - ratio);
    double lat   = (to->lat - ratio * from->lat) / (1 - ratio);
    return std::make_shared<SlonlatPosition>(lon, lat);
}
#endif // BENCHMARK_UTILITY_H
