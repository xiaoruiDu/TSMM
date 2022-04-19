//
// Created by xiaorui on 21.02.22.
//

#ifndef BENCHMARK_SMATH_H
#define BENCHMARK_SMATH_H

#include "OSMCoreTypes.h"
#include "SxyPosition.h"

#include <algorithm>
#include <cmath>

class SlonlatPosition;

/**
 * @brief Math is a convenience wrapper class for various math functions and is based on
 * the java.lang.Math class. It is used to make translation of code from the Java
 * prototype of the simulation core easier.
 *
 * @author heiko.aydt
 */
class SMath {
public:
    static double PI;
    static double EARTH_RADIUS;
    static double EARTH_CIRCUMFERENCE;

    static double toDegrees(double rad) {
        return rad * 180.0 / PI;
    }

    static double toRadians(double deg) {
        return deg * PI / 180.0;
    }

    static double sin(double x) {
        return std::sin(x);
    }

    static double cos(double x) {
        return std::cos(x);
    }

    static double asin(double x) {
        return std::asin(x);
    }

    static double acos(double x) {
        return std::acos(x);
    }

    static double atan(double x) {
        return std::atan(x);
    }

    static double sqrt(double x) {
        return std::sqrt(x);
    }

    static double atan2(double y, double x) {
        return std::atan2(y, x);
    }

    static double hypott(double dx, double dy) {
        return hypot(dx, dy);
    }

    /**
     * @brief conversion of distance in meter to distance in degrees
     *
     * @param dist distance in meters
     * @return double distance in degrees on the globe.
     */
    static double degrees(double dist) {
        double a = 360.0 / EARTH_CIRCUMFERENCE; // unit: [degrees/meter]

        double degrees = a * dist;

        return degrees;
    }

    static double max(double x, double y) {
        return std::max(x, y);
    }

    static int max(int x, int y) {
        return std::max(x, y);
    }

    static double min(double x, double y) {
        return std::min(x, y);
    }

    static int min(int x, int y) {
        return std::min(x, y);
    }

    static int sign(double x) {
        if (x < 0) return -1;
        if (x > 0) return 1;
        return 0;
    }

    static int sign(int x) {
        if (x < 0) return -1;
        if (x > 0) return 1;
        return 0;
    }
};

#endif // BENCHMARK_SMATH_H
