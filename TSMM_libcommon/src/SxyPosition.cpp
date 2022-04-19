//
// Created by xiaorui on 21.02.22.
//

#include "SxyPosition.h"

SlonlatPosition::SlonlatPosition() {
    this->lat = 0;
    this->lon = 0;
}

SlonlatPosition::SlonlatPosition(double lon, double lat) {
    this->lon = lon;
    this->lat = lat;
}

SlonlatPosition::SlonlatPosition(SlonlatPosition& pos) {
    this->lon = pos.lon;
    this->lat = pos.lat;
}

void SlonlatPosition::operator=(SlonlatPosition& pos) {
    this->lat = pos.lat;
    this->lon = pos.lon;
}

double SlonlatPosition::distance(SlonlatPosition::ptr position) {
    double earthRadius = 6371000; // in meters
    double dLat        = SMath::toRadians(this->lat - position->lat);
    double dLng        = SMath::toRadians(this->lon - position->lon);
    double a           = SMath::sin(dLat / 2) * SMath::sin(dLat / 2) +
               SMath::cos(SMath::toRadians(position->lat)) * SMath::cos(SMath::toRadians(this->lat)) * SMath::sin(dLng / 2) * SMath::sin(dLng / 2);
    double c    = 2 * SMath::atan2(SMath::sqrt(a), SMath::sqrt(1 - a));
    double dist = earthRadius * c;

    return dist;
}

SlonlatPosition SlonlatPosition::midpoint(const SlonlatPosition& position) {
    SlonlatPosition midpoint(0.5 * (this->lon + position.lon), 0.5 * (this->lat + position.lat));
    return midpoint;
}
