//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   11.05.2017
//------------------------------------------------------------------------------

#pragma once

#include <array>
#include <cmath>

#include "map.h"
#include "particle_filter.h"

//------------------------------------------------------------------------------
//! Control metrics
//------------------------------------------------------------------------------
struct Control {
  double velocity; // Velocity [m/s]
  double yawrate;  // Yaw rate [rad/s]
};

//------------------------------------------------------------------------------
//! Ground truth
//------------------------------------------------------------------------------
struct GroundTruth {
  double x;     // Global vehicle x position [m]
  double y;     // Global vehicle y position [m]
  double theta; // Global vehicle yaw [rad]
};

//------------------------------------------------------------------------------
//! Calculate distance between points
//------------------------------------------------------------------------------
inline double CalculateDistance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

//------------------------------------------------------------------------------
//! Calculate error
//------------------------------------------------------------------------------
std::array<double, 3> CalculateError(double gt_x, double gt_y, double gt_theta,
                                     double pf_x, double pf_y, double pf_theta);

//------------------------------------------------------------------------------
//! Read map data
//------------------------------------------------------------------------------
void ReadMapData(Map &map, const std::string &filename);

//------------------------------------------------------------------------------
//! Read control data
//------------------------------------------------------------------------------
void ReadControlData(std::vector<Control> &position_meas,
                     const std::string    &filename);

//------------------------------------------------------------------------------
//! Read ground truth data
//------------------------------------------------------------------------------
void ReadGTData(std::vector<GroundTruth> &gt,
                const std::string        &filename);

//------------------------------------------------------------------------------
//! Read landmark data
//------------------------------------------------------------------------------
void ReadObservationData(std::vector<Observation> &observations,
                         const std::string        &filename);

//------------------------------------------------------------------------------
//! Write particles
//------------------------------------------------------------------------------
void WriteParticleData(const std::string           &filename,
                       const std::vector<Particle> &particles);
