//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   11.05.2017
//------------------------------------------------------------------------------

#include <fstream>
#include <sstream>
#include <cmath>

#include "utils.h"

//------------------------------------------------------------------------------
// Calculate error
//------------------------------------------------------------------------------
std::array<double, 3> CalculateError(double gt_x, double gt_y, double gt_theta,
                                     double pf_x, double pf_y, double pf_theta)
{
  std::array<double, 3> error;
  error[0] = fabs(pf_x - gt_x);
  error[1] = fabs(pf_y - gt_y);
  error[2] = fabs(pf_theta - gt_theta);
  error[2] = fmod(error[2], 2.0 * M_PI);
  return error;
}

//------------------------------------------------------------------------------
// Read map data
//------------------------------------------------------------------------------
void ReadMapData( Map& map, const std::string &filename) {
  std::ifstream in_file(filename.c_str());
  if(!in_file.is_open())
    throw std::runtime_error("Cannot open input file: " + filename);

  std::string line;
  while(getline(in_file, line)) {
    std::istringstream iss(line);
    double landmark_x, landmark_y;
    int    id;
    iss >> landmark_x >> landmark_y >> id;
    map.landmarks.push_back({id, landmark_x, landmark_y});
  }

  in_file.close();
}

//------------------------------------------------------------------------------
// Read control data
//------------------------------------------------------------------------------
void ReadControlData(std::vector<Control> &position_meas,
                     const std::string    &filename) {
  std::ifstream in_file(filename.c_str());
  if(!in_file.is_open())
    throw std::runtime_error("Cannot open input file: " + filename);

  std::string line;
  while(getline(in_file, line)) {
    std::istringstream iss(line);
    double velocity, yawrate;
    iss >> velocity >> yawrate;
    position_meas.push_back({velocity, yawrate});
  }

  in_file.close();
}

//------------------------------------------------------------------------------
// Read ground truth data
//------------------------------------------------------------------------------
void ReadGTData(std::vector<GroundTruth> &gt,
                const std::string        &filename) {
  std::ifstream in_file(filename.c_str());
  if(!in_file.is_open())
    throw std::runtime_error("Cannot open iniput file: " + filename);

  std::string line;
  while(getline(in_file, line)) {
    std::istringstream iss(line);
    double x, y, azimuth;
    iss >> x >> y >> azimuth;
    gt.push_back({x, y, azimuth});
  }

  in_file.close();
}

//------------------------------------------------------------------------------
// Read landmark data
//------------------------------------------------------------------------------
void ReadObservationData(std::vector<Observation> &observations,
                         const std::string        &filename) {
  std::ifstream in_file(filename.c_str());
  if(!in_file.is_open())
    throw std::runtime_error("Cannot open input file: " + filename);

  std::string line;
  while(getline(in_file, line)) {
    std::istringstream iss(line);
    double local_x, local_y;
    iss >> local_x >> local_y;
    observations.push_back({local_x, local_y});
  }

  in_file.close();
}

//------------------------------------------------------------------------------
// Write particles
//------------------------------------------------------------------------------
void WriteParticleData(const std::string           &filename,
                       const std::vector<Particle> &particles) {
  std::ofstream out_file(filename.c_str());
  if(!out_file.is_open())
    throw std::runtime_error("Cannot open output file: " + filename);

  for(const auto &p: particles) {
    out_file << p.x() << " " << p.y() << " " << p.theta() << " " << p.weight();
    out_file << std::endl;
  }
  out_file.close();
}
