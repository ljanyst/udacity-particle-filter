//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   11.05.2017
//------------------------------------------------------------------------------

#include <random>
#include <fstream>
#include <cmath>
#include <vector>

#include "particle_filter.h"
#include "utils.h"

//------------------------------------------------------------------------------
// Make sure that the angle value is between [0, 2*pi]
//------------------------------------------------------------------------------
static inline double NormalizeAngle(double angle) {
  angle = fmod(angle, 2*M_PI);
  if(angle < 0)
    angle += 2*M_PI;
  return angle;
}

//------------------------------------------------------------------------------
// Return value of 2D gaussian distribution
//------------------------------------------------------------------------------
static inline double Gaussian2D(double x, double y, double mux, double muy,
                                double stdx, double stdy) {

  double norm = 1./(2*M_PI*stdx*stdy);
  double exp1 = pow(x-mux, 2)/(2*stdx*stdx);
  double exp2 = pow(y-muy, 2)/(2*stdy*stdy);
  return norm * exp(-(exp1+exp2));
}

//------------------------------------------------------------------------------
// Move the particle
//------------------------------------------------------------------------------
void Particle::Move(double                       delta_t,
                    double                       velocity,
                    double                       yaw_rate,
                    const std::array<double, 3> &pos_noise) {
  if(fabs(yaw_rate) < 10e-3) {
    x_ += velocity * delta_t * cos(theta_);
    y_ += velocity * delta_t * sin(theta_);
  }
  else {
    double delta_theta = yaw_rate * delta_t;
    x_     += velocity/yaw_rate * (sin(theta_ + delta_theta) - sin(theta_));
    y_     += velocity/yaw_rate * (cos(theta_) - cos(theta_ + delta_theta));
    theta_ += delta_theta;
  }
  x_     += pos_noise[0];
  y_     += pos_noise[1];
  theta_ += pos_noise[2];
  theta_ = NormalizeAngle(theta_);
}

//------------------------------------------------------------------------------
// Process observations
//------------------------------------------------------------------------------
void Particle::Observe(double                          sensor_range,
                       const std::array<double, 2>    &std_landmark,
                       const std::vector<Observation> &observations,
                       const Map                      &map) {
  //----------------------------------------------------------------------------
  // Find landmarks in range
  //----------------------------------------------------------------------------
  std::vector<Map::Landmark> landmarks_in_range;
  landmarks_in_range.reserve(map.landmarks.size());
  for(const auto &l: map.landmarks) {
    if(CalculateDistance(x_, y_, l.x, l.y) <= sensor_range)
      landmarks_in_range.push_back(l);
  }

  //----------------------------------------------------------------------------
  // Convert observations to the map coordinate system
  //----------------------------------------------------------------------------
  weight_ = 1;

  for(const auto &o: observations) {
    double distance    = CalculateDistance(0, 0, o.x, o.y);
    double angle       = NormalizeAngle(atan2(o.y, o.x) + theta_);
    double global_x    = x_ + distance * cos(angle);
    double global_y    = y_ + distance * sin(angle);
    double lm_x        = x_ + sensor_range * cos(angle);
    double lm_y        = y_ + sensor_range * sin(angle);
    double lm_distance = sensor_range - distance;

    for(const auto &l: landmarks_in_range) {
      double distance = CalculateDistance(l.x, l.y, global_x, global_y);
      if(distance < lm_distance) {
        lm_distance = distance;
        lm_x        = l.x;
        lm_y        = l.y;
      }
    }
    weight_ *= Gaussian2D(global_x, global_y, lm_x, lm_y,
                          std_landmark[0], std_landmark[1]);
  }
}

//------------------------------------------------------------------------------
// Initializer
//------------------------------------------------------------------------------
void ParticleFilter::Init(double                       x,
                          double                       y,
                          double                       theta,
                          const std::array<double, 3> &std) {
  particles_.reserve(num_particles_);
  new_particles_.reserve(num_particles_);
  weights_.reserve(num_particles_);

  std::normal_distribution<double> rnd_x(0,     std[0]);
  std::normal_distribution<double> rnd_y(0,     std[1]);
  std::normal_distribution<double> rnd_theta(0, std[2]);

  for(int i = 0; i < num_particles_; ++i) {
    particles_.emplace_back(x + rnd_x(gen_), y + rnd_y(gen_),
                            NormalizeAngle(theta + rnd_theta(gen_)));
  }
  is_initialized_ = true;
}


//------------------------------------------------------------------------------
// Move the particles
//------------------------------------------------------------------------------
void ParticleFilter::Move(double                       delta_t,
                          double                       velocity,
                          double                       yaw_rate,
                          const std::array<double, 3> &std_pos) {
  std::normal_distribution<double> rnd_x(0,     std_pos[0]);
  std::normal_distribution<double> rnd_y(0,     std_pos[1]);
  std::normal_distribution<double> rnd_theta(0, std_pos[2]);
  std::array<double, 3>            noise;

  for(auto &p: particles_) {
    noise[0] = rnd_x(gen_);
    noise[1] = rnd_y(gen_);
    noise[2] = rnd_theta(gen_);
    p.Move(delta_t, velocity, yaw_rate, noise);
  }
}

//------------------------------------------------------------------------------
// Process observations
//------------------------------------------------------------------------------
void ParticleFilter::Observe(double                          sensor_range,
                             const std::array<double, 2>    &std_landmark,
                             const std::vector<Observation> &observations,
                             const Map                      &map) {
  //----------------------------------------------------------------------------
  // Update particle weights
  //----------------------------------------------------------------------------
  for(auto &p: particles_)
    p.Observe(sensor_range, std_landmark, observations, map);

  //----------------------------------------------------------------------------
  // Resample particles
  //----------------------------------------------------------------------------
  weights_.clear();
  for(auto &p: particles_)
    weights_.push_back(p.weight());

  std::discrete_distribution<int> sample(weights_.begin(), weights_.end());
  new_particles_.clear();
  for(int i = 0; i < particles_.size(); ++i)
    new_particles_.push_back(particles_[sample(gen_)]);

  std::swap(particles_, new_particles_);
}

//------------------------------------------------------------------------------
// Get best particle
//------------------------------------------------------------------------------
Particle ParticleFilter::GetBestParticle() const {
  double   highest_weight = -1.0;
  Particle best_particle;
  for(const auto &p: particles_) {
    if(p.weight() > highest_weight) {
      highest_weight = p.weight();
      best_particle  = p;
    }
  }
  return best_particle;
}
