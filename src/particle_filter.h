//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   11.05.2017
//------------------------------------------------------------------------------

#pragma once

#include <array>
#include <random>

#include "map.h"

//------------------------------------------------------------------------------
//! Landmark observation (in vehicle coordinates)
//------------------------------------------------------------------------------
struct Observation {
  double x;  // Local x position of landmark observation [m]
  double y;  // Local y position of landmark observation [m]
};


//------------------------------------------------------------------------------
//! A particle
//------------------------------------------------------------------------------
class Particle {
  public:
    //--------------------------------------------------------------------------
    // Constructors and destructor
    //--------------------------------------------------------------------------
    Particle() {}
    Particle(double x, double y, double theta):
      x_(x), y_(y), theta_(theta) {}
    virtual ~Particle() {}

    //--------------------------------------------------------------------------
    // Accessors
    //--------------------------------------------------------------------------
    double x() const { return x_; }
    double y() const { return y_; }
    double theta() const { return theta_; }
    double weight() const { return weight_; }

    //--------------------------------------------------------------------------
    //! Move the particle
    //--------------------------------------------------------------------------
    void Move(double                       delta_t,
              double                       velocity,
              double                       yaw_rate,
              const std::array<double, 3> &pos_noise);

    //--------------------------------------------------------------------------
    //! Process observations
    //--------------------------------------------------------------------------
    void Observe(double                          sensor_range,
                 const std::array<double, 2>    &std_landmark,
                 const std::vector<Observation> &observations,
                 const Map                      &map);

  private:
    double x_      = 0;
    double y_      = 0;
    double theta_  = 0;
    double weight_ = 0;
};

//------------------------------------------------------------------------------
//! A particle filter
//------------------------------------------------------------------------------
class ParticleFilter {
  public:

    //--------------------------------------------------------------------------
    //! Constructor
    //--------------------------------------------------------------------------
    ParticleFilter(int num_particles): num_particles_(num_particles) {}

    //--------------------------------------------------------------------------
    //! Descturctor
    //--------------------------------------------------------------------------
    virtual ~ParticleFilter() {}

    //--------------------------------------------------------------------------
    //! Initializer
    //--------------------------------------------------------------------------
    void Init(double                       x,
              double                       y,
              double                       theta,
              const std::array<double, 3> &std);


    //--------------------------------------------------------------------------
    //! Move the particles
    //--------------------------------------------------------------------------
    void Move(double                       delta_t,
              double                       velocity,
              double                       yaw_rate,
              const std::array<double, 3> &std_pos);

    //--------------------------------------------------------------------------
    //! Process observations
    //--------------------------------------------------------------------------
    void Observe(double                          sensor_range,
                 const std::array<double, 2>    &std_landmark,
                 const std::vector<Observation> &observations,
                 const Map                      &map);

    //--------------------------------------------------------------------------
    //! Check if the filter is initialized
    //--------------------------------------------------------------------------
    bool IsInitialized() const {
      return is_initialized_;
    }

    //--------------------------------------------------------------------------
    //! Get best particle
    //--------------------------------------------------------------------------
    Particle GetBestParticle() const;
  private:
    int                        num_particles_;
    bool                       is_initialized_ = false;
    std::vector<Particle>      particles_;
    std::vector<Particle>      new_particles_;
    std::vector<double>        weights_;
    std::default_random_engine gen_;
};
