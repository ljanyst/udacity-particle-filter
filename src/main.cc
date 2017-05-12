//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   11.05.2017
//------------------------------------------------------------------------------

#include <iostream>
#include <ctime>
#include <iomanip>
#include <random>
#include <sstream>
#include <array>
#include <fstream>

#include "particle_filter.h"
#include "utils.h"

using namespace std;

//------------------------------------------------------------------------------
// Start the show
//------------------------------------------------------------------------------
int main(int argc, char **argv) {
  //----------------------------------------------------------------------------
  // Check the commandline
  //----------------------------------------------------------------------------
  if(argc != 4) {
    cerr << "Usage: " << argv[0] << " data_dir n_particles output" << endl;
    return 1;
  }
  int num_particles = 0;
  try {
    num_particles = std::stoi(argv[2]);
  }
  catch(std::invalid_argument &e) {
    cerr << "n_particles needs to be an integer" << endl;
    return 1;
  }

  //----------------------------------------------------------------------------
  // Constants
  //----------------------------------------------------------------------------
  const double kDeltaT = 0.1; // Time elapsed between measurements [sec]
  const double kSensorRange = 50;  // Sensor range [m]
  const std::array<double, 3> kSigmaPos = { // GPS measurement uncertainty
    0.3, // x [m]
    0.3, // y [m]
    0.01 // theta [rad]
  };
  const std::array<double, 2> kSigmaLandmark = { // Landmark measurement uncertainty
    0.3, // x [m]
    0.3  // y [m]
  };

  //----------------------------------------------------------------------------
  // Random number generators
  //----------------------------------------------------------------------------
  default_random_engine       gen;
  normal_distribution<double> rnd_x_init(0,     kSigmaPos[0]);
  normal_distribution<double> rnd_y_init(0,     kSigmaPos[1]);
  normal_distribution<double> rnd_theta_init(0, kSigmaPos[2]);
  normal_distribution<double> rnd_obs_x(0, kSigmaLandmark[0]);
  normal_distribution<double> rnd_obs_y(0, kSigmaLandmark[1]);

  //----------------------------------------------------------------------------
  // Read the input data
  //----------------------------------------------------------------------------
  Map                 map;
  vector<Control>     position_meas;
  vector<GroundTruth> gt;
  try {
    string data_dir = argv[1];
    ReadMapData(map, data_dir + "/map_data.txt");
    ReadControlData(position_meas, data_dir + "/control_data.txt");
    ReadGTData(gt, data_dir + "/gt_data.txt");
  }
  catch(runtime_error &e) {
    cerr << "Unable to read data: " << e.what() << endl;
    return 1;
  }

  //----------------------------------------------------------------------------
  // Loop over all of the measurements
  //----------------------------------------------------------------------------
  ParticleFilter        pf(num_particles);
  int                   start             = clock();
  int                   n_steps           = position_meas.size();
  double                total_error[3]    = {0,0,0};
  double                cum_mean_error[3] = {0,0,0};
  std::vector<Particle> best_particles;

  for(int i = 0; i < n_steps; ++i) {
    cout << "Time step: " << i << endl;

    //--------------------------------------------------------------------------
    // Read the observations
    //--------------------------------------------------------------------------
    ostringstream file;
    file << argv[1] << "/observation/observations_" << setfill('0');
    file << setw(6) << i+1 << ".txt";
    vector<Observation> observations;
    try {
      ReadObservationData(observations, file.str());
    }
    catch(runtime_error &e) {
      cerr << "Unable to read data: " << e.what() << endl;
      return 1;
    }

    //--------------------------------------------------------------------------
    // Initialize the filter
    //--------------------------------------------------------------------------
    if(!pf.IsInitialized()) {
      double noise_x     = rnd_x_init(gen);
      double noise_y     = rnd_y_init(gen);
      double noise_theta = rnd_theta_init(gen);
      pf.Init(gt[i].x + noise_x, gt[i].y + noise_y, gt[i].theta + noise_theta,
              kSigmaPos);
    }

    //--------------------------------------------------------------------------
    // Move the vehicle (predict the position)
    //--------------------------------------------------------------------------
    else
      pf.Move(kDeltaT, position_meas[i-1].velocity, position_meas[i-1].yawrate,
              kSigmaPos);

    //--------------------------------------------------------------------------
    // Simulate addition of noise to noiseless observation data
    //--------------------------------------------------------------------------
    vector<Observation> noisy_observations;
    for (int j = 0; j < observations.size(); ++j) {
      double noise_x = rnd_obs_x(gen);
      double noise_y = rnd_obs_y(gen);
      auto   obs     = observations[j];
      obs.x += noise_x;
      obs.y += noise_y;
      noisy_observations.push_back(obs);
    }

    pf.Observe(kSensorRange, kSigmaLandmark, noisy_observations, map);

    //--------------------------------------------------------------------------
    // Calculate the error metric
    //--------------------------------------------------------------------------
    auto bp      = pf.GetBestParticle();
    auto avg_err = CalculateError(gt[i].x, gt[i].y, gt[i].theta,
                                  bp.x(),  bp.y(),  bp.theta());

    for(int j = 0; j < 3; ++j) {
      total_error[j] += avg_err[j];
      cum_mean_error[j] = total_error[j] / (double)(i + 1);
    }

    cout << "Cumulative mean weighted error: x = " << cum_mean_error[0];
    cout << " y = " << cum_mean_error[1] << " yaw = " << cum_mean_error[2];
    cout << endl;
    best_particles.push_back(bp);
  }

  //----------------------------------------------------------------------------
  // Finalize
  //----------------------------------------------------------------------------
  double runtime = (clock() - start) / double(CLOCKS_PER_SEC);
  cout << "Runtime (sec): " << runtime << endl;
  try {
    WriteParticleData(argv[3], best_particles);
  }
  catch(runtime_error &e) {
    cerr << "Unable to write data: " << e.what() << endl;
    return 1;
  }

  return 0;
}
