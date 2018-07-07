/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
  //   x, y, theta and their uncertainties from GPS) and all weights to 1.
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  num_particles = 20;

  std::default_random_engine dre;

  std::normal_distribution<double> x_dist(x, std[0]);
  std::normal_distribution<double> y_dist(y, std[1]);
  std::normal_distribution<double> theta_dist(theta, std[2]);

  particles.resize(num_particles);
  weights.resize(num_particles);

  for (int i = 0; i < num_particles; ++i)
  {
    double x_p = x_dist(dre);
    double y_p = y_dist(dre);
    double theta_p = theta_dist(dre);

    particles[i].id = i;
    particles[i].x = x_p;
    particles[i].y = y_p;
    particles[i].theta = theta_p;
    particles[i].weight = 1.;

    weights[i] = 1.;
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  default_random_engine dre;

  normal_distribution<double> x_noise_dist(0, std_pos[0]);
  normal_distribution<double> y_noise_dist(0, std_pos[1]);
  normal_distribution<double> theta_noise_dist(0, std_pos[2]);

  for (int i = 0; i < particles.size(); ++i)
  {
    Particle p = particles[i];

    double x_pred, y_pred, theta_pred;

    if (fabs(yaw_rate) > 0.001)
    {

      x_pred     = p.x + (velocity / yaw_rate) * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
      y_pred     = p.y + (velocity / yaw_rate) * (-cos(p.theta + yaw_rate * delta_t) + cos(p.theta));
      theta_pred = p.theta + yaw_rate * delta_t;
    }
    else
    {
      x_pred     = p.x + velocity * cos(p.theta) * delta_t;
      y_pred     = p.y + velocity * sin(p.theta) * delta_t;
      theta_pred = p.theta + yaw_rate * delta_t;
    }

    double x_noise = x_noise_dist(dre);
    double y_noise = y_noise_dist(dre);
    double theta_noise = theta_noise_dist(dre);

    x_pred += x_noise;
    y_pred += y_noise;
    theta_pred += theta_noise;

    Particle p_pred;
    p_pred.x = x_pred;
    p_pred.y = y_pred;
    p_pred.theta = theta_pred;

    particles[i] = p_pred;
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
  //   implement this method and use it as a helper during the updateWeights phase.

  for (int i = 0; i < observations.size(); ++i) {

    double min_dist = 1000000.0;

    for (int j = 0; j < predicted.size(); ++j)
    {
      double dist_ij = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

      if (dist_ij <= min_dist)
      {
        min_dist = dist_ij;
        observations[i].id = predicted[j].id;
      }
    }
  }
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   std::vector<LandmarkObs> observations, Map map_landmarks) {
  // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
  //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
  //   according to the MAP'S coordinate system. You will need to transform between the two systems.
  //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
  //   The following is a good resource for the theory:
  //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //   and the following is a good resource for the actual equation to implement (look at equation
  //   3.33
  //   http://planning.cs.uiuc.edu/node99.html

  for (int p = 0; p < particles.size(); ++p)
  {
    // transform each observation by rotating by rot_angle and translating by tr_x and tr_y
    double rot_angle = particles[p].theta;
    double tr_x = particles[p].x;
    double tr_y = particles[p].y;

    std::vector<LandmarkObs> transformed_observations;
    for (int o = 0; o < observations.size(); ++o)
    {
      LandmarkObs tr_observation;

      double obs_x = observations[o].x;
      double obs_y = observations[o].y;

      tr_observation.x = obs_x * cos(rot_angle) - obs_y * sin(rot_angle) + tr_x;
      tr_observation.y = obs_x * sin(rot_angle) + obs_y * cos(rot_angle) + tr_y;

      transformed_observations.push_back(tr_observation);
    }


    std::vector<LandmarkObs> close_landmarks;
    for (int l = 0; l < map_landmarks.landmark_list.size(); ++l)
    {
      // take sensor range into consideration
      if (dist(particles[p].x, particles[p].y, map_landmarks.landmark_list[l].x_f, map_landmarks.landmark_list[l].y_f) < sensor_range)
      {
        LandmarkObs close_obs;
        close_obs.x = map_landmarks.landmark_list[l].x_f;
        close_obs.y = map_landmarks.landmark_list[l].y_f;
        close_obs.id = map_landmarks.landmark_list[l].id_i;

        close_landmarks.push_back(close_obs);
      }
    }

    dataAssociation(close_landmarks, transformed_observations);

    double weight = 1.;
    for (int l = 0; l < close_landmarks.size(); ++l) {


      for (int o = 0; o < transformed_observations.size(); ++o) {
        if (close_landmarks[l].id == transformed_observations[o].id)
        {
          double x = transformed_observations[o].x;
          double y = transformed_observations[o].y;

          double mu_x = close_landmarks[l].x;
          double mu_y = close_landmarks[l].y;

          double sig_x = std_landmark[0];
          double sig_y = std_landmark[1];

          // precalculations for exponent
          double xx = (x - mu_x) * (x - mu_x) / (2 * sig_x * sig_x);
          double yy = (y - mu_y) * (y - mu_y) / (2 * sig_y * sig_y);

          weight *= 0.5 / (M_PI * sig_x * sig_y) * exp(-(xx + yy));
        }
        weights[p] = weight;
      }
    }
  }
}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  std::default_random_engine dre;
  std::discrete_distribution<> particles_dist(weights.begin(), weights.end());

  vector<Particle> updated_particles(num_particles);

  for (int i = 0; i < num_particles; ++i)
  {
    updated_particles[i] = particles[particles_dist(dre)];
  }

  particles = updated_particles;

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
  //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  //Clear the previous associations
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;

  return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseX(Particle best)
{
  vector<double> v = best.sense_x;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseY(Particle best)
{
  vector<double> v = best.sense_y;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
