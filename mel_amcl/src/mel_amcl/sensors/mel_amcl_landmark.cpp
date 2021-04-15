/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL laser routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_laser.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif

#include "mel_amcl/sensors/mel_amcl_landmark.h"

// roscpp
#include "ros/ros.h"

using namespace mel_amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLLandmark::AMCLLandmark(map_t* map) : AMCLSensor(),
						     max_samples(0), max_obs(0),
						     temp_obs(NULL)
{
  this->time = 0.0;

  this->map = map;

  return;
}

AMCLLandmark::~AMCLLandmark()
{
  if(temp_obs){
	for(int k=0; k < max_samples; k++){
	  delete [] temp_obs[k];
	}
	delete []temp_obs;
  }
}

void
AMCLLandmark::SetModelLikelihoodField(double z_hit,
                                   double z_rand,
                                   double sigma_hit,
                                   double max_occ_dist)
{
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;

  // map_update_cspace(this->map, max_occ_dist);
}


////////////////////////////////////////////////////////////////////////////////
// Apply the sensor model
bool AMCLLandmark::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{

  pf_update_sensor(pf, (pf_sensor_model_fn_t) LikelihoodFieldModel, data);

  return true;
}



double AMCLLandmark::LikelihoodFieldModel(AMCLLandmarkData *data, pf_sample_set_t* set)
{
  AMCLLandmark *self;
  int i, j;
  double z, pz;
  double p;
  double x_landmark, y_landmark;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;


  self = (AMCLLandmark*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    p = 1.0;

    // Pre-compute a couple of things
    double z_rand_mult = 1.0/30;


    for (i = 0; i < data->landmark_count; i++)
    {
      x_landmark = data->poses[i][0]; // this is in base_link - need to transform to map
      y_landmark = data->poses[i][1];

      // This model ignores max range readings
      if(x_landmark >= 30)
        continue;

      // Check for NaN
      if(x_landmark != x_landmark)
        continue;

      pz = 0.0;

      // Transform pose to map here - NOT DONE YET!!! Assume landmarks are in base_link for now
      // maybe do transformation in main node to use ROS tf and keep ROS stuff together
      // NO - need to do it here - as trans is different for each particle - take care of angle!
      hit.v[0] = pose.v[0] + (x_landmark * cos(pose.v[2])) - (y_landmark * sin(pose.v[2]));
      hit.v[1] = pose.v[1] + (x_landmark * sin(pose.v[2])) + (y_landmark * cos(pose.v[2]));
      // ROS_INFO("getting map coords at x=%f, y=%f",  hit.v[0] ,  hit.v[1]);
      // ROS_INFO("Map info origin=%f",  self->map->origin_x);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      // ROS_INFO("got map coords");

      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if(!MAP_VALID(self->map, mi, mj)){
        z = self->map->max_occ_dist;}
        // ROS_INFO("MAP NOT VALID at %d %d", mi, mj);}
      else
        z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist / 100;
        // ROS_INFO("Prob %f x: %f, y: %f lx: %f, ly: %f", z, hit.v[0], hit.v[1], x_landmark, y_landmark);

      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)

      // pz += self->z_hit * z;
      pz += z;
      // replaced with above since our published likelihood field does this
      // keep the z_hit bit for now till I know for sure what that models - think its just prob of hit so nearly 1 usually - Michael Apr 2021
      // pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      // pz += self->z_rand * z_rand_mult;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);

      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      // p += pz*pz*pz;
      p += pz;
    }
    p/=data->landmark_count;
    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}
