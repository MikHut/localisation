/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey et al.
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
// Desc: LASER sensor model for AMCL
// Author: Andrew Howard
// Date: 17 Aug 2003
// CVS: $Id: amcl_laser.h 6443 2008-05-15 19:46:11Z gerkey $
//
///////////////////////////////////////////////////////////////////////////

#ifndef AMCL_FEATURE_H
#define AMCL_FEATURE_H

#include "mel_amcl_sensor.h"
#include "../map/map.h"

namespace mel_amcl
{



// Laser sensor data
class AMCLFeatureData : public AMCLSensorData
{
  public:
    AMCLFeatureData () {poses=NULL;};
    virtual ~AMCLFeatureData() {delete [] poses;};
  // Laser range data (range, bearing tuples)
  public: int feature_count;
  public: double (*poses)[2];
};


// Laseretric sensor model
class AMCLFeature : public AMCLSensor
{
  // Default constructor
  public: AMCLFeature(map_t* map);

  public: virtual ~AMCLFeature();

  public: void SetModelLikelihoodField(double z_hit,
                                       double z_rand,
                                       double sigma_hit,
                                       double max_occ_dist);



  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  public: virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data);



  // Determine the probability for the given pose
  private: static double LikelihoodFieldModel(AMCLFeatureData *data, 
                                              pf_sample_set_t* set);


  // Current data timestamp
  private: double time;

  // The feature map
  private: map_t *map;

  // Laser offset relative to robot
  private: pf_vector_t laser_pose;


  // Beam skipping parameters (used by LikelihoodFieldModelProb model)
  private: bool do_beamskip;
  private: double beam_skip_distance;
  private: double beam_skip_threshold;
  //threshold for the ratio of invalid beams - at which all beams are integrated to the likelihoods 
  //this would be an error condition
  private: double beam_skip_error_threshold;

  //temp data that is kept before observations are integrated to each particle (requried for beam skipping)
  private: int max_samples;
  private: int max_obs;
  private: double **temp_obs;

  // Laser model params
  //
  // Mixture params for the components of the model; must sum to 1
  private: double z_hit;
  private: double z_short;
  private: double z_max;
  private: double z_rand;
  //
  // Stddev of Gaussian model for laser hits.
  private: double sigma_hit;
  // Decay rate of exponential model for short readings.
  private: double lambda_short;
  // Threshold for outlier rejection (unused)
  private: double chi_outlier;
};


}

#endif
