/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "pilz_industrial_motion_planner/cartesian_limit.h"

pilz_industrial_motion_planner::CartesianLimit::CartesianLimit()
  : has_max_trans_vel_(false)
  , max_trans_vel_(0.0)
  , has_max_trans_acc_(false)
  , max_trans_acc_(0.0)
  , has_max_trans_dec_(false)
  , max_trans_dec_(0.0)
  , has_max_rot_vel_(false)
  , max_rot_vel_(0.0)
  , has_max_jerk_(false)
  ,max_jerk_(0.0)
  ,velocity_scaling_factor(0.3)
  ,acceleration_scaling_factor(0.2)
{
}

// Translational Velocity Limit

bool pilz_industrial_motion_planner::CartesianLimit::hasMaxTranslationalVelocity() const
{
  return has_max_trans_vel_;
}

void pilz_industrial_motion_planner::CartesianLimit::setMaxTranslationalVelocity(double max_trans_vel)
{
  has_max_trans_vel_ = true;
  max_trans_vel_ = max_trans_vel;
}

double pilz_industrial_motion_planner::CartesianLimit::getMaxTranslationalVelocity() const
{
  return max_trans_vel_;
}

// Translational Acceleration Limit

bool pilz_industrial_motion_planner::CartesianLimit::hasMaxTranslationalAcceleration() const
{
  return has_max_trans_acc_;
}

void pilz_industrial_motion_planner::CartesianLimit::setMaxTranslationalAcceleration(double max_trans_acc)
{
  has_max_trans_acc_ = true;
  max_trans_acc_ = max_trans_acc;
}

double pilz_industrial_motion_planner::CartesianLimit::getMaxTranslationalAcceleration() const
{
  return max_trans_acc_;
}

// Translational Deceleration Limit

bool pilz_industrial_motion_planner::CartesianLimit::hasMaxTranslationalDeceleration() const
{
  return has_max_trans_dec_;
}

void pilz_industrial_motion_planner::CartesianLimit::setMaxTranslationalDeceleration(double max_trans_dec)
{
  has_max_trans_dec_ = true;
  max_trans_dec_ = max_trans_dec;
}

double pilz_industrial_motion_planner::CartesianLimit::getMaxTranslationalDeceleration() const
{
  return max_trans_dec_;
}

// Rotational Velocity Limit

bool pilz_industrial_motion_planner::CartesianLimit::hasMaxRotationalVelocity() const
{
  return has_max_rot_vel_;
}

void pilz_industrial_motion_planner::CartesianLimit::setMaxRotationalVelocity(double max_rot_vel)
{
  has_max_rot_vel_ = true;
  max_rot_vel_ = max_rot_vel;
}

double pilz_industrial_motion_planner::CartesianLimit::getMaxRotationalVelocity() const
{
  return max_rot_vel_;
}

bool pilz_industrial_motion_planner::CartesianLimit::hasMaxJerk() const
{
  return has_max_jerk_;
}

void pilz_industrial_motion_planner::CartesianLimit::setMaxJerk(double max_jerk)
{
  has_max_jerk_ = true;
  max_jerk_ = max_jerk;
}

double pilz_industrial_motion_planner::CartesianLimit::getMaxJerk() const
{
  return max_jerk_;
}

  void pilz_industrial_motion_planner::CartesianLimit::setVelocityScalingFactor(double scaling_factor)
  {
    velocity_scaling_factor = scaling_factor;
  }

  double pilz_industrial_motion_planner::CartesianLimit::getVelocityScalingFactor() const
  {
    return velocity_scaling_factor;
  }
  
  void pilz_industrial_motion_planner::CartesianLimit::setAccelerationScalingFactor(double scaling_factor)
  {
    acceleration_scaling_factor=scaling_factor;
  }

  double pilz_industrial_motion_planner::CartesianLimit::getAccelerationScalingFactor() const
  {
    return acceleration_scaling_factor;
  }
  