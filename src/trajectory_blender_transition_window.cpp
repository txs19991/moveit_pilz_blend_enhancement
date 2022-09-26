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

#include "pilz_industrial_motion_planner/trajectory_blender_transition_window.h"

#include <algorithm>
#include <memory>
#include <math.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/planning_interface/planning_interface.h>

bool pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow::blend(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const pilz_industrial_motion_planner::TrajectoryBlendRequest& req,
    pilz_industrial_motion_planner::TrajectoryBlendResponse& res)
{
  ROS_INFO("Start trajectory blending using transition window.");

  double sampling_time = 0.;
  if (!validateRequest(req, sampling_time, res.error_code))
  {
    ROS_ERROR("Trajectory blend request is not valid.");
    return false;
  }

  // search for intersection points of the two trajectories with the blending
  // sphere
  // intersection points belongs to blend trajectory after blending
  std::size_t first_intersection_index;
  std::size_t second_intersection_index;
  if (!searchIntersectionPoints(req, first_intersection_index, second_intersection_index))
  {
    ROS_ERROR("Blend radius to large.");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // Select blending period and adjust the start and end point of the blend
  // phase
  std::size_t blend_align_index;
  determineTrajectoryAlignment(req, first_intersection_index, second_intersection_index, blend_align_index);

  // blend the trajectories in Cartesian space
  pilz_industrial_motion_planner::CartesianTrajectory blend_trajectory_cartesian;
  blendTrajectoryCartesian(req, first_intersection_index, second_intersection_index, blend_align_index, sampling_time,
                           blend_trajectory_cartesian);

  // generate the blending trajectory in joint space
  std::map<std::string, double> initial_joint_position, initial_joint_velocity;
  for (const std::string& joint_name :
       req.first_trajectory->getFirstWayPointPtr()->getJointModelGroup(req.group_name)->getActiveJointModelNames())
  {
    initial_joint_position[joint_name] =
        req.first_trajectory->getWayPoint(first_intersection_index - 1).getVariablePosition(joint_name);
    initial_joint_velocity[joint_name] =
        req.first_trajectory->getWayPoint(first_intersection_index - 1).getVariableVelocity(joint_name);
  }
  trajectory_msgs::JointTrajectory blend_joint_trajectory;
  moveit_msgs::MoveItErrorCodes error_code;

  if (!generateJointTrajectory(planning_scene, limits_.getJointLimitContainer(), blend_trajectory_cartesian,
                               req.group_name, req.link_name, initial_joint_position, initial_joint_velocity,
                               blend_joint_trajectory, error_code, true))
  {
    // LCOV_EXCL_START
    ROS_INFO("Failed to generate joint trajectory for blending trajectory.");
    res.error_code.val = error_code.val;
    return false;
    // LCOV_EXCL_STOP
  }
  // TODO:初始和结尾速度/加速度根据其他数据计算
  auto q0 = req.first_trajectory->getWayPoint(first_intersection_index - 1);
  auto q1 = blend_joint_trajectory.points.begin();
  auto q2 = blend_joint_trajectory.points.begin() + 1;
  auto qn__ = blend_joint_trajectory.points.end() - 2;
  auto qn_ = blend_joint_trajectory.points.end() - 1;
  auto qn = req.second_trajectory->getWayPoint(second_intersection_index + 1);
  // *均匀采样时间
  for (size_t i = 0; i < blend_joint_trajectory.joint_names.size(); ++i)
  {
    double v1 = (q2->positions[i] - q0.getVariablePosition(blend_joint_trajectory.joint_names[i])) / (2 * sampling_time);
    double vn = (qn.getVariablePosition(blend_joint_trajectory.joint_names[i]) - qn__->positions[i]) / (2 * sampling_time);
    double a1 = (q2->positions[i] - 2 * q1->positions[i] + q0.getVariablePosition(blend_joint_trajectory.joint_names[i])) / (2 * pow(sampling_time, 2));
    double an = (qn.getVariablePosition(blend_joint_trajectory.joint_names[i]) - 2 * qn_->positions[i] + qn__->positions[i]) / (2 * pow(sampling_time, 2));
    q1->velocities.emplace_back(v1);
    qn_->velocities.emplace_back(vn);
    q1->accelerations.emplace_back(a1);
    qn_->accelerations.emplace_back(an);
  }

  res.first_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(req.first_trajectory->getRobotModel(),
                                                                             req.first_trajectory->getGroup());
  res.blend_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(req.first_trajectory->getRobotModel(),
                                                                             req.first_trajectory->getGroup());
  res.second_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(req.first_trajectory->getRobotModel(),
                                                                              req.first_trajectory->getGroup());

  // set the three trajectories after blending in response
  // erase the points [first_intersection_index, back()] from the first
  // trajectory
  for (size_t i = 0; i < first_intersection_index; ++i)
  {
    res.first_trajectory->insertWayPoint(i, req.first_trajectory->getWayPoint(i),
                                         req.first_trajectory->getWayPointDurationFromPrevious(i));
  }

  // append the blend trajectory
  res.blend_trajectory->setRobotTrajectoryMsg(req.first_trajectory->getFirstWayPoint(), blend_joint_trajectory);
  // copy the points [second_intersection_index, len] from the second trajectory
  for (size_t i = second_intersection_index + 1; i < req.second_trajectory->getWayPointCount(); ++i)
  {
    res.second_trajectory->insertWayPoint(i - (second_intersection_index + 1), req.second_trajectory->getWayPoint(i),
                                          req.second_trajectory->getWayPointDurationFromPrevious(i));
  }

  // adjust the time from start
  res.second_trajectory->setWayPointDurationFromPrevious(0, sampling_time);

  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  return true;
}

bool pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow::validateRequest(
    const pilz_industrial_motion_planner::TrajectoryBlendRequest& req, double& sampling_time,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
  ROS_DEBUG("Validate the trajectory blend request.");

  // check planning group
  if (!req.first_trajectory->getRobotModel()->hasJointModelGroup(req.group_name))
  {
    ROS_ERROR_STREAM("Unknown planning group: " << req.group_name);
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }

  // check link exists
  if (!req.first_trajectory->getRobotModel()->hasLinkModel(req.link_name) &&
      !req.first_trajectory->getLastWayPoint().hasAttachedBody(req.link_name))
  {
    ROS_ERROR_STREAM("Unknown link name: " << req.link_name);
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
    return false;
  }

  if (req.blend_radius <= 0)
  {
    ROS_ERROR("Blending radius must be positive");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // end position of the first trajectory and start position of second
  // trajectory must be the same
  if (!pilz_industrial_motion_planner::isRobotStateEqual(
          req.first_trajectory->getLastWayPoint(), req.second_trajectory->getFirstWayPoint(), req.group_name, EPSILON))
  {
    ROS_ERROR_STREAM("During blending the last point of the preceding and the first point of the succeding trajectory");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // same uniform sampling time
  if (!pilz_industrial_motion_planner::determineAndCheckSamplingTime(req.first_trajectory, req.second_trajectory,
                                                                     EPSILON, sampling_time))
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // end position of the first trajectory and start position of second
  // trajectory must have zero
  // velocities/accelerations
  if (!pilz_industrial_motion_planner::isRobotStateStationary(req.first_trajectory->getLastWayPoint(), req.group_name,
                                                              EPSILON) ||
      !pilz_industrial_motion_planner::isRobotStateStationary(req.second_trajectory->getFirstWayPoint(), req.group_name,
                                                              EPSILON))
  {
    ROS_ERROR("Intersection point of the blending trajectories has non-zero "
              "velocities/accelerations.");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  return true;
}

// 计算控制点
template <typename T1, typename T2>
void CalControlPoints(int bezier_degree, std::vector<T1> &p, std::vector<double> lamdax, const std::vector<T2> &diffx)
{
  if (bezier_degree == 3)
  {
    p[1] = p[0].rplus(lamdax[0] * diffx[0]);
    p[2] = p[3].rplus(-lamdax[1] * diffx[1]);
  }
  else if (bezier_degree == 5)
  {
    p[1] = p[0].rplus(lamdax[0] * diffx[0]);
    p[2] = p[1].rplus(lamdax[0] * diffx[0] + lamdax[1] * diffx[2]);
    p[4] = p[5].rplus(-lamdax[3] * diffx[1]);
    p[3] = p[4].rplus(-lamdax[3] * diffx[1] + lamdax[2] * diffx[3]);
  }
}

// 根据最大速度计算过渡段的时间
void CalBlendNum(const std::vector<manif::R3d> best_control_points_tra, const std::vector<manif::SO3d> best_control_points_rot, double &blend_sample_num, const double sampling_time, const double limit_vel_tra, const double limit_acc_tra, const double limit_vel_rot)
{
  double vel_tra, acc_tra, maxvel_tra, maxacc_tra;
  double vel_rot, maxvel_rot;
  std::vector<manif::R3d> b_tra;
  std::vector<manif::SO3d> b_rot;
  for (size_t i = 0; i <= blend_sample_num; ++i)
  {
    double u = i / blend_sample_num;
    b_tra.emplace_back(manif::decasteljau(best_control_points_tra, u));
    b_rot.emplace_back(manif::decasteljau(best_control_points_rot, u));
  }
  for (size_t i = 0; i < b_tra.size() - 1; ++i)
  {
    vel_tra = ((b_tra[i + 1] - b_tra[i]) / sampling_time).weightedNorm();
    vel_tra > maxvel_tra ? maxvel_tra = vel_tra : 0;
    vel_rot = ((b_rot[i + 1] - b_rot[i]) / sampling_time).weightedNorm();
    vel_rot > maxvel_rot ? maxvel_rot = vel_rot : 0;
    if (i > 0)
    {
      acc_tra = (((b_tra[i + 1] - b_tra[i]) - (b_tra[i] - b_tra[i - 1])) / (2 * pow(sampling_time, 2))).weightedNorm();
      acc_tra > maxacc_tra ? maxacc_tra = acc_tra : 0;
    }
  }
  double lamda = std::min(std::min(limit_vel_tra / maxvel_tra, sqrt(limit_acc_tra / maxacc_tra)), limit_vel_rot / maxvel_rot);
  blend_sample_num = std::ceil(blend_sample_num * lamda);
  std::cout << " lamda: " << lamda << std::endl;
}

void pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow::blendTrajectoryCartesian(
    const pilz_industrial_motion_planner::TrajectoryBlendRequest& req, const std::size_t first_interse_index,
    const std::size_t second_interse_index, const std::size_t blend_align_index, double sampling_time,
    pilz_industrial_motion_planner::CartesianTrajectory& trajectory) const
{
  // other fields of the trajectory
  trajectory.group_name = req.group_name;
  trajectory.link_name = req.link_name;
  // 第一条和第二条轨迹的插入点的位置和速度
  int align_num = 1;
  int bezier_degree = 5;
  // std::map<int, manif::SE3d> liepose1, lieposen;
  std::map<int, manif::SO3d> lierot1, lierotn;
  std::map<int, manif::R3d> lietra1, lietran;
  for (int i = -1; i < align_num + 1; ++i)
  {
    // liepose1[i] = req.first_trajectory->getWayPoint(first_interse_index + i).getFrameTransform(req.link_name);
    manif::SE3d &&temp1 = req.first_trajectory->getWayPoint(first_interse_index + i).getFrameTransform(req.link_name);
    lietra1[i] = temp1.translation();
    lierot1[i] = temp1.asSO3();
    // lieposen[-i] = req.second_trajectory->getWayPoint(second_interse_index - i).getFrameTransform(req.link_name);
    manif::SE3d &&tempn = req.second_trajectory->getWayPoint(second_interse_index - i).getFrameTransform(req.link_name);
    lietran[-i] = tempn.translation();
    lierotn[-i] = tempn.asSO3();
  }
  // manif::SE3Tangentd lievel1 = (liepose1[1] - liepose1[-1]) / (2 * sampling_time);
  // manif::SE3Tangentd lieveln = (lieposen[1] - lieposen[-1]) / (2 * sampling_time);
  // manif::SE3Tangentd lieacc1 = ((liepose1[1] - liepose1[0]) - (liepose1[0] - liepose1[-1])) / (2 * pow(sampling_time, 2));
  // manif::SE3Tangentd lieaccn = ((lieposen[1] - lieposen[0]) - (lieposen[0] - lieposen[-1])) / (2 * pow(sampling_time, 2));

  // !LOCAL坐标系下的表示！！！
  manif::SO3Tangentd lierotvel1 = (lierot1[1] - lierot1[-1]) / (2 * sampling_time);
  manif::SO3Tangentd lierotveln = (lierotn[1] - lierotn[-1]) / (2 * sampling_time);
  manif::SO3Tangentd lierotacc1 = ((lierot1[1] - lierot1[0]) - (lierot1[0] - lierot1[-1])) / (2 * pow(sampling_time, 2));
  manif::SO3Tangentd lierotaccn = ((lierotn[1] - lierotn[0]) - (lierotn[0] - lierotn[-1])) / (2 * pow(sampling_time, 2));

  manif::R3Tangentd lietravel1 = (lietra1[1] - lietra1[-1]) / (2 * sampling_time);
  manif::R3Tangentd lietraveln = (lietran[1] - lietran[-1]) / (2 * sampling_time);
  manif::R3Tangentd lietraacc1 = ((lietra1[1] - lietra1[0]) - (lietra1[0] - lietra1[-1])) / (2 * pow(sampling_time, 2));
  manif::R3Tangentd lietraaccn = ((lietran[1] - lietran[0]) - (lietran[0] - lietran[-1])) / (2 * pow(sampling_time, 2));

  // std::vector<manif::SE3Tangentd> diffx{lievel1, lieveln, lieacc1, lieaccn};
  std::vector<manif::SO3Tangentd> diffrotx{lierotvel1, lierotveln, lierotacc1, lierotaccn};
  std::vector<manif::R3Tangentd> difftrax{lietravel1, lietraveln, lietraacc1, lietraaccn};
  // *Jacobian
  // auto blend_sample_vel1 = rstate1.getVariableVelocities();
  // Eigen::MatrixXd jacobian1 = rstate1.getJacobian(req.first_trajectory->getGroup());
  // Eigen::VectorXd vel1(rstate1.getVariableCount());
  // for (Eigen::Index i = 0; i < vel1.size(); ++i)
  // {
  //   vel1(i) = blend_sample_vel1[i];
  // }
  // Eigen::VectorXd twist1 = jacobian1 * vel1;
  // manif::SE3Tangentd lie_twist1(-twist1);
  // ROS_INFO_STREAM("twist1: \n"
  //                 << twist1 << "\n");

  // blend the trajectory
  double blend_sample_num = second_interse_index + blend_align_index - first_interse_index + 3;
  // Eigen::Matrix<double, 3, 4> param = cubicbezier(blend_sample_pose1, twist1, blend_sample_pose2, twist2, blend_sample_num * sampling_time);
  pilz_industrial_motion_planner::CartesianTrajectoryPoint waypoint;

  std::vector<manif::SO3d> best_control_points_rot(bezier_degree + 1);
  std::vector<manif::SO3d> px_1_rot(bezier_degree + 1), px_2_rot(bezier_degree + 1);
  std::vector<double> lamdax_rot;
  std::vector<int> seq_rot;
  if (bezier_degree == 3)
  {
    std::vector<double>{0.17, 0.17}.swap(lamdax_rot);
    std::vector<int>{0, 1}.swap(seq_rot);
  }
  else if (bezier_degree == 5)
  {
    std::vector<double>{0.1, 0.025, 0.025, 0.1}.swap(lamdax_rot);
    std::vector<int>{0, 3, 1, 2}.swap(seq_rot);
  }
  std::vector<double> lamdax_1_rot(lamdax_rot), lamdax_2_rot(lamdax_rot);
  std::vector<double> lastdeltax_rot(2, DELTA + 10), deltax_rot(2, DELTA + 1), deltax_1_rot(2, DELTA + 1), deltax_2_rot(2, DELTA + 1);
  std::map<int, manif::SO3d> calrot1, calrotn;
  px_1_rot[0] = lierot1[0];
  px_1_rot[bezier_degree] = lierotn[0];
  px_2_rot[0] = lierot1[0];
  px_2_rot[bezier_degree] = lierotn[0];
  best_control_points_rot[0] = lierot1[0];
  best_control_points_rot[bezier_degree] = lierotn[0];

  std::vector<manif::R3d> best_control_points_tra(bezier_degree + 1);
  std::vector<manif::R3d> px_1_tra(bezier_degree + 1), px_2_tra(bezier_degree + 1);
  std::vector<double> lamdax_tra;
  std::vector<int> seq_tra;
  // *三阶贝塞尔
  if (bezier_degree == 3)
  {
    std::vector<double>{0.17, 0.17}.swap(lamdax_tra);
    std::vector<int>{0, 1}.swap(seq_tra);
  }
  // *五阶贝塞尔
  else if (bezier_degree == 5)
  {
    std::vector<double>{0.1, 0.025, 0.025, 0.1}.swap(lamdax_tra);
    std::vector<int>{0, 3, 1, 2}.swap(seq_tra);
  }
  std::vector<double> lamdax_1_tra(lamdax_tra), lamdax_2_tra(lamdax_tra);
  std::vector<double> lastdeltax_tra(2, DELTA + 10), deltax_tra(2, DELTA + 1), deltax_1_tra(2, DELTA + 1), deltax_2_tra(2, DELTA + 1);
  std::map<int, manif::R3d> caltra1, caltran;
  px_1_tra[0] = lietra1[0];
  px_1_tra[bezier_degree] = lietran[0];
  px_2_tra[0] = lietra1[0];
  px_2_tra[bezier_degree] = lietran[0];
  best_control_points_tra[0] = lietra1[0];
  best_control_points_tra[bezier_degree] = lietran[0];
  // CalControlPoints(bezier_degree, best_control_points_rot, lamdax_rot, diffrotx);
  // CalControlPoints(bezier_degree, best_control_points_tra, lamdax_tra, difftrax);
  for (size_t re = 0; re < 1; ++re)
  {
    // CalBlendNum(best_control_points_tra, best_control_points_rot, blend_sample_num, sampling_time, limits_.getCartesianLimits().getVelocityScalingFactor() * limits_.getCartesianLimits().getMaxTranslationalVelocity(), limits_.getCartesianLimits().getAccelerationScalingFactor() * limits_.getCartesianLimits().getMaxTranslationalAcceleration(), limits_.getCartesianLimits().getVelocityScalingFactor() * limits_.getCartesianLimits().getMaxRotationalVelocity());
    // *SO(3)
    for (int i : seq_rot)
    {
      lamdax_1_rot[i] = 1e-4;
      lamdax_2_rot[i] = 0.4;
      for (size_t epoch = 0; epoch < 100; ++epoch)
      {
        std::vector<double>(2, 0.0).swap(deltax_rot);
        // 左侧参数误差
        CalControlPoints(bezier_degree, px_1_rot, lamdax_1_rot, diffrotx);
        for (int j = 1; j < align_num + 1; ++j)
        {
          calrot1[j] = manif::decasteljau(px_1_rot, double(j) / blend_sample_num);
          calrotn[-j] = manif::decasteljau(px_1_rot, 1.0 + double(-j) / blend_sample_num);
          deltax_rot[0] += Eigen::VectorXd((calrot1[j] - lierot1[j]).coeffs()).norm();
          deltax_rot[0] += Eigen::VectorXd((calrotn[-j] - lierotn[-j]).coeffs()).norm();
        }
        // 右侧参数误差
        CalControlPoints(bezier_degree, px_2_rot, lamdax_2_rot, diffrotx);
        for (int j = 1; j < align_num + 1; ++j)
        {
          calrot1[j] = manif::decasteljau(px_2_rot, double(j) / blend_sample_num);
          calrotn[-j] = manif::decasteljau(px_2_rot, 1.0 + double(-j) / blend_sample_num);
          deltax_rot[1] += Eigen::VectorXd((calrot1[j] - lierot1[j]).coeffs()).norm();
          deltax_rot[1] += Eigen::VectorXd((calrotn[-j] - lierotn[-j]).coeffs()).norm();
        }
        if ((fabs(lastdeltax_rot[1] - deltax_rot[1]) + fabs(lastdeltax_rot[0] - deltax_rot[0])) < 1e-7)
        {
          // std::cout << "迭代次数: " << epoch << std::endl;
          break;
        }
        // 左侧误差大，修改左侧参数，记录右侧参数为最佳
        if (deltax_rot[0] > deltax_rot[1])
        {
          lamdax_rot[i] = lamdax_2_rot[i];
          lamdax_1_rot[i] = (lamdax_1_rot[i] + lamdax_2_rot[i]) / 2.0;
        }
        else
        {
          lamdax_rot[i] = lamdax_1_rot[i];
          lamdax_2_rot[i] = (lamdax_1_rot[i] + lamdax_2_rot[i]) / 2.0;
        }
        lastdeltax_rot = deltax_rot;
      }
    }
    // *R(3)
    for (int i : seq_tra)
    {
      lamdax_1_tra[i] = 1e-4;
      lamdax_2_tra[i] = 0.4;
      for (size_t epoch = 0; epoch < 100; ++epoch)
      {
        std::vector<double>(2, 0.0).swap(deltax_tra);
        // 左侧参数误差
        CalControlPoints(bezier_degree, px_1_tra, lamdax_1_tra, difftrax);
        for (int j = 1; j < align_num + 1; ++j)
        {
          caltra1[j] = manif::decasteljau(px_1_tra, double(j) / blend_sample_num);
          caltran[-j] = manif::decasteljau(px_1_tra, 1.0 + double(-j) / blend_sample_num);
          deltax_tra[0] += Eigen::VectorXd((caltra1[j] - lietra1[j]).coeffs()).norm();
          deltax_tra[0] += Eigen::VectorXd((caltran[-j] - lietran[-j]).coeffs()).norm();
        }
        // 右侧参数误差
        CalControlPoints(bezier_degree, px_2_tra, lamdax_2_tra, difftrax);
        for (int j = 1; j < align_num + 1; ++j)
        {
          caltra1[j] = manif::decasteljau(px_2_tra, double(j) / blend_sample_num);
          caltran[-j] = manif::decasteljau(px_2_tra, 1.0 + double(-j) / blend_sample_num);
          deltax_tra[1] += Eigen::VectorXd((caltra1[j] - lietra1[j]).coeffs()).norm();
          deltax_tra[1] += Eigen::VectorXd((caltran[-j] - lietran[-j]).coeffs()).norm();
        }
        if ((fabs(lastdeltax_tra[1] - deltax_tra[1]) + fabs(lastdeltax_tra[0] - deltax_tra[0])) < 1e-7)
        {
          // std::cout << "迭代次数: " << epoch << std::endl;
          break;
        }
        // 左侧误差大，修改左侧参数，记录右侧参数为最佳
        if (deltax_tra[0] > deltax_tra[1])
        {
          lamdax_tra[i] = lamdax_2_tra[i];
          lamdax_1_tra[i] = (lamdax_1_tra[i] + lamdax_2_tra[i]) / 2.0;
        }
        else
        {
          lamdax_tra[i] = lamdax_1_tra[i];
          lamdax_2_tra[i] = (lamdax_1_tra[i] + lamdax_2_tra[i]) / 2.0;
        }
        lastdeltax_tra = deltax_tra;
      }
    }
    CalControlPoints(bezier_degree, best_control_points_rot, lamdax_rot, diffrotx);
    CalControlPoints(bezier_degree, best_control_points_tra, lamdax_tra, difftrax);
  }
  std::cout << "\n-----SO(3)-----\n";
  std::cout << "\n 最小误差: "
            << "\n";
  for (auto it : deltax_rot)
  {
    std::cout << it << "\n";
  }
  std::cout << "\n 最佳偏置: "
            << "\n";
  for (auto it : lamdax_rot)
  {
    std::cout << it << "\n";
  }
  std::cout << "\n----------\n";

  std::cout << "\n-----R(3)-----\n";
  std::cout << "\n 最小误差: "
            << "\n";
  for (auto it : deltax_tra)
  {
    std::cout << it << "\n";
  }
  std::cout << "\n 最佳偏置: "
            << "\n";
  for (auto it : lamdax_tra)
  {
    std::cout << it << "\n";
  }
  std::cout << "\n----------\n";

  // *SE(3)
  // std::vector<manif::SE3d> best_control_points(bezier_degree + 1);
  // std::vector<manif::SE3d> px_1(bezier_degree + 1), px_2(bezier_degree + 1);
  // std::vector<double> lamdax;
  // std::vector<int> seq;
  // // *三阶贝塞尔
  // if (bezier_degree == 3)
  // {
  //   std::vector<double>{0.17, 0.17}.swap(lamdax);
  //   std::vector<int>{0, 1}.swap(seq);
  // }
  // // *五阶贝塞尔
  // else if (bezier_degree == 5)
  // {
  //   std::vector<double>{0.1, 0.025, 0.025, 0.1}.swap(lamdax);
  //   std::vector<int>{0, 3, 1, 2}.swap(seq);
  // }
  // std::vector<double> lamdax_1(lamdax), lamdax_2(lamdax);
  // std::vector<double> lastdeltax(2, DELTA + 10), deltax(2, DELTA + 1), deltax_1(2, DELTA + 1), deltax_2(2, DELTA + 1);
  // std::map<int, manif::SE3d> calpose1, calposen;
  // px_1[0] = liepose1[0];
  // px_1[bezier_degree] = lieposen[0];
  // px_2[0] = liepose1[0];
  // px_2[bezier_degree] = lieposen[0];
  // best_control_points[0] = liepose1[0];
  // best_control_points[bezier_degree] = lieposen[0];
  // for (size_t re = 0; re < 1; ++re)
  // {
  //   for (int i : seq)
  //   {
  //     lamdax_1[i] = 1e-4;
  //     lamdax_2[i] = 0.4;
  //     for (size_t epoch = 0; epoch < 100; ++epoch)
  //     {
  //       std::vector<double>(2, 0.0).swap(deltax);
  //       // 左侧参数误差
  //       CalControlPoints(bezier_degree, px_1, lamdax_1, diffx);
  //       for (int j = 1; j < align_num + 1; ++j)
  //       {
  //         calpose1[j] = manif::decasteljau(px_1, double(j) / blend_sample_num);
  //         calposen[-j] = manif::decasteljau(px_1, 1.0 + double(-j) / blend_sample_num);
  //         deltax[0] += Eigen::VectorXd((calpose1[j] - liepose1[j]).coeffs()).norm();
  //         deltax[0] += Eigen::VectorXd((calposen[-j] - lieposen[-j]).coeffs()).norm();
  //       }
  //       // 右侧参数误差
  //       CalControlPoints(bezier_degree, px_2, lamdax_2, diffx);
  //       for (int j = 1; j < align_num + 1; ++j)
  //       {
  //         calpose1[j] = manif::decasteljau(px_2, double(j) / blend_sample_num);
  //         calposen[-j] = manif::decasteljau(px_2, 1.0 + double(-j) / blend_sample_num);
  //         deltax[1] += Eigen::VectorXd((calpose1[j] - liepose1[j]).coeffs()).norm();
  //         deltax[1] += Eigen::VectorXd((calposen[-j] - lieposen[-j]).coeffs()).norm();
  //       }
  //       if ((fabs(lastdeltax[1] - deltax[1]) + fabs(lastdeltax[0] - deltax[0])) < 1e-7)
  //       {
  //         // std::cout << "迭代次数: " << epoch << std::endl;
  //         break;
  //       }
  //       // 左侧误差大，修改左侧参数，记录右侧参数为最佳
  //       if (deltax[0] > deltax[1])
  //       {
  //         lamdax[i] = lamdax_2[i];
  //         lamdax_1[i] = (lamdax_1[i] + lamdax_2[i]) / 2.0;
  //       }
  //       else
  //       {
  //         lamdax[i] = lamdax_1[i];
  //         lamdax_2[i] = (lamdax_1[i] + lamdax_2[i]) / 2.0;
  //       }
  //       lastdeltax = deltax;
  //     }
  //   }
  // }
  // CalControlPoints(bezier_degree, best_control_points, lamdax, diffx);
  // std::cout << "\n-----SE(3)-----\n";
  // std::cout << "\n 最小误差: "
  //           << "\n";
  // for (auto it : deltax)
  // {
  //   std::cout << it << "\n";
  // }
  // std::cout << "\n 最佳偏置: "
  //           << "\n";
  // for (auto it : lamdax)
  // {
  //   std::cout << it << "\n";
  // }
  // std::cout << "\n----------\n";

  Eigen::Isometry3d blend_sample_pose;
  for (size_t i = 0; i <= blend_sample_num; ++i)
  {
    double u = i / blend_sample_num;
    // *李群自带插值
    // blend_sample_pose = manif::interpolate(lie_pose1, lie_pose2, u, manif::INTERP_METHOD::CNSMOOTH, lie_twist1, lie_twist2).transform();
    // *R3和SO3分离的三阶贝塞尔
    auto &&a = manif::decasteljau(best_control_points_tra, u);
    auto &&b = manif::decasteljau(best_control_points_rot, u);
    blend_sample_pose = manif::SE3d(a.coeffs(), b).transform();
    // *SE3
    // blend_sample_pose = manif::decasteljau(best_control_points, u).transform();

    // // 位置平滑插值，三阶贝塞尔
    // blend_sample_pose.translation() = param.col(0) + param.col(1) * u + param.col(2) * pow(u, 2) + param.col(3) * pow(u, 3);

    // // 旋转平滑插值，slerp
    // double alpha = 6 * std::pow(u, 5) - 15 * std::pow(u, 4) + 10 * std::pow(u, 3);
    // Eigen::Quaterniond start_quat(blend_sample_pose1.rotation());
    // Eigen::Quaterniond end_quat(blend_sample_pose2.rotation());
    // blend_sample_pose.linear() = start_quat.slerp(alpha, end_quat).toRotationMatrix();

    // push to the trajectory
    waypoint.pose = tf2::toMsg(blend_sample_pose);
    waypoint.time_from_start = ros::Duration((i + 1.0) * sampling_time);
    trajectory.points.push_back(waypoint);
  }
}

bool pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow::searchIntersectionPoints(
    const pilz_industrial_motion_planner::TrajectoryBlendRequest& req, std::size_t& first_interse_index,
    std::size_t& second_interse_index) const
{
  ROS_INFO("Search for start and end point of blending trajectory.");

  // compute the position of the center of the blend sphere
  // (last point of the first trajectory, first point of the second trajectory)
  Eigen::Isometry3d circ_pose = req.first_trajectory->getLastWayPoint().getFrameTransform(req.link_name);

  // Searh for intersection points according to distance
  if (!linearSearchIntersectionPoint(req.link_name, circ_pose.translation(), req.blend_radius, req.first_trajectory,
                                     true, first_interse_index))
  {
    ROS_ERROR_STREAM("Intersection point of first trajectory not found.");
    return false;
  }
  ROS_INFO_STREAM("Intersection point of first trajectory found, index: " << first_interse_index);

  if (!linearSearchIntersectionPoint(req.link_name, circ_pose.translation(), req.blend_radius, req.second_trajectory,
                                     false, second_interse_index))
  {
    ROS_ERROR_STREAM("Intersection point of second trajectory not found.");
    return false;
  }

  ROS_INFO_STREAM("Intersection point of second trajectory found, index: " << second_interse_index);
  return true;
}

void pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow::determineTrajectoryAlignment(
    const pilz_industrial_motion_planner::TrajectoryBlendRequest& req, std::size_t first_interse_index,
    std::size_t second_interse_index, std::size_t& blend_align_index) const
{
  size_t way_point_count_1 = req.first_trajectory->getWayPointCount() - first_interse_index;
  size_t way_point_count_2 = second_interse_index + 1;

  if (way_point_count_1 > way_point_count_2)
  {
    blend_align_index = req.first_trajectory->getWayPointCount() - second_interse_index - 1;
  }
  else
  {
    blend_align_index = first_interse_index;
  }
}
