#pragma once

#include "kdl/velocityprofile.hpp"
#include <iostream>

namespace pilz_industrial_motion_planner
{
	class VelocityProfileDoubles : public KDL::VelocityProfile
	{
	public:
		/**
		 * @brief Constructor
		 * @param max_vel: maximal velocity (absolute value, always positive)
		 * @param max_acc: maximal acceleration (absolute value, always positive)
		 * @param max_jerk: maximal jerk (absolute value, always positive)
		 */
		VelocityProfileDoubles(double max_vel, double max_acc, double max_jerk);
		
		/**
		 * @brief Constructor(max jerk is 3*max_acc)
		 * @param max_vel: maximal velocity (absolute value, always positive)
		 * @param max_acc: maximal acceleration (absolute value, always positive)
		 */
		VelocityProfileDoubles(double max_vel, double max_acc);
		
		/**
		 * @brief compute the fastest profile
		 * Algorithm:
		 * @param pos1: start position
		 * @param pos2: goal position
		 */
		void SetProfile(double pos1, double pos2) override;

		/**
		 * @brief Profile scaled by the total duration
		 * @param pos1: start position
		 * @param pos2: goal position
		 * @param duration: trajectory duration (must be longer than fastest case,
		 * otherwise will be ignored)
		 */
		void SetProfileDuration(double pos1, double pos2, double duration) override;

		/**
		 * @brief Profile scaled by the total duration
		 * 
		 * @param pos1 起始位置
		 * @param pos2 目标位置
		 * @param duration 轨迹总时间(要比最短时间轨迹长,否则将返回false)
		 * @param alpha 加速段和减速段持续时间相对总时间的占比
		 * @param beta 加加速时间(减减速时间)相对加速段(减速段时间)的占比
		 * @return true 成功
		 * @return false 失败
		 */
		bool MySetProfileDuration(double pos1,double pos2,double duration,double alpha=0.33,double beta=0.2);

		/**
		 * @brief Profile scaled by each durations(must be longer than the fastest axis, ohtnerwise will return false)
		 * @param pos1: 起始位置
		 * @param pos2: 目标位置
		 * @param T_jerk1: 加(减)加速时间
		 * @param T_acc:加速运动时间
		 * @param T_const:匀速运动时间
		 * @param T_jerk2:加(减)减速运动时间
		 * @param T_dec:减速运动时间
		 */
		bool setProfileAllDurations(double pos1, double pos2, double T_jerk1, double T_acc,double T_const,double T_jerk2, double T_dec);

		/**
		 * @brief Profile with start velocity
		 * @param pos1: start position
		 * @param pos2: goal position
		 * @param vel1: start velocity
		 * @param vel2: goal velocity
		 * @return
		 */
		bool setProfileVelocities(double pos1, double pos2, double vel1, double vel2);

		/**
		 * @brief get the time of acc phase
		 * @return
		 */
		double firstPhaseDuration() const
		{
			return T_acc_;
		}
		/**
		 * @brief get the time of constant velcity phase
		 * @return
		 */
		double secondPhaseDuration() const
		{
			return T_const_;
		}
		/**
		 * @brief get the time of dec phase
		 * @return
		 */
		double thirdPhaseDuration() const
		{
			return T_dec_;
		}
		/**
		 * @brief Compares two Asymmetric Trapezoidal Velocity Profiles.
		 *
		 * @return True if equal, false otherwise.
		 */
		bool operator==(const VelocityProfileDoubles &other) const;

		/**
		 * @brief Duration
		 * @return total duration of the trajectory
		 */
		double Duration() const override;
		/**
		 * @brief Get position at given time
		 * @param time
		 * @return
		 */
		double Pos(double time) const override;
		/**
		 * @brief Get velocity at given time
		 * @param time
		 * @return
		 */
		double Vel(double time) const override;
		/**
		 * @brief Get given acceleration/deceleration at given time
		 * @param time
		 * @return
		 */
		double Acc(double time) const override;
		/**
		 * @brief Get given jerk at given time
		 * @param time
		 * @return
		 */
		double Jerk(double time) const;
		/**
		 * @brief Write basic information
		 * @param os
		 */
		void Write(std::ostream &os) const override;
		/**
		 * @brief returns copy of current VelocityProfile object
		 * @return
		 */
		KDL::VelocityProfile *Clone() const override;

		friend std::ostream &operator<<(std::ostream &os, const VelocityProfileDoubles &p); // LCOV_EXCL_LINE

		~VelocityProfileDoubles() override;

	private:
		/// helper functions
		void setEmptyProfile();
		void transformparam();

	private:
		/// specification of the motion profile :
		double max_vel_;
		double min_vel_;
		double max_acc_;
		double min_acc_;
		double max_jerk_;
		double min_jerk_;

		double start_pos_;
		double end_pos_;
		double start_vel_;
		double end_vel_;
		double T_acc_;
		double T_const_;
		double T_dec_;
		double T_jerk1_;
		double T_jerk2_;
		double lim_vel_;
		double lim_acca_;
		double lim_deca_;
		double sigma_;
	};
	std::ostream &operator<<(std::ostream &os,
							 const VelocityProfileDoubles &p);
}
