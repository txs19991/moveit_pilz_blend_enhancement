#include "pilz_industrial_motion_planner/velocity_profile_doubles.h"

namespace pilz_industrial_motion_planner
{
	VelocityProfileDoubles::VelocityProfileDoubles(double max_vel, double max_acc, double max_jerk)
		: max_vel_(fabs(max_vel)), min_vel_(-fabs(max_vel)), max_acc_(fabs(max_acc)), min_acc_(-fabs(max_acc)), max_jerk_(fabs(max_jerk)), min_jerk_(-fabs(max_jerk)), start_pos_(0), end_pos_(0), start_vel_(0), end_vel_(0), T_acc_(0), T_const_(0), T_dec_(0), T_jerk1_(0), T_jerk2_(0), lim_vel_(0), lim_acca_(0), lim_deca_(0)
	{
	}

	VelocityProfileDoubles::VelocityProfileDoubles(double max_vel, double max_acc)
		: max_vel_(fabs(max_vel)), min_vel_(-fabs(max_vel)), max_acc_(fabs(max_acc)), min_acc_(-fabs(max_acc)), start_pos_(0), end_pos_(0), start_vel_(0), end_vel_(0), T_acc_(0), T_const_(0), T_dec_(0), T_jerk1_(0), T_jerk2_(0), lim_vel_(0), lim_acca_(0), lim_deca_(0)
	{
		max_jerk_ = 3*fabs(max_acc);
		min_jerk_ = -3*fabs(max_acc);
	}

	void VelocityProfileDoubles::SetProfile(double pos1, double pos2)
	{
		setProfileVelocities(pos1, pos2, 0, 0);
	}

	bool VelocityProfileDoubles::setProfileVelocities(double pos1, double pos2, double vel1, double vel2)
	{
		start_pos_ = pos1;
		end_pos_ = pos2;
		start_vel_ = vel1;
		end_vel_ = vel2;
		transformparam();
		// 检查能否进行规划
		double T1 = sqrt(abs(end_vel_ - start_vel_) / max_jerk_);
		double T2 = max_acc_ / max_jerk_;
		double h = end_pos_ - start_pos_;
		if (T1 < T2 && h >= T1 * (start_vel_ + end_vel_))
			;
		else if (T1 >= T2 && h >= 0.5 * (start_vel_ + end_vel_) * (T2 + abs(end_vel_ - start_vel_) / max_acc_))
			;
		else
			return false;
		if (start_pos_ == end_pos_)
		{
			// goal already reached, set everything to zero
			setEmptyProfile();
			return true;
		}
		else
		{
			// 达不到最大加速度
			if ((max_vel_ - start_vel_) * max_jerk_ < pow(max_acc_, 2))
			{
				T_jerk1_ = sqrt((max_vel_ - start_vel_) / max_jerk_);
				T_acc_ = 2 * T_jerk1_;
				lim_acca_ = max_jerk_ * T_jerk1_;
			}
			// 能够达到最大加速度
			else
			{
				T_jerk1_ = max_acc_ / max_jerk_;
				T_acc_ = T_jerk1_ + (max_vel_ - start_vel_) / max_acc_;
				lim_acca_ = max_acc_;
			}

			// 达不到最小加速度
			if ((max_vel_ - end_vel_) * max_jerk_ < pow(max_acc_, 2))
			{
				T_jerk2_ = sqrt((max_vel_ - end_vel_) / max_jerk_);
				T_dec_ = 2 * T_jerk2_;
				lim_deca_ = -max_jerk_ * T_jerk2_;
			}
			// 能够达到最小加速度
			else
			{
				T_jerk2_ = max_acc_ / max_jerk_;
				T_dec_ = T_jerk2_ + (max_vel_ - end_vel_) / max_acc_;
				lim_deca_ = -max_acc_;
			}
			// 计算匀速段时间
			T_const_ = (end_pos_ - start_pos_) / max_vel_ - (T_acc_ / 2) * (1 + start_vel_ / max_vel_) - (T_dec_ / 2) * (1 + end_vel_ / max_vel_);
			// 达到最大速度v_max，即存在匀速阶段
			if (T_const_ > 0)
			{
				lim_vel_ = max_vel_;
				return true;
			}
			// 达不到最大速度，即匀速阶段Tv<=0
			else
			{
				// 假设新的最大加速度和新的最小加速度均能达到
				T_const_ = 0;
				double delta = (pow(max_acc_, 4) / pow(max_jerk_, 2)) + 2 * (pow(start_vel_, 2) + pow(end_vel_, 2)) + max_acc_ * (4 * (end_pos_ - start_pos_) - 2 * (max_acc_ / max_jerk_) * (start_vel_ + end_vel_));
				T_acc_ = ((pow(max_acc_, 2) / max_jerk_) - 2.0 * start_vel_ + sqrt(delta)) / (2.0 * max_acc_);
				T_dec_ = ((pow(max_acc_, 2) / max_jerk_) - 2.0 * end_vel_ + sqrt(delta)) / (2.0 * max_acc_);
				double T_jerk = max_acc_ / max_jerk_;
				T_jerk1_ = T_jerk;
				T_jerk2_ = T_jerk;
				double gamma = 0.99;
				while (T_acc_ < 2 * T_jerk || T_dec_ < 2 * T_jerk)
				{
					// 没有加速段，只有减速段
					if (T_acc_ < 0)
					{
						T_acc_ = 0;
						T_jerk1_ = 0;
						T_dec_ = 2 * (end_pos_ - start_pos_) / (start_vel_ + end_vel_);
						T_jerk2_ = (max_jerk_ * (end_pos_ - start_pos_) - sqrt(max_jerk_ * (max_jerk_ * pow(end_pos_ - start_pos_, 2) + pow(end_vel_ + start_vel_, 2) * (end_vel_ - start_vel_)))) / (max_jerk_ * (end_vel_ + start_vel_));
						lim_acca_ = 0;
						lim_deca_ = -max_jerk_ * T_jerk2_;
						lim_vel_ = start_vel_;
						return true;
					}
					// 没有减速段，只有加速段
					else if (T_dec_ < 0)
					{
						T_dec_ = 0;
						T_jerk2_ = 0;
						T_acc_ = 2 * (end_pos_ - start_pos_) / (start_vel_ + end_vel_);
						T_jerk1_ = (max_jerk_ * (end_pos_ - start_pos_) - sqrt(max_jerk_ * (max_jerk_ * pow(end_pos_ - start_pos_, 2)) - pow(end_vel_ + start_vel_, 2) * (end_vel_ - start_vel_))) / (max_jerk_ * (end_vel_ + start_vel_));
						lim_acca_ = max_jerk_ * T_jerk1_;
						lim_deca_ = 0;
						lim_vel_ = start_vel_ + lim_acca_ * (T_acc_ - T_jerk1_);
						return true;
					}
					// 加速和减速阶段至少有一段不能达到最大加速度,则减小最大加速度
					max_acc_ = gamma * max_acc_;
					delta = (pow(max_acc_, 4) / pow(max_jerk_, 2)) + 2 * (pow(start_vel_, 2) + pow(end_vel_, 2)) + max_acc_ * (4 * (end_pos_ - start_pos_) - 2 * (max_acc_ / max_jerk_) * (start_vel_ + end_vel_));
					T_acc_ = ((pow(max_acc_, 2) / max_jerk_) - 2.0 * start_vel_ + sqrt(delta)) / (2.0 * max_acc_);
					T_dec_ = ((pow(max_acc_, 2) / max_jerk_) - 2.0 * end_vel_ + sqrt(delta)) / (2.0 * max_acc_);
					T_jerk = max_acc_ / max_jerk_;
					T_jerk1_ = T_jerk;
					T_jerk2_ = T_jerk;
				}
				// 加速段和减速段都能达到最大加速度
				lim_acca_ = max_acc_;
				lim_deca_ = -max_acc_;
				lim_vel_ = start_vel_+lim_acca_*(T_acc_-T_jerk1_);
				return true;
			}
		}
	}

	void VelocityProfileDoubles::SetProfileDuration(double pos1, double pos2, double duration)
	{
		MySetProfileDuration(pos1, pos2, duration);
	}

	bool VelocityProfileDoubles::MySetProfileDuration(double pos1, double pos2, double duration, double alpha, double beta)
	{
		// TODO:根据总Duration重新规划
		// ATTENTION:目前只有初末速度都为0的情况
		if(duration<Duration())	return false;
		start_pos_ = pos1;
		end_pos_ = pos2;
		start_vel_ = 0;
		end_vel_ = 0;
		transformparam();
		double T = duration;
		double h = end_pos_ - start_pos_;
		T_acc_ = alpha * T;
		T_dec_ = alpha * T;
		T_const_ = T - T_acc_ - T_dec_;
		T_jerk1_ = beta * T_acc_;
		T_jerk2_ = beta * T_dec_;
		double T_jerk = beta * T_acc_;
		lim_vel_ = max_vel_ = h / (T - T_acc_);
		min_vel_ = -max_vel_;
		lim_acca_ = max_acc_ = max_vel_ / (T_acc_ - T_jerk);
		lim_deca_ = min_acc_ = -max_acc_;
		min_acc_ = -max_acc_;
		max_jerk_ = max_acc_ / T_jerk;
		min_jerk_ = -max_jerk_;
		return true;
	}

	// 根据设定时间重新进行S型轨迹规划
	bool VelocityProfileDoubles::setProfileAllDurations(double pos1, double pos2, double T_jerk1, double T_acc, double T_const, double T_jerk2, double T_dec)
	{
		return false;
	}

	double VelocityProfileDoubles::Duration() const
	{
		return T_acc_ + T_const_ + T_dec_;
	}

	double VelocityProfileDoubles::Pos(double time) const
	{
		double T = Duration();
		double q;
		// 加速段
		if (time >= 0 && time < T_jerk1_)
			q = start_pos_ + start_vel_ * time + max_jerk_ * pow(time, 3) / 6;
		else if (time >= T_jerk1_ && time < T_acc_ - T_jerk1_)
			q = start_pos_ + start_vel_ * time + (lim_acca_ / 6) * (3 * pow(time, 2) - 3 * T_jerk1_ * time + pow(T_jerk1_, 2));
		else if (time >= T_acc_ - T_jerk1_ && time < T_acc_)
			q = start_pos_ + (lim_vel_ + start_vel_) * (T_acc_ / 2) - lim_vel_ * (T_acc_ - time) - min_jerk_ * (pow((T_acc_ - time), 3) / 6);
		// 匀速段
		else if (time >= T_acc_ && time < T_acc_ + T_const_)
			q = start_pos_ + (lim_vel_ + start_vel_) * (T_acc_ / 2) + lim_vel_ * (time - T_acc_);
		// 减速段
		else if (time >= T_acc_ + T_const_ && time < T - T_dec_ + T_jerk2_)
			q = end_pos_ - (lim_vel_ + end_vel_) * (T_dec_ / 2) + lim_vel_ * (time - T + T_dec_) - max_jerk_ * (pow(time - T + T_dec_, 3) / 6);
		else if (time >= T - T_dec_ + T_jerk2_ && time < T - T_jerk2_)
			q = end_pos_ - end_vel_ * (T - time) + (lim_deca_ / 6) * (3 * pow(T - time, 2) - 3 * T_jerk2_ * (T - time) + pow(T_jerk2_, 2));
		else if (time >= T - T_jerk2_ && time <= T)
			q = end_pos_ - end_vel_ * (T - time) - max_jerk_ * (pow(T - time, 3) / 6);
		else
			q = end_pos_;
		return sigma_ * q;
	}

	double VelocityProfileDoubles::Vel(double time) const
	{
		double T = Duration();
		double qd;
		if (time >= 0 && time < T_jerk1_)
			qd = start_vel_ + max_jerk_ * (pow(time, 2) / 2);
		else if (time >= T_jerk1_ && time < T_acc_ - T_jerk1_)
			qd = start_vel_ + lim_acca_ * (time - T_jerk1_ / 2);
		else if (time >= T_acc_ - T_jerk1_ && time < T_acc_)
			qd = lim_vel_ + min_jerk_ * (pow(T_acc_ - time, 2) / 2);
		// 匀速段
		else if (time >= T_acc_ && time < T_acc_ + T_const_)
			qd = lim_vel_;
		// 减速段
		else if (time >= T_acc_ + T_const_ && time < T - T_dec_ + T_jerk2_)
			qd = lim_vel_ - max_jerk_ * (pow(time - T + T_dec_, 2) / 2);
		else if (time >= T - T_dec_ + T_jerk2_ && time < T - T_jerk2_)
			qd = end_vel_ - lim_deca_ * (T - time - T_jerk2_ / 2);
		else if (time >= T - T_jerk2_ && time <= T)
			qd = end_vel_ + max_jerk_ * (pow(time - T, 2) / 2);
		else
			qd = end_vel_;
		return sigma_ * qd;
	}

	double VelocityProfileDoubles::Acc(double time) const
	{
		double qdd;
		double T = Duration();
		if (time >= 0 && time < T_jerk1_)
			qdd = max_jerk_ * time;
		else if (time >= T_jerk1_ && time < T_acc_ - T_jerk1_)
			qdd = lim_acca_;
		else if (time >= T_acc_ - T_jerk1_ && time < T_acc_)
			qdd = -min_jerk_ * (T_acc_ - time);
		// 匀速段
		else if (time >= T_acc_ && time < T_acc_ + T_const_)
			qdd = 0;
		// 减速段
		else if (time >= T_acc_ + T_const_ && time < T - T_dec_ + T_jerk2_)
			qdd = -max_jerk_ * (time - T + T_dec_);
		else if (time >= T - T_dec_ + T_jerk2_ && time < T - T_jerk2_)
			qdd = lim_deca_;
		else if (time >= T - T_jerk2_ && time <= T)
			qdd = -max_jerk_ * (T - time);
		else
			qdd = -max_jerk_ * (T - time);
		return sigma_ * qdd;
	}

	double VelocityProfileDoubles::Jerk(double time) const
	{
		double T = Duration();
		double qddd;
		if (time >= 0 && time < T_jerk1_)
			qddd = max_jerk_;
		else if (time >= T_jerk1_ && time < T_acc_ - T_jerk1_)
			qddd = 0;
		else if (time >= T_acc_ - T_jerk1_ && time < T_acc_)
			qddd = min_jerk_;
		// 匀速段
		else if (time >= T_acc_ && time < T_acc_ + T_const_)
			qddd = 0;
		// 减速段
		else if (time >= T_acc_ + T_const_ && time < T - T_dec_ + T_jerk2_)
			qddd = -max_jerk_;
		else if (time >= T - T_dec_ + T_jerk2_ && time < T - T_jerk2_)
			qddd = 0;
		else if (time >= T - T_jerk2_ && time <= T)
			qddd = max_jerk_;
		else
			qddd = max_jerk_;
		return sigma_ * qddd;
	}

	KDL::VelocityProfile *VelocityProfileDoubles::Clone() const
	{
		VelocityProfileDoubles *trap = new VelocityProfileDoubles(max_vel_, max_acc_, max_jerk_);
		trap->start_pos_ = start_pos_;
		trap->end_pos_ = end_pos_;
		trap->start_vel_ = start_vel_;
		trap->end_vel_ = end_vel_;
		trap->T_acc_ = T_acc_;
		trap->T_const_ = T_const_;
		trap->T_dec_ = T_dec_;
		trap->T_jerk1_ = T_jerk1_;
		trap->T_jerk2_ = T_jerk2_;
		trap->lim_vel_ = lim_vel_;
		trap->lim_acca_ = lim_acca_;
		trap->lim_deca_ = lim_deca_;
		trap->sigma_ = sigma_;
		return trap;
	}

	void VelocityProfileDoubles::Write(std::ostream &os) const
	{
		os << *this;
	}

	std::ostream &operator<<(std::ostream &os, const VelocityProfileDoubles &p)
	{
		os << "Asymmetric Trapezoid " << std::endl
		   << "maximal velocity: " << p.max_vel_ << std::endl
		   << "maximal acceleration: " << p.max_acc_ << std::endl
		   << "maximal jerk: " << p.max_jerk_ << std::endl
		   << "start position: " << p.start_pos_ << std::endl
		   << "end position: " << p.end_pos_ << std::endl
		   << "start velocity: " << p.start_vel_ << std::endl
		   << "end velocity: " << p.end_vel_ << std::endl
		   << "T_jerk1: " << p.T_jerk1_ << std::endl
		   << "T_jerk2: " << p.T_jerk2_ << std::endl
		   << "Tacc: " << p.T_acc_ << std::endl
		   << "Tconst: " << p.T_const_ << std::endl
		   << "Tdec: " << p.T_dec_ << std::endl
		   << "Vlim: " << p.lim_vel_ << std::endl
		   << "Alima: " << p.lim_acca_ << std::endl
		   << "Alimd: " << p.lim_deca_ << std::endl;
		return os;
	}

	bool VelocityProfileDoubles::operator==(const VelocityProfileDoubles &other) const
	{
		return false;
	}

	VelocityProfileDoubles::~VelocityProfileDoubles()
	{
	}

	void VelocityProfileDoubles::setEmptyProfile()
	{
		T_acc_ = 0;
		T_const_ = 0;
		T_dec_ = 0;
		T_jerk1_ = 0;
		T_jerk2_ = 0;
		lim_vel_ = 0;
		lim_acca_ = 0;
		lim_deca_ = 0;
	}

	void VelocityProfileDoubles::transformparam()
	{
		sigma_ = end_pos_ - start_pos_ > 0 ? 1 : -1;
		start_pos_ = sigma_ * start_pos_;
		end_pos_ = sigma_ * end_pos_;
		start_vel_ = sigma_ * start_vel_;
		end_vel_ = sigma_ * end_vel_;
		max_vel_ = ((sigma_ + 1) / 2) * max_vel_ + ((sigma_ - 1) / 2) * min_vel_;
		min_vel_ = ((sigma_ + 1) / 2) * min_vel_ + ((sigma_ - 1) / 2) * max_vel_;
		max_acc_ = ((sigma_ + 1) / 2) * max_acc_ + ((sigma_ - 1) / 2) * min_acc_;
		min_acc_ = ((sigma_ + 1) / 2) * min_acc_ + ((sigma_ - 1) / 2) * max_acc_;
		max_jerk_ = ((sigma_ + 1) / 2) * max_jerk_ + ((sigma_ - 1) / 2) * min_jerk_;
		min_jerk_ = ((sigma_ + 1) / 2) * min_jerk_ + ((sigma_ - 1) / 2) * max_jerk_;
	}

} // namespace pilz_industrial_motion_planner
