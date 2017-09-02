#ifndef LWR_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_H
#define LWR_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>



namespace lwr_controllers
{
	class ComputedTorqueController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:

		ComputedTorqueController();
		~ComputedTorqueController();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg);

	private:

		ros::Subscriber sub_posture_;
		ros::Subscriber sub_gains_;
        ros::Publisher  pub_all_data_;
        std_msgs::Float64MultiArray allDataMsg;
        
        KDL::JntArray cmd_states_, init_states_;
		int cmd_flag_;	// discriminate if a user command arrived
		double lambda;	// flattening coefficient of tanh
		int step_;		// step used in tanh for reaching gradually the desired posture

        bool enbRecordAllData_;
        double timer_temp_, timer02ms_, timer10ms_;
        double spline_time_;

		KDL::JntArray tau_cmd_;
		KDL::JntSpaceInertiaMatrix M_; //Inertia matrix
		KDL::JntArray C_, G_;	//Coriolis and Gravitational matrices
        KDL::JntArray Kp_, Kv_;	//Position and Velocity gains

		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;

        void recordAllData(const ros::Time &time, const ros::Duration &period);

	};
}

#endif
