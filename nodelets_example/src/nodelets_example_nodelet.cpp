#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <boost/thread.hpp>
#include "nodelets_example/nodelets_example.h"


namespace examples
{

  class NodeletsExampleNodelet : public nodelet::Nodelet
  {
  public:
    NodeletsExampleNodelet(){}
    ~NodeletsExampleNodelet(){}

    /**
     * @brief Initialise the nodelet
     *
     * This function is called, when the nodelet manager loads the nodelet.
     */
    virtual void onInit()
    {
      ros::NodeHandle nh = this->getPrivateNodeHandle();

      // resolve node(let) name
      std::string name = nh.getUnresolvedNamespace();
      int pos = name.find_last_of('/');
      name = name.substr(pos + 1);

      NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
      nodelets_example_ptr_.reset(new NodeletsExample());

      if (nodelets_example_ptr_->init(nh)){
        NODELET_INFO_STREAM("Nodelet initialised. [" << name << "]");
        run_thread_ptr_.reset(new boost::thread(boost::bind(&NodeletsExampleNodelet::run, this)));
      }
      else
        NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << name << "]");
    }

    /** @brief run thread loop. */
    void run() {
      ros::Time last_time = ros::Time::now(), init_time = ros::Time::now();
      double des_perion_sec;
      if (!getPrivateNodeHandle().getParam("des_period_sec", des_perion_sec))
        ROS_WARN("Parameter des_period_sec was not found. Default value is used: %lf", des_perion_sec);
      ros::Duration des_period(des_perion_sec), msr_period(des_perion_sec), work_period(des_perion_sec);;

      while(ros::ok())
      {
          init_time = ros::Time::now();
          nodelets_example_ptr_->update(ros::Time::now(), work_period);
          msr_period = ros::Time::now() - last_time;
          if (msr_period< des_period)
            (des_period - msr_period).sleep();
          else
            NODELET_WARN_COND(des_perion_sec >0, "Desired period exceed. Desired period is %lf, last period is %lf", des_period.toSec() ,msr_period.toSec());
          last_time = ros::Time::now();
          work_period = last_time - init_time ;
      }
    }

  private:
    boost::shared_ptr<NodeletsExample> nodelets_example_ptr_;
    boost::shared_ptr<boost::thread> run_thread_ptr_;
  };

  } // namespace examples

  PLUGINLIB_EXPORT_CLASS(examples::NodeletsExampleNodelet, nodelet::Nodelet);
