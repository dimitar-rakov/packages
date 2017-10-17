#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <boost/thread.hpp>
#include "obstacle_detector/obstacle_detector.h"


namespace obstacle_detector
{

  class ObstacleDetectorNodelet : public nodelet::Nodelet
  {
  public:
    ObstacleDetectorNodelet(){}
    ~ObstacleDetectorNodelet(){}

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
      obstacle_detector_ptr_.reset(new ObstacleDetector());

      if (obstacle_detector_ptr_->init(nh)){
        NODELET_INFO_STREAM("Nodelet initialised. [" << name << "]");
        run_thread_ptr_.reset(new boost::thread(boost::bind(&ObstacleDetectorNodelet::run, this)));
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
          obstacle_detector_ptr_->update(ros::Time::now(), work_period);
          msr_period = ros::Time::now() - last_time;
          if (msr_period< des_period)
            (des_period - msr_period).sleep();
          else
            NODELET_WARN_COND(des_perion_sec >0, "Desired period exceed. Desired period is %lf, last period is %lf", des_period.toSec() ,msr_period.toSec());
          obstacle_detector_ptr_->publish();
          last_time = ros::Time::now();
          work_period = last_time - init_time ;
      }
    }

  private:
    boost::shared_ptr<ObstacleDetector> obstacle_detector_ptr_;
    boost::shared_ptr<boost::thread> run_thread_ptr_;
  };

  } // namespace obstacle_detector

  PLUGINLIB_EXPORT_CLASS(obstacle_detector::ObstacleDetectorNodelet, nodelet::Nodelet);
