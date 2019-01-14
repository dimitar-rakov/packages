#ifndef LWR_HW_FRIL_H
#define LWR_HW_FRIL_H

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

// FRIL remote hooks
#include <FastResearchInterface.h>

#define NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK   2000

namespace lwr_hw
{

class LWRHWFRIL : public LWRHW
{

public:

  LWRHWFRIL();
  ~LWRHWFRIL() final;

  /**
   * @brief init Initialize a new lwr_hw
   * @param nh Node handle
   * @return True by success, false otherwise
   */
  bool init(ros::NodeHandle &nh) final;

  /**
   * @brief write Write command values for joint handles
   * @param time Current ros time
   * @param period Last period used by integration and differentiation
   */
  void read(ros::Time time, ros::Duration period) final;

  /**
   * @brief write Write command values for joint handles
   * @param time Current ros time
   * @param period Last period used by integration and differentiation
   */
  void write(ros::Time time, ros::Duration period) final;

  /**
   * @brief doSwitch Switch on controller from start_list and switch off controller from stop_list
   * @param start_list List with controller to be started
   * @param stop_list List with controller to be stopped
   */
  void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                const std::list<hardware_interface::ControllerInfo> &stop_list);

  /**
   * @brief stop Stop FRIL communication
   */
  void stop();

  void set_mode();

  /**
   * @brief setInitFile Set configuration file
   * @param init_file Pat to configuration file
   */
  void setInitFile(std::string init_file);

private:
  /// Path to configuration file
  std::string init_file_;

  /// Indicate whatever the configuration file was set
  bool file_set_;// = false;

  /// low-level interface
  std::unique_ptr <FastResearchInterface> device_;

  /// Command status
  int result_status_;// = 0;
};

}


#endif // LWR_HW_FRIL_H
