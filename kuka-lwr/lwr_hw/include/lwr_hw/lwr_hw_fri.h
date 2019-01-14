#ifndef LWR_HW_FRI_H
#define LWR_HW_FRI_H

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

// FRI remote hooks
#include "fri/friudp.h"
#include "fri/friremote.h"
#include <thread>
#include <pthread.h>
#include <mutex>

// ToDo: add timeouts to all sync-while's to KRL since the UDP connection might be lost and we will know

namespace lwr_hw
{

class LWRHWFRI : public LWRHW
{

public:

  /**
   * @brief LWRHWFRI Default constructor
   */
  LWRHWFRI();
  /**
   * @brief ~LWRHW Default constructor
   */
  ~LWRHWFRI() final;

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
   * @brief setPort Set TCP/IP port used for FRI communication
   * @param port Port number
   */
  void setPort(int port);

  /**
   * @brief setIPSet  Set IP of the remote host used for FRI communication
   * @param hint_to_remote_host IP of remote host
   */
  void setIP(std::string hint_to_remote_host);

  /**
   * @brief KRCCommStart Communication thread function
   * @return
   */
  bool KRCCommStart();

  void setControlReady();

private:

  /// TCP/IP port used for FRI communication
  int port_;

  /// Indicate whatever the TCP/IP port was set
  bool port_set_;

  /// Indicate whatever the IP was set
  bool ip_set_;

  /// Stop communication flag
  bool stopKRCComm_ ;

  /// Indicate whatever control loop was started
  bool control_started_;

  /// Indicate whatever the communication with the robot is running
  bool comm_is_alive;

  /// IP of remote host
  std::string hint_to_remote_host_;

  /// Last timestamp from FRI
  float fri_last_timestamp;

  /// low-level interface
  std::unique_ptr <friRemote> device_;

  /// FRI last quality values
  FRI_QUALITY fri_last_quality_;

  /// FRI last control scheme
  FRI_CTRL fri_last_ctrl_scheme_;

  /// Thread for communication
  std::unique_ptr <std::thread> KRCCommThread_;

  /// Mutex to safety data handling of read/write data
  std::mutex iomutex;

  /**
   * @brief KRCCommThreadCallback KRC communication callback function
   */
  void KRCCommThreadCallback();

  /**
   * @brief startFRI Start FRI communication
   */
  void startFRI();

  /**
   * @brief stopFRI Stop FRI communication
   */
  void stopFRI();

};

}
#endif // LWR_HW_FRI_H
