/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2019, Centro "E. Piaggio"
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 *  following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *    following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NMMI_COMMUNICATION_HANDLER_H
#define NMMI_COMMUNICATION_HANDLER_H

// Standard libraries
#include <mutex>
#include <regex>

// ROS libraries
#include <ros/ros.h>

// internal libraries
#include <qb_device_driver/qb_device_communication_handler.h>
#include <nmmi_driver/nmmi_driver.h>
#include <nmmi_srvs/nmmi_srvs.h>

namespace nmmi_communication_handler {
/**
 * The Communication Handler class is aimed to instantiate a ROS node which provides several ROS services to
 * communicate with one - or many - qbrobotics devices connected to the ROS ecosystem.
 *
 * Each hardware interface which manage a single qbrobotics device must first request the initialization of its ID to
 * the Communication Handler and then interact with it through specific ROS services - blocking by nature. The
 * Communication Handler essentially manage the shared serial port resources and lets all the hardware interfaces to
 * get access to them. This could seem a bottleneck in the architecture, but the real bottleneck is the shared resource
 * itself; moreover this structure is also necessary since the multi-process nature of ROS which does not easily allow
 * to share common resources among distinct processes.
 * Actually the implementation of qbrobotics classes could be reshaped to exploit the multi-threading approach, e.g.
 * using [nodelet](http://wiki.ros.org/nodelet) or similar plugins each device node could handle the communication
 * process by itself, without the intermediary Communication Handler. The advantage of such a restyle in term of
 * communication speed does not worth the effort though.
 */
class nmmiCommunicationHandler : public qb_device_communication_handler::qbDeviceCommunicationHandler{
 public:
  /**
   * Wait until at least one device is connected and then initialize the Communication Handler.
   * \sa getSerialPortsAndDevices()
   */
  nmmiCommunicationHandler();
  
  /**
   * Close all the still open serial ports.
   * \sa close()
   */
  virtual ~nmmiCommunicationHandler();

 protected:
    
  /**
   * Scan for all the serial ports of type \p /dev/ttyUSB* detected in the system, initialize their mutex protector
   * (each serial port connected to the system has to be accessed in a mutually exclusive fashion), and retrieve all
   * the NMMI devices connected to them. For each device, store its ID in the private map \p connected_devices_,
   * i.e. insert a pair [\p device_id, \p serial_port]. The map \p connected_devices_ is constructed from scratch at
   * each call.
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \return the number of connected devices.
   * \sa qb_device_driver::qbDeviceAPI::getDeviceIds(), qb_device_driver::qbDeviceAPI::getSerialPorts(), isInConnectedSet(), open()
   */
  //virtual int getSerialPortsAndDevices(const int &max_repeats);

  int getADCRawvalues(const int &id, const int &max_repeats, const uint8_t &used_adc_channels, std::vector<int16_t> &adc);

  bool getADCRawvaluesCallback(nmmi_srvs::GetADCRawValuesRequest &request, nmmi_srvs::GetADCRawValuesResponse &response);

  bool getADCconfCallback(nmmi_srvs::GetADCMapRequest &request, nmmi_srvs::GetADCMapResponse &response);

  bool getControlModeCallback(nmmi_srvs::GetControlModeRequest &request, nmmi_srvs::GetControlModeResponse &response);

  int getEncoderRawvalues(const int &id, const int &max_repeats, const uint8_t &num_encoder_conf_total, const bool old_board, std::vector<uint16_t> &enc);

  bool getEncoderRawvaluesCallback(nmmi_srvs::GetEncoderRawValuesRequest &request, nmmi_srvs::GetEncoderRawValuesResponse &response);

  bool getEncoderconfCallback(nmmi_srvs::GetEncoderMapRequest &request, nmmi_srvs::GetEncoderMapResponse &response);

  /**
   * Retrieve the IMU values of the given device.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param n_imu The number of IMU of device ID.
   * \param[out] acc The three-element per n_imu device accelerometers vector, expressed in \em g
   * \param[out] gyro The three-element per n_imu device gyroscopes vector, expressed in \em deg/s
   * \param[out] mag The three-element per n_imu device magnetometers vector, expressed in \em uT
   * \param[out] quat The four-element per n_imu device quaternion vector
   * \param[out] temp The one-element per n_imu device temperature vector, expressed in \em °C
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \return The number of failure reads between \p 0 and \p max_repeats.
   * \sa nmmi_driver::nmmiAPI::getIMUValues(), getIMUValuesCallback()
   */
  int getIMUvalues(const int &id, const int &max_repeats, const nmmi_msgs::ImuTable &imu_table, const bool &custom_read_timeout, std::vector<float> &acc, std::vector<float> &gyro, std::vector<float> &mag, std::vector<float> &quat, std::vector<float> &temp);

  bool getIMUvaluesCallback(nmmi_srvs::GetIMUValuesRequest &request, nmmi_srvs::GetIMUValuesResponse &response);

  bool getIMUparamCallback(nmmi_srvs::GetIMUParamRequest &request, nmmi_srvs::GetIMUParamResponse &response);

  /**
   * Retrieve some of the parameters from the given device.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param[out] limits The vector of motor position limits expressed in \em ticks: two values for each motor,
   * respectively [\p lower_limit, \p upper_limit].
   * \param[out] resolutions The vector of encoder resolutions, each in range [\p 0, \p 8]: one value for each encoder
   * (\b note: the \em qbmove has also the shaft encoder even if it is not actuated). The word "resolution" could be
   * misunderstood: taken the resolution \p r, \f$2^r\f$ is the number of turns of the wire inside the device mechanics.
   * It is used essentially to convert the measured position of the motors in \em ticks to \em radians or \em degrees.
   * \return \p 0 on success.
   * \sa qb_device_driver::qbDeviceAPI::getParameters(), getInfo(), initializeCallback()
   */
  int getParameters(const int &id, std::vector<int> &limits, std::vector<int> &resolutions);

  /**
   * Initialize the device node requesting the service to the Communication Handler if the relative physical device is
   * connected through any serial port to the system (can re-scan the serial resources if specified in the request).
   * If the device is found, retrieve some of its parameter and activate its motors, if requested.
   * \param request The request of the given service (see qb_device_srvs::InitializeDevice for details).
   * \param response The response of the given service (see qb_device_srvs::InitializeDevice for details).
   * \return \p true if the call succeed (actually \p response.success may be false).
   * \sa getSerialPortsAndDevices()
   */
  bool initializeNMMIDeviceCallback(qb_device_srvs::InitializeDeviceRequest &request, qb_device_srvs::InitializeDeviceResponse &response);
  
  /**
   * Check whether the the device specified by the given ID is connected through the serial port.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \return The number of failure reads between \p 0 and \p max_repeats.
   * \sa qb_device_driver::qbDeviceAPI::getStatus()
   */
  int isConnected(const int &id, const int &max_repeats);

  /**
   * Send the reference command to the motors of the device relative to the node requesting the service.
   * \param request The request of the given service (see qb_device_srvs::SetCommands for details).
   * \param response The response of the given service (see qb_device_srvs::SetCommands for details).
   * \return \p true if the call succeed (actually \p response.success may be false).
   * \sa setCommandsAndWait(), setCommandsAsync()
   */
  bool setCommandsCallback(qb_device_srvs::SetCommandsRequest &request, qb_device_srvs::SetCommandsResponse &response);
 
 private:
  ros::NodeHandle node_handle_;
  nmmi_driver::nmmiAPIPtr nmmi_api_;
  bool set_commands_async;

  ros::ServiceServer get_adc_raw_values_;
  ros::ServiceServer get_adc_conf_;
  ros::ServiceServer get_control_mode_;
  ros::ServiceServer get_encoder_raw_values_;
  ros::ServiceServer get_encoder_conf_;
  ros::ServiceServer get_imu_values_;
  ros::ServiceServer get_imu_param_;
  ros::ServiceServer initialize_nmmi_device_;
  ros::ServiceServer set_motor_commands_;

  /**
   * Check whether the reading failures are in the given range.
   * \param failures The current number of communication failures per serial resource reading.
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \return \p true if the failures are less than the given threshold.
   */
  inline bool isReliable(int const &failures, int const &max_repeats) { return failures >= 0 && failures <= max_repeats; }
};
}  // namespace nmmi_communication_handler

#endif // NMMI_COMMUNICATION_HANDLER_H