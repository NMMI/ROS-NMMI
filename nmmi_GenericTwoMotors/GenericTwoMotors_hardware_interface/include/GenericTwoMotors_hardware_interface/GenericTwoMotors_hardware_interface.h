/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2019-2021, Centro "E. Piaggio"
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

#ifndef GENERICTWOMOTORS_HARDWARE_INTERFACE_H
#define GENERICTWOMOTORS_HARDWARE_INTERFACE_H

// ROS libraries
#include <pluginlib/class_list_macros.hpp>

// internal libraries
#include <qb_device_hardware_interface/qb_device_hardware_interface.h>
#include <GenericTwoMotors_hardware_interface/GenericTwoMotors_transmission_interface.h>
#include <nmmi_srvs/nmmi_srvs.h>

namespace GenericTwoMotors_hardware_interface {
/**
 * The \em GenericTwoMotors HardWare interface implements the specific structures to manage the communication with a
 * \em Generic Two Motors device. It exploits the features provided by the base device-independent hardware interface and the
 * specific transmission interface.
 * \sa qb_device_hardware_interface::qbDeviceHW, GenericTwoMotors_transmission_interface::GenericTwoMotorsVirtualTransmission
 */
class GenericTwoMotorsHW : public qb_device_hardware_interface::qbDeviceHW {
 public:
  /**
   * Initialize the \p qb_device_hardware_interface::qbDeviceHW with the specific transmission interface and actuator
   * and joint names.
   * \sa GenericTwoMotors_transmission_interface::GenericTwoMotorsVirtualTransmission
   */
  GenericTwoMotorsHW();

  /**
   * Do nothing.
   */
  virtual ~GenericTwoMotorsHW() override;

  /**
   * \return The vector of controller joint names.
   */
  std::vector<std::string> getJoints() override;

  /**
   * Call the base method and nothing more.
   * \param root_nh A NodeHandle in the root of the caller namespace.
   * \param robot_hw_nh A NodeHandle in the namespace from which the RobotHW should read its configuration.
   * \returns \p true on success.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) override;

  /**
   * Call the base method and nothing more.
   * \param time The current time.
   * \param period The time passed since the last call to this method, i.e. the control period.
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /**
   * Call the base method and nothing more.
   * \param time The current time.
   * \param period The time passed since the last call to this method, i.e. the control period.
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

protected:

  bool  is_reliable_;
  int   control_mode_;
  
  /**
   * Call the service to initialize the device with parameters from the Communication Handler and wait for the response.
   * If the initializaation succeed, store the device parameters received, e.g. \p n_imu_. 
   * \return \p 0 on success.
   * \sa waitForInitialization()
   */
  int initializeDevice();

  /**
   * Subscribe to all the services advertised by the Communication Handler and wait until all the services are
   * properly advertised.
   * \sa resetServicesAndWait(), waitForServices()
   */
  void initializeGenericTwoMotorsServicesAndWait();

  /**
   * Re-subscribe to all the services advertised by the Communication Handler and wait until all the services are
   * properly advertised. Then re-initialize the device parameters (it is assumed that the this method can be called
   * only if the device was previously initialized), unless otherwise specified.
   * \param reinitialize_device If \p true, i.e. by default, reinitialize the device.
   * \sa initializeServicesAndWait(), waitForInitialization(), waitForServices()
   */
  void resetServicesAndWait(const bool &reinitialize_device = true);

  /**
   * Wait until the device is initialized.
   * \sa initializeDevice()
   */
  void waitForInitialization();
  
    /**
   * Wait until all the services advertised by the Communication Handler are active, then reinitialize the device to
   * avoid disconnection problems.
   * \sa initializeServicesAndWait(), resetServicesAndWait()
   */
  void waitForServices();

private:
    /**
   * Add the namespace prefix stored in the private \p namespace_ to all the elements of the given vector.
   * \param vector Any vector of strings.
   * \return The namespaced vector.
   */
  std::vector<std::string> addNamespacePrefix(const std::vector<std::string> &vector);

  double max_motor_limits_;
  double min_motor_limits_;

};
typedef std::shared_ptr<GenericTwoMotorsHW> GenericTwoMotorsHWPtr;
}  // namespace GenericTwoMotors_hardware_interface

#endif // GENERICTWOMOTRS_HARDWARE_INTERFACE_H