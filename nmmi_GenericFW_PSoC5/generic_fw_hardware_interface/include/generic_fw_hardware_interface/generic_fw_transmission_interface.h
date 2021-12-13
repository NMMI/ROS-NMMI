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

#ifndef GENERIC_FW_TRANSMISSION_H
#define GENERIC_FW_TRANSMISSION_H

#include <transmission_interface/transmission.h>
#include <control_toolbox/filters.h>

namespace generic_fw_transmission_interface {
/**
 * The \em Generic FW Transmission interface implements the specific \p transmission_interface::Transmission to
 * convert from \em motors state to its equivalent joint state representation, and vice versa.
 * \sa qb_device_transmission_interface::qbDeviceTransmissionResources
 */
class GenericFWVirtualTransmission : public transmission_interface::Transmission {
 public:
  /**
   * Build the \em Generic FW transmission transparent.
   */
  GenericFWVirtualTransmission()
      : Transmission() {}

  inline void actuatorToJointEffort(const transmission_interface::ActuatorData& actuator, transmission_interface::JointData& joint) {
    ROS_ASSERT(numActuators() == actuator.effort.size() && numJoints() == joint.effort.size());
    ROS_ASSERT(actuator.effort[0] && joint.effort[0]);

    *joint.effort[0] = *actuator.effort[0];
  }

  inline void actuatorToJointVelocity(const transmission_interface::ActuatorData& actuator, transmission_interface::JointData& joint) {
    ROS_ASSERT(numActuators() == actuator.velocity.size() && numJoints() == joint.velocity.size());
    ROS_ASSERT(actuator.velocity[0] && joint.velocity[0]);

    // *actuator.velocity[0] is the current measured velocity in [ticks/s] while *joint.velocity[0] is the previous step velocity in [percent/s]
    *joint.velocity[0] = filters::exponentialSmoothing(*actuator.velocity[0], *joint.velocity[0], 1.0);
  }

  inline void actuatorToJointPosition(const transmission_interface::ActuatorData& actuator, transmission_interface::JointData& joint) {
    ROS_ASSERT(numActuators() == actuator.position.size() && numJoints() == joint.position.size());
    ROS_ASSERT(actuator.position[0] && joint.position[0]);

    *joint.position[0] = *actuator.position[0];
  }

  inline void jointToActuatorEffort(const transmission_interface::JointData& joint, transmission_interface::ActuatorData& actuator) {
    ROS_ASSERT(numActuators() == actuator.effort.size() && numJoints() == joint.effort.size());
    ROS_ASSERT(actuator.effort[0] && joint.effort[0]);

    // the qbhand cannot be controlled in current, but this could help in the near future
    *actuator.effort[0] = *joint.effort[0];
  }

  inline void jointToActuatorVelocity(const transmission_interface::JointData& joint, transmission_interface::ActuatorData& actuator) {
    ROS_ASSERT(numActuators() == actuator.velocity.size() && numJoints() == joint.velocity.size());
    ROS_ASSERT(actuator.velocity[0] && joint.velocity[0]);

    // the qbhand cannot be controlled in velocity
    *actuator.velocity[0] = 0.0;
  }

  inline void jointToActuatorPosition(const transmission_interface::JointData& joint, transmission_interface::ActuatorData& actuator) {
    ROS_ASSERT(numActuators() == actuator.position.size() && numJoints() == joint.position.size());
    ROS_ASSERT(actuator.position[0] && joint.position[0]);

    *actuator.position[0] = *joint.position[0];
  }

  /**
   * \return The number of actuators of this transmission.
   */
  inline std::size_t numActuators() const { return 1; }

  /**
   * \return The number of joints of this transmission.
   */
  inline std::size_t numJoints() const { return 1; }


 private:

};
}  // namespace generic_fw_transmission_interface

#endif // GENERIC_FW_TRANSMISSION_H