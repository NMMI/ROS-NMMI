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

#ifndef GENERICTWOMOTORS_TRANSMISSION_H
#define GENERICTWOMOTORS_TRANSMISSION_H

#include <transmission_interface/transmission.h>
#include <control_toolbox/filters.h>

namespace GenericTwoMotors_transmission_interface {
/**
 * The GenericTwoMotors Transmission interface implements the specific \p transmission_interface::Transmission to
 * convert from \em GenericTwoMotors motors state to its equivalent joint state representation, and vice versa.
 * \sa qb_device_transmission_interface::qbDeviceTransmissionResources
 */
class GenericTwoMotorsTransmission : public transmission_interface::Transmission {
 public:
  /**
   * Build the \em GenericTwoMotors transmission with default velocity and effort scale factors (respectively \p 0.2 and \p 0.001).
   * The position scale factor is based on the "motor position limit" (expressed in \em ticks) which is the maximum
   * closure value, and follows the formula \f$f_{pos} = \frac{1}{closure\_limit}\f$. The default \p closure_limit is
   * set to 16000 ticks, but it can be modified during the execution with the proper method.
   */
  GenericTwoMotorsTransmission()
      : GenericTwoMotorsTransmission(std::vector<double>(3, 1.0), 1.0, 0.001) {}

  /**
   * Build the \em GenericTwoMotors transmission with the given scale factors.
   * \param position_factor Motor position \em ticks to closure percent value [\p 0, \p 1] scale factor.
   * \param velocity_factor Exponential filter smoothing factor.
   * \param effort_factor Motor current \em mA to joint effort \em A scale factor.
   */
  GenericTwoMotorsTransmission(const std::vector<double> &position_factor, const double &velocity_factor, const double &effort_factor)
      : Transmission(),
        position_factor_(position_factor),
        velocity_factor_(velocity_factor),
        effort_factor_(effort_factor) {}

  /**
   * Transform \em effort variables from actuator to joint space.
   * \param actuator Actuator-space variables.
   * \param[out] joint Joint-space variables.
   * \pre Actuator and joint effort vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void actuatorToJointEffort(const transmission_interface::ActuatorData &actuator, transmission_interface::JointData &joint) {
    ROS_ASSERT(numActuators() == actuator.effort.size() && numJoints() == joint.effort.size());
    ROS_ASSERT(actuator.effort[0] && actuator.effort[1] && actuator.effort[2] && joint.effort[0] && joint.effort[1] && joint.effort[2] && joint.effort[3]);

    *joint.effort[0] = *actuator.effort[0] * effort_factor_;  // motor_1_joint [A]
    *joint.effort[1] = *actuator.effort[1] * effort_factor_;  // motor_2_joint [A]
    *joint.effort[2] = 0.0;  // meaningless
    *joint.effort[3] = 0.0;  // meaningless
  }

  /**
   * Transform \em velocity variables from actuator to joint space.
   * \param actuator Actuator-space variables.
   * \param[out] joint Joint-space variables.
   * \pre Actuator and joint velocity vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void actuatorToJointVelocity(const transmission_interface::ActuatorData &actuator, transmission_interface::JointData &joint) {
    ROS_ASSERT(numActuators() == actuator.velocity.size() && numJoints() == joint.velocity.size());
    ROS_ASSERT(actuator.velocity[0] && actuator.velocity[1] && actuator.velocity[2] && joint.velocity[0] && joint.velocity[1] && joint.velocity[2] && joint.velocity[3]);

    // note: be aware that this method _misuses_ actuator.velocity to store the current measured velocity
    //   - *actuator.velocity[i] is the current measured velocity of the motor i in [ticks/s]
    //   - *joint.velocity[i] is the previous step velocity in [radians/s] or [percent/s]
    *joint.velocity[0] = filters::exponentialSmoothing(*actuator.velocity[0] * position_factor_.at(0), *joint.velocity[0], velocity_factor_);  // motor_1_joint [radians/s]
    *joint.velocity[1] = filters::exponentialSmoothing(*actuator.velocity[1] * position_factor_.at(1), *joint.velocity[1], velocity_factor_);  // motor_2_joint [radians/s]
    *joint.velocity[2] = filters::exponentialSmoothing(*actuator.velocity[2] * position_factor_.at(2), *joint.velocity[2], velocity_factor_);  // shaft_joint [radians/s]
    *joint.velocity[3] = (*joint.velocity[0] - *joint.velocity[1]) / 2;  // stiffness preset [percent/s]
  }

  /**
   * Transform \em position variables from actuator to joint space.
   * \param actuator Actuator-space variables.
   * \param[out] joint Joint-space variables.
   * \pre Actuator and joint position vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void actuatorToJointPosition(const transmission_interface::ActuatorData &actuator, transmission_interface::JointData &joint) {
    ROS_ASSERT(numActuators() == actuator.position.size() && numJoints() == joint.position.size());
    ROS_ASSERT(actuator.position[0] && actuator.position[1] && actuator.position[2] && joint.position[0] && joint.position[1] && joint.position[2] && joint.position[3]);

    *joint.position[0] = *actuator.position[0] * position_factor_.at(0);  // motor_1_joint [radians]
    *joint.position[1] = *actuator.position[1] * position_factor_.at(1);  // motor_2_joint [radians]
    *joint.position[2] = *actuator.position[2] * position_factor_.at(2);  // shaft_joint [radians]
    *joint.position[3] = 0;
  }

  /**
   * \return The current position scale factor.
   */
  inline const std::vector<double> getPositionFactor() const { return position_factor_; }

  /**
   * \return The current velocity scale factor.
   */
  inline const double& getVelocityFactor() const { return velocity_factor_; }

  /**
   * \return The current effort scale factor.
   */
  inline const double& getEffortFactor() const { return effort_factor_; }

  /**
   * Transform \em effort variables from joint to actuator space.
   * \param joint Joint-space variables.
   * \param[out] actuator Actuator-space variables.
   * \pre Actuator and joint effort vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void jointToActuatorEffort(const transmission_interface::JointData &joint, transmission_interface::ActuatorData &actuator) {
    ROS_ASSERT(numActuators() == actuator.effort.size() && numJoints() == joint.effort.size());
    ROS_ASSERT(actuator.effort[0] && actuator.effort[1] && actuator.effort[2] && joint.effort[0] && joint.effort[1] && joint.effort[2] && joint.effort[3]);

    // direct motors control
    // not used yet, but this could help in the near future
    *actuator.effort[0] = *joint.effort[0] / effort_factor_;
    *actuator.effort[1] = *joint.effort[1] / effort_factor_;
    *actuator.effort[2] = 0.0;
  
  }

  /**
   * Transform \em velocity variables from joint to actuator space.
   * \param joint Joint-space variables.
   * \param[out] actuator Actuator-space variables.
   * \pre Actuator and joint velocity vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void jointToActuatorVelocity(const transmission_interface::JointData &joint, transmission_interface::ActuatorData &actuator) {
    ROS_ASSERT(numActuators() == actuator.velocity.size() && numJoints() == joint.velocity.size());
    ROS_ASSERT(actuator.velocity[0] && actuator.velocity[1] && actuator.velocity[2] && joint.velocity[0] && joint.velocity[1] && joint.velocity[2] && joint.velocity[3]);

    // the qbmove cannot be controlled in velocity
    *actuator.velocity[0] = 0.0;
    *actuator.velocity[1] = 0.0;
    *actuator.velocity[2] = 0.0;
  }

  /**
   * Transform \em position variables from joint to actuator space.
   * \param joint Joint-space variables.
   * \param[out] actuator Actuator-space variables.
   * \pre Actuator and joint position vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void jointToActuatorPosition(const transmission_interface::JointData &joint, transmission_interface::ActuatorData &actuator) {
    ROS_ASSERT(numActuators() == actuator.position.size() && numJoints() == joint.position.size());
    ROS_ASSERT(actuator.position[0] && actuator.position[1] && actuator.position[2] && joint.position[0] && joint.position[1] && joint.position[2] && joint.position[3]);

    *actuator.position[0] = *joint.position[0] / position_factor_.at(0);  // motor_1 = motor_1_command [ticks]
    *actuator.position[1] = *joint.position[1] / position_factor_.at(1);  // motor_2 = motor_2_command [ticks]
    *actuator.position[2] = (*joint.position[0] / position_factor_.at(0) + *joint.position[1] / position_factor_.at(1))/2;
  
  }

  /**
   * \return The number of actuators of this transmission, i.e always 2 for the \em qbmove.
   */
  inline std::size_t numActuators() const { return 3; }

  /**
   * \return The number of joints of this transmission, i.e always 4 for the \em qbmove.
   */
  inline std::size_t numJoints() const { return 4; }

  /**
   * Set the position scale factor.
   * \param position_factor Motor position \em ticks to closure percent value [\p 0, \p 1] scale factor.
   */
  inline void setPositionFactor(const std::vector<int> &encoder_resolutions) {
    ROS_ASSERT(numActuators() == position_factor_.size() && numActuators() == encoder_resolutions.size());
    ROS_ASSERT(encoder_resolutions[0] == encoder_resolutions[1]);
    for (int i=0; i< position_factor_.size(); i++) {
      position_factor_.at(i) = 1.0; //M_PI/std::pow(2, 15-encoder_resolutions.at(i));
    }
  }

 private:
  std::vector<double> position_factor_;  
  double velocity_factor_;
  double effort_factor_;
};
}  // namespace GenericTwoMotors_transmission_interface

#endif // GENERICTWOMOTORS_TRANSMISSION_H