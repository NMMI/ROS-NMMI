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

#include <GenericTwoMotors_hardware_interface/GenericTwoMotors_hardware_interface.h>

using namespace GenericTwoMotors_hardware_interface;

GenericTwoMotorsHW::GenericTwoMotorsHW()
    : qbDeviceHW(std::make_shared<GenericTwoMotors_transmission_interface::GenericTwoMotorsTransmission>(), {"motor_1_joint", "motor_2_joint", "shaft_joint"}, {"motor_1_joint", "motor_2_joint", "enc_3_joint", "enc_4_joint"}) {
}

GenericTwoMotorsHW::~GenericTwoMotorsHW() {

}

std::vector<std::string> GenericTwoMotorsHW::addNamespacePrefix(const std::vector<std::string> &vector) {
  std::vector<std::string> namespaced_vector(vector);
  std::string prefix = device_.name + "_";
  for (auto &elem : namespaced_vector) {
    if (!std::regex_match(elem, std::regex("^" + prefix + ".*"))) {
      elem = prefix + elem;
    }
  }
  return namespaced_vector;
}

std::vector<std::string> GenericTwoMotorsHW::getJoints() {
  /*if (command_with_position_and_preset_) {
    return {joints_.names.at(2), joints_.names.at(3)};
  }*/
  return {joints_.names.at(0), joints_.names.at(1)};
}

bool GenericTwoMotorsHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) {
   
  node_handle_ = robot_hw_nh;
  if (!robot_hw_nh.getParam("device_name", device_.name)) {
    ROS_ERROR_STREAM_NAMED("device_hw", "[GENERIC TWO MOTORS DeviceHW] cannot retrieve 'device_name' from the Parameter Service [" << robot_hw_nh.getNamespace() << "].");
    return false;
  }
  if (!robot_hw_nh.getParam("device_id", device_.id)) {
    ROS_ERROR_STREAM_NAMED("device_hw", "[GENERIC TWO MOTORS DeviceHW] cannot retrieve 'device_id' from the Parameter Service [" << robot_hw_nh.getNamespace() << "].");
    return false;
  }
  if (!urdf_model_.initParamWithNodeHandle("robot_description", root_nh)) {
    ROS_ERROR_STREAM_NAMED("device_hw", "[GENERIC TWO MOTORS DeviceHW] cannot retrieve 'robot_description' from the Parameter Service [" << robot_hw_nh.getNamespace() << "].");
    return false;
  }

  state_publisher_ = robot_hw_nh.advertise<qb_device_msgs::StateStamped>("state", 1);

  actuators_.setJoints(robot_hw_nh.param<std::vector<std::string>>("actuators", addNamespacePrefix(actuators_.names)));
  joints_.setJoints(robot_hw_nh.param<std::vector<std::string>>("joints", addNamespacePrefix(joints_.names)));

  interfaces_.initialize(this, joints_);
  joint_limits_.initialize(robot_hw_nh, joints_, urdf_model_, interfaces_.joint_position);
  transmission_.initialize(robot_hw_nh.param<std::string>("transmission", "transmission"), actuators_, joints_);

  use_simulator_mode_ = robot_hw_nh.param<bool>("use_simulator_mode", false);

  // initialize Generic FW services before initializing the device
  initializeGenericTwoMotorsServicesAndWait();
  waitForInitialization();
  ROS_INFO_STREAM(getInfo());

  // if the device interface initialization has succeed the device info have been retrieved
  std::static_pointer_cast<GenericTwoMotors_transmission_interface::GenericTwoMotorsTransmission>(transmission_.getTransmission());
  std::static_pointer_cast<GenericTwoMotors_transmission_interface::GenericTwoMotorsTransmission>(transmission_.getTransmission())->setPositionFactor(device_.encoder_resolutions);

  max_motor_limits_ = device_.position_limits.at(1);
  min_motor_limits_ = device_.position_limits.at(0);
  
return true;
}

int GenericTwoMotorsHW::initializeDevice() {

  if (services_.at("initialize_nmmi_device")) {
    qb_device_srvs::InitializeDevice srv;
    srv.request.id = device_.id;
    srv.request.activate = node_handle_.param<bool>("activate_on_initialization", false) && !use_simulator_mode_;
    srv.request.rescan = node_handle_.param<bool>("rescan_on_initialization", false);
    int max_repeats = node_handle_.param<int>("max_repeats", 3);
    srv.request.max_repeats = max_repeats;
    services_.at("initialize_nmmi_device").call(srv);
    
    if (!srv.response.success) {
      ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] cannot initialize device [" << device_.id << "].");
      return -1;
    }
    device_.max_repeats = max_repeats;
    device_.get_currents = node_handle_.param<bool>("get_currents", true);
    device_.get_positions = node_handle_.param<bool>("get_positions", true);
    device_.get_distinct_packages = node_handle_.param<bool>("get_distinct_packages", false);
    device_.set_commands = node_handle_.param<bool>("set_commands", true);
    device_.set_commands_async = node_handle_.param<bool>("set_commands_async", false);
    device_.serial_port = srv.response.info.serial_port;
    device_.position_limits = srv.response.info.position_limits;
    device_.encoder_resolutions = srv.response.info.encoder_resolutions;

    device_info_.id = device_.id;
    device_info_.serial_port = device_.serial_port;
    device_info_.max_repeats = device_.max_repeats;
    device_info_.get_currents = device_.get_currents;
    device_info_.get_positions = device_.get_positions;
    device_info_.get_distinct_packages = device_.get_distinct_packages;
    device_info_.set_commands = device_.set_commands;
    device_info_.set_commands_async = device_.set_commands_async;
    device_info_.position_limits = device_.position_limits;
    device_info_.encoder_resolutions = device_.encoder_resolutions;


    if (services_.at("get_control_mode")) {
      nmmi_srvs::GetControlMode ctrlmode_srv;
      ctrlmode_srv.request.id = device_.id;
      services_.at("get_control_mode").call(ctrlmode_srv);
      if (!ctrlmode_srv.response.success) {
        ROS_ERROR_STREAM_NAMED("device_hw", "[GENERIC TWO MOTORS DeviceHW] cannot read Control Mode from device [" << device_.id << "].");
        return -1;
      }

      control_mode_ = ctrlmode_srv.response.control_mode;
    }

    ROS_INFO_STREAM_NAMED("device_hw", "[GENERIC TWO MOTORS DeviceHW] device [" << device_.id << "] is initialized.");
    return 0;
  }

  ROS_WARN_STREAM_NAMED("device_hw", "[GENERIC TWO Motors DeviceHW] service [initialize_device] is no longer advertised.");
  resetServicesAndWait(false);
  return -1;
}

void GenericTwoMotorsHW::initializeGenericTwoMotorsServicesAndWait() {
  services_["activate_motors"] = node_handle_.serviceClient<qb_device_srvs::Trigger>("/communication_handler/activate_motors", true);
  services_["deactivate_motors"] = node_handle_.serviceClient<qb_device_srvs::Trigger>("/communication_handler/deactivate_motors", true);
  services_["get_control_mode"] = node_handle_.serviceClient<nmmi_srvs::GetControlMode>("/communication_handler/get_control_mode", true);
  services_["get_info"] = node_handle_.serviceClient<qb_device_srvs::Trigger>("/communication_handler/get_info", true);
  services_["get_measurements"] = node_handle_.serviceClient<qb_device_srvs::GetMeasurements>("/communication_handler/get_measurements", true);
  services_["initialize_nmmi_device"] = node_handle_.serviceClient<qb_device_srvs::InitializeDevice>("/communication_handler/initialize_nmmi_device", true);
  services_["set_commands"] = node_handle_.serviceClient<qb_device_srvs::SetCommands>("/communication_handler/set_commands", true);
  waitForServices();
}

void GenericTwoMotorsHW::read(const ros::Time& time, const ros::Duration& period) {
  // read actuator state from the hardware (convert to proper measurement units)
  qb_device_hardware_interface::qbDeviceHW::read(time, period);
}

void GenericTwoMotorsHW::resetServicesAndWait(const bool &reinitialize_device) {
  waitForServices();
  // reset all the service clients in case they were not yet advertised during initialization
  initializeGenericTwoMotorsServicesAndWait();
  if (reinitialize_device) {
    waitForInitialization();
  }
}

void GenericTwoMotorsHW::waitForInitialization() {
  while(initializeDevice()) {
    ros::Duration(1.0).sleep();
  }
}

void GenericTwoMotorsHW::waitForServices() {
  for (auto &service : services_) {
    service.second.waitForExistence();
  }
  ROS_INFO_STREAM_NAMED("device_hw", "[GENERIC TWO MOTORS DeviceHW] is connected to all the services advertise by [CommunicationHandler].");
}

void GenericTwoMotorsHW::write(const ros::Time& time, const ros::Duration& period) {
  // send actuator command to the hardware (saturate and convert to proper measurement units)
  qb_device_hardware_interface::qbDeviceHW::write(time, period);
}

PLUGINLIB_EXPORT_CLASS(GenericTwoMotors_hardware_interface::GenericTwoMotorsHW, hardware_interface::RobotHW)