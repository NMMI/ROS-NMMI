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

#include <generic_fw_hardware_interface/generic_fw_hardware_interface.h>

using namespace generic_fw_hardware_interface;

GenericFWHW::GenericFWHW() 
  : qbDeviceHW(std::make_shared<generic_fw_transmission_interface::GenericFWVirtualTransmission>(), {"fake_joint"}, {"fake_joint"}) {
}

GenericFWHW::~GenericFWHW() {

}

int GenericFWHW::getADCRawvalues() {
  if (services_.at("get_adc_raw_values")) {
    nmmi_srvs::GetADCRawValues srv;
    srv.request.id = device_.id;
    srv.request.max_repeats = device_.max_repeats;
    srv.request.get_values = get_adc_values_;
    srv.request.used_adc_channels = used_adc_channels_;
    services_.at("get_adc_raw_values").call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_NAMED("device_hw", "[GENERIC FW DeviceHW] cannot get ADC values from device [" << device_.id << "].");
      return srv.response.failures;     
    }
    Adc_Raw_.resize(srv.response.adc_raw.size());
    for (int i=0; i<srv.response.adc_raw.size(); i++){
      Adc_Raw_.at(i) = srv.response.adc_raw.at(i);
    }
    adc_time_stamp_ = srv.response.stamp;
    return srv.response.failures;
  }
  ROS_WARN_STREAM_NAMED("device_hw", "[GENERIC FW DeviceHW] service [get_ADC] seems no longer advertised. Trying to reconnect...");
  resetServicesAndWait();
  return -1;
}

int GenericFWHW::getEncoderRawvalues(){
if (services_.at("get_encoder_raw_values")) {
    nmmi_srvs::GetEncoderRawValues srv;
    srv.request.id = device_.id;
    srv.request.max_repeats = device_.max_repeats;
    srv.request.get_values = get_encoders_values_;
    srv.request.num_encoder_conf_total = num_encoder_conf_total_;
    srv.request.old_board = old_board_;
    services_.at("get_encoder_raw_values").call(srv);

    if (!srv.response.success) {
      ROS_ERROR_STREAM_NAMED("device_hw", "[GENERIC FW DeviceHW] cannot get Encoder Raw values from device [" << device_.id << "].");
      return srv.response.failures;     
    }
    Encoder_Raw_.resize(srv.response.enc_raw.size());
        
    for (int i=0; i<srv.response.enc_raw.size(); i++){
      Encoder_Raw_.at(i) = srv.response.enc_raw.at(i);
    }
    enc_time_stamp_ = srv.response.stamp;
    return srv.response.failures;
  }
  ROS_WARN_STREAM_NAMED("device_hw", "[GENERIC FW DeviceHW] service [get_Encoder_Raw] seems no longer advertised. Trying to reconnect...");
  resetServicesAndWait();
  return -1;
}

std::vector<std::string> GenericFWHW::getJoints() {
  return joints_.names;
}

bool GenericFWHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) {

  node_handle_ = robot_hw_nh;
  if (!robot_hw_nh.getParam("device_name", device_.name)) {
    ROS_ERROR_STREAM_NAMED("device_hw", "[GENERIC FW DeviceHW] cannot retrieve 'device_name' from the Parameter Service [" << robot_hw_nh.getNamespace() << "].");
    return false;
  }
  if (!robot_hw_nh.getParam("device_id", device_.id)) {
    ROS_ERROR_STREAM_NAMED("device_hw", "[GENERIC FW DeviceHW] cannot retrieve 'device_id' from the Parameter Service [" << robot_hw_nh.getNamespace() << "].");
    return false;
  }

  generic_pub_adc_ = robot_hw_nh.advertise<nmmi_msgs::adcArray>("adc", 1);
  generic_pub_adc_state_ = robot_hw_nh.advertise<nmmi_msgs::ADC_State>("adc_state", 1);
  generic_pub_encoders_   = robot_hw_nh.advertise<nmmi_msgs::encoderArray>("encoders", 1);
  generic_pub_encoders_state_ = robot_hw_nh.advertise<nmmi_msgs::Encoder_State>("encoders_state", 1);

  generic_sub_motors_ = robot_hw_nh.subscribe("actuators_command", 1, &GenericFWHW::motorCallback, this);

  interfaces_.initialize(this, joints_);
  transmission_.initialize(robot_hw_nh.param<std::string>("transmission", "transmission"), actuators_, joints_);

  // initialize Generic FW services before initializing the device
  initializeGenericFWServicesAndWait();
  waitForInitialization();
  ROS_INFO_STREAM(getInfo());

  // if the device interface initialization has succeed the device info have been retrieved
  std::static_pointer_cast<generic_fw_transmission_interface::GenericFWVirtualTransmission>(transmission_.getTransmission());

  return true;
}

int GenericFWHW::initializeDevice() {

  if (services_.at("initialize_nmmi_device")) {
    qb_device_srvs::InitializeDevice srv;
    srv.request.id = device_.id;
    srv.request.activate = node_handle_.param<bool>("activate_on_initialization", true);
    srv.request.rescan = node_handle_.param<bool>("rescan_on_initialization", false);
    int max_repeats = node_handle_.param<int>("max_repeats", 3);
    srv.request.max_repeats = max_repeats;
    services_.at("initialize_nmmi_device").call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_NAMED("device_hw", "[GENERIC FW DeviceHW] cannot initialize device [" << device_.id << "].");
      return -1;
    }
    device_.max_repeats = max_repeats;
    device_.serial_port = srv.response.info.serial_port;
    device_.set_commands = node_handle_.param<bool>("set_commands", true);
    device_.set_commands_async = node_handle_.param<bool>("set_commands_async", false);
    
    device_info_.id = device_.id;
    device_info_.serial_port = device_.serial_port;
    device_info_.max_repeats = device_.max_repeats;
    device_info_.get_currents = device_.get_currents;
    device_info_.get_positions = device_.get_positions;
    device_info_.get_distinct_packages = device_.get_distinct_packages;
    device_info_.position_limits = device_.position_limits;
    device_info_.encoder_resolutions = device_.encoder_resolutions;

    get_adc_values_ = node_handle_.param<bool>("get_adc_raw_values", false);
    get_encoders_values_ = node_handle_.param<bool>("get_encoder_raw_values", false);

    ROS_INFO_STREAM_NAMED("device_hw", "[GENERIC FW DeviceHW] device [" << device_.id << "] is initialized.");

    if (get_adc_values_){
      // Initialize also ADC map

      if (services_.at("get_adc_conf")) {
        nmmi_srvs::GetADCMap adcmap_srv;
        adcmap_srv.request.id = device_.id;
        services_.at("get_adc_conf").call(adcmap_srv);
        if (!adcmap_srv.response.success) {
          ROS_ERROR_STREAM_NAMED("device_hw", "[GENERIC FW DeviceHW] cannot read ADC parameters from device [" << device_.id << "].");
          return -1;
        }

        Adc_Raw_.resize(adcmap_srv.response.tot_adc_channels);

        used_adc_channels_ = 0;
        // Retrieve adc channel from map and update used_adc_channels_ variable
        for (int i = 0; i< adcmap_srv.response.tot_adc_channels; i++){
          if (adcmap_srv.response.map[i] == 1) {
              Adc_Map_.push_back(i);     // Adc raw channel reading configured
              used_adc_channels_++;
            }
        }
      }
    }

    if (get_encoders_values_) {
      // Initialize also Encoder map
      
      if (services_.at("get_encoder_conf")) {
        nmmi_srvs::GetEncoderMap encmap_srv;
        encmap_srv.request.id = device_.id;
        services_.at("get_encoder_conf").call(encmap_srv);
        if (!encmap_srv.response.success) {
          ROS_ERROR_STREAM_NAMED("device_hw", "[GENERIC FW DeviceHW] cannot read Encoder parameters from device [" << device_.id << "].");
          return -1;
        }

        Encoder_Raw_.resize(encmap_srv.response.num_encoder_lines*encmap_srv.response.num_encoder_per_line);

        num_encoder_conf_total_ = 0;
        for (int i = 0; i < encmap_srv.response.num_encoder_lines*encmap_srv.response.num_encoder_per_line; i++){
          if (encmap_srv.response.map[i]){
            Encoder_Map_.push_back(i);   // Encoder raw reading configured
            num_encoder_conf_total_++;
          }
        }

        // If the response variable 'old_board' is set to true, the connected board is a PSoC3 board instead of a STM32 or PSoC5 board
        // so read encoders with standard reading
        old_board_ = encmap_srv.response.old_board;
      }
    }

    // If all the services were good, return 0
    return 0;
  }

  ROS_WARN_STREAM_NAMED("device_hw", "[GENERIC FW DeviceHW] service [initialize_device] is no longer advertised.");
  resetServicesAndWait(false);
  return -1;
}

void GenericFWHW::initializeGenericFWServicesAndWait() {
  services_["activate_motors"] = node_handle_.serviceClient<qb_device_srvs::Trigger>("/communication_handler/activate_motors", true);
  services_["deactivate_motors"] = node_handle_.serviceClient<qb_device_srvs::Trigger>("/communication_handler/deactivate_motors", true);
  services_["get_adc_raw_values"] = node_handle_.serviceClient<nmmi_srvs::GetADCRawValues>("/communication_handler/get_adc_raw_values", true);
  services_["get_adc_conf"] = node_handle_.serviceClient<nmmi_srvs::GetADCMap>("/communication_handler/get_adc_conf", true);
  services_["get_encoder_raw_values"] = node_handle_.serviceClient<nmmi_srvs::GetEncoderRawValues>("/communication_handler/get_encoder_raw_values", true);
  services_["get_encoder_conf"] = node_handle_.serviceClient<nmmi_srvs::GetEncoderMap>("/communication_handler/get_encoder_conf", true);
  services_["initialize_nmmi_device"] = node_handle_.serviceClient<qb_device_srvs::InitializeDevice>("/communication_handler/initialize_nmmi_device", true);
  services_["set_motor_commands"] = node_handle_.serviceClient<qb_device_srvs::SetCommands>("/communication_handler/set_motor_commands", true);
  waitForServices();
}

/*---------------------------------------------------------------------*
*                                                *
*                                                                      *
*----------------------------------------------------------------------*/
void GenericFWHW::motorCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){

  // Note: This callback acts as a bypass for the qbDeviceHWResources interface

  actuators_.commands.clear();
  actuators_.commands.push_back(msg->data[0]);
  actuators_.commands.push_back(msg->data[1]);

}

void GenericFWHW::publish() {
  nmmi_msgs::adc tmp_adc;
  nmmi_msgs::adcArray adc;
  nmmi_msgs::ADC_State adc_msg;
  nmmi_msgs::encoder tmp_enc;
  nmmi_msgs::encoderArray enc;
  nmmi_msgs::Encoder_State enc_msg;

  // ADC
  adc_msg.id = device_.id;
  adc_msg.is_reliable = adc_consecutive_failures_ >= 0 && adc_consecutive_failures_ <= device_.max_repeats;
  adc_msg.consecutive_failures = adc_consecutive_failures_;
  adc_msg.stamp = adc_time_stamp_;

  for (int i=0; i<used_adc_channels_; i++){
    nmmi_msgs::ADCResourceData adc_msg_rd;
    adc_msg_rd.channel = Adc_Map_.at(i);
    adc_msg_rd.value = Adc_Raw_.at(i);

    adc_msg.s.push_back(adc_msg_rd);


    // Fill single topic
    tmp_adc.board_id = device_.id;
    tmp_adc.channel = Adc_Map_.at(i);
    tmp_adc.value = Adc_Raw_.at(i);
    adc.m.push_back(tmp_adc);    
  }

  // Encoder
  enc_msg.id = device_.id;
  enc_msg.is_reliable = encoder_consecutive_failures_ >= 0 && encoder_consecutive_failures_ <= device_.max_repeats;
  enc_msg.consecutive_failures = encoder_consecutive_failures_;
  enc_msg.stamp = enc_time_stamp_;

  for (int i=0; i<num_encoder_conf_total_; i++) {
    nmmi_msgs::EncoderResourceData enc_msg_rd;    
    enc_msg_rd.encoder_id = Encoder_Map_.at(i);
    enc_msg_rd.value = Encoder_Raw_.at(i);

    enc_msg.s.push_back(enc_msg_rd);


    // Fill single topic
    tmp_enc.board_id = device_.id;
    tmp_enc.encoder_id = Encoder_Map_.at(i);
    tmp_enc.value = Encoder_Raw_.at(i);
    enc.m.push_back(tmp_enc);    
  }

  if (get_adc_values_){
    generic_pub_adc_.publish(adc);  
  }
  generic_pub_adc_state_.publish(adc_msg);
  
  if (get_encoders_values_){
    generic_pub_encoders_.publish(enc);
  }
  generic_pub_encoders_state_.publish(enc_msg);
}

void GenericFWHW::read(const ros::Time& time, const ros::Duration& period) {

  // Call associated service
  adc_consecutive_failures_ = getADCRawvalues();

  encoder_consecutive_failures_ = getEncoderRawvalues();

  // propagate current actuator state to joints
  transmission_.actuator_to_joint_state.propagate();

  // make data available for other ROS nodes
  publish();
}

void GenericFWHW::resetServicesAndWait(const bool &reinitialize_device) {
  waitForServices();
  // reset all the service clients in case they were not yet advertised during initialization
  initializeGenericFWServicesAndWait();
  if (reinitialize_device) {
    waitForInitialization();
  }
}

int GenericFWHW::setCommands(const std::vector<double> &commands) {
  if (services_.at("set_motor_commands")) {
    qb_device_srvs::SetCommands srv;
    srv.request.id = device_.id;
    srv.request.max_repeats = device_.max_repeats;
    srv.request.set_commands = device_.set_commands;
    srv.request.set_commands_async = device_.set_commands_async;

    for (auto const &command : commands) {
      srv.request.commands.push_back(static_cast<short int>(command));
      //srv.request.commands.push_back(static_cast<short int>(command) * device_.motor_axis_direction);
    }

    services_.at("set_motor_commands").call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] cannot send commands to device [" << device_.id << "].");
      return -1;
    }
    return 0;
  }
  ROS_WARN_STREAM_NAMED("device_hw", "[DeviceHW] service [set_commands] seems no longer advertised. Trying to reconnect...");
  resetServicesAndWait();
  return -1;
}

void GenericFWHW::waitForInitialization() {
  while(initializeDevice()) {
    ros::Duration(1.0).sleep();
  }
}

void GenericFWHW::waitForServices() {
  for (auto &service : services_) {
    service.second.waitForExistence();
  }
  ROS_INFO_STREAM_NAMED("device_hw", "[GENERIC FW DeviceHW] is connected to all the services advertise by [CommunicationHandler].");
}

void GenericFWHW::write(const ros::Time& time, const ros::Duration& period) {
  // Generic device has 2 actuators as default

  // enforce joint limits for all registered interfaces
  joint_limits_.enforceLimits(period);

  // propagate joint commands to actuators
  transmission_.joint_to_actuator_position.propagate();

  // send actuator commands to the hardware
  setCommands(actuators_.commands);
}

PLUGINLIB_EXPORT_CLASS(generic_fw_hardware_interface::GenericFWHW, hardware_interface::RobotHW)
