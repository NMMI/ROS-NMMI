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

#include <nmmi_driver/nmmi_communication_handler.h>

using namespace nmmi_communication_handler;

nmmiCommunicationHandler::nmmiCommunicationHandler() :
    qbDeviceCommunicationHandler::qbDeviceCommunicationHandler(),
    node_handle_(ros::NodeHandle()),
    nmmi_api_(std::make_shared<nmmi_driver::nmmiAPI>()),
    get_imu_values_(node_handle_.advertiseService("/communication_handler/get_imu_values", &nmmiCommunicationHandler::getIMUvaluesCallback, this)),
    get_imu_param_(node_handle_.advertiseService("/communication_handler/get_imu_param", &nmmiCommunicationHandler::getIMUparamCallback, this)),
    get_adc_raw_values_(node_handle_.advertiseService("/communication_handler/get_adc_raw_values", &nmmiCommunicationHandler::getADCRawvaluesCallback, this)),
    get_adc_conf_(node_handle_.advertiseService("/communication_handler/get_adc_conf", &nmmiCommunicationHandler::getADCconfCallback, this)),
    get_control_mode_(node_handle_.advertiseService("/communication_handler/get_control_mode", &nmmiCommunicationHandler::getControlModeCallback, this)),
    get_encoder_raw_values_(node_handle_.advertiseService("/communication_handler/get_encoder_raw_values", &nmmiCommunicationHandler::getEncoderRawvaluesCallback, this)),
    get_encoder_conf_(node_handle_.advertiseService("/communication_handler/get_encoder_conf", &nmmiCommunicationHandler::getEncoderconfCallback, this)),    
    set_motor_commands_(node_handle_.advertiseService("/communication_handler/set_motor_commands", &nmmiCommunicationHandler::setCommandsCallback, this)),
    initialize_nmmi_device_(node_handle_.advertiseService("/communication_handler/initialize_nmmi_device", &nmmiCommunicationHandler::initializeNMMIDeviceCallback, this)){
}

nmmiCommunicationHandler::~nmmiCommunicationHandler() {
}

int nmmiCommunicationHandler::getADCRawvalues(const int &id, const int &max_repeats, const uint8_t &used_adc_channels, std::vector<int16_t> &adc){
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  bool good_reading = false;
  std::vector<int16_t> adc_raw_values;

  // Resize adc raw values vector
  adc.resize(used_adc_channels);
  adc_raw_values.resize(used_adc_channels);

  //std::cout << "Used ADC channels: " << std::endl << used_adc_channels << std::endl;

  while (failures <= max_repeats) {
  
      good_reading = (!nmmi_api_->getADCRawValues(&file_descriptors_.at(connected_devices_.at(id)), id, 
                    used_adc_channels, adc_raw_values));
  
    if (good_reading) {
      // Unpack adc_raw_values vector
      for (int i = 0; i < used_adc_channels; i++) {
        adc[i] = adc_raw_values.at(i);
      }
    }
    else {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

bool nmmiCommunicationHandler::getADCRawvaluesCallback(nmmi_srvs::GetADCRawValuesRequest &request, nmmi_srvs::GetADCRawValuesResponse &response){
  ROS_ERROR_STREAM_COND_NAMED(request.max_repeats < 0, "NMMI communication_handler", "Device [" << request.id << "] has request service with non-valid 'max_request' [" << request.max_repeats << "].");
  if (!isInConnectedSet(request.id)) {
    response.success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request.id)));

  response.failures = 0;  // need to return true even if 'get_values' is set to false
  if (request.get_values) {
    response.failures = getADCRawvalues(request.id, request.max_repeats, request.used_adc_channels, response.adc_raw);  // blocks while reading
  }

  response.stamp = ros::Time::now();
  response.success = isReliable(response.failures, request.max_repeats);
  return true;
}

bool nmmiCommunicationHandler::getADCconfCallback(nmmi_srvs::GetADCMapRequest &request, nmmi_srvs::GetADCMapResponse &response){
  uint8_t tot_adc_channels = 0;
  std::vector<uint8_t> adc_map_v(12);     // 12 MAX NUMBER OF ADC CHANNELS AVAILABLE

  ROS_ERROR_STREAM_COND_NAMED(request.id < 0, "NMMI communication_handler", "Device [" << request.id << "] does not exist.");
  if (!isInConnectedSet(request.id)) {
    response.success = false;
    return true;
  }
  
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request.id)));

  response.failures = nmmi_api_->getADCConf(&file_descriptors_.at(connected_devices_.at(request.id)), request.id, tot_adc_channels, adc_map_v);  // blocks while reading
  response.tot_adc_channels = tot_adc_channels;
  response.success = isReliable(response.failures, 3);
  response.map.resize(response.tot_adc_channels);
  for (int i=0; i< response.tot_adc_channels; i++){
    response.map.at(i) = adc_map_v.at(i);
  }

  return true;
}

bool nmmiCommunicationHandler::getControlModeCallback(nmmi_srvs::GetControlModeRequest &request, nmmi_srvs::GetControlModeResponse &response){
  
  ROS_ERROR_STREAM_COND_NAMED(request.id < 0, "NMMI communication_handler", "Device [" << request.id << "] does not exist.");
  if (!isInConnectedSet(request.id)) {
    response.success = false;
    return true;
  }
  
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request.id)));

  response.failures = nmmi_api_->getControlMode(&file_descriptors_.at(connected_devices_.at(request.id)), request.id, response.control_mode);  // blocks while reading
  response.success = isReliable(response.failures, 3);

  return true;
}

int nmmiCommunicationHandler::getEncoderRawvalues(const int &id, const int &max_repeats, const uint8_t &num_encoder_conf_total, const bool old_board, std::vector<uint16_t> &enc){
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  bool good_reading = false;
  std::vector<uint16_t> encoder_raw_values;

  // Resize adc raw values vector
  enc.resize(num_encoder_conf_total);
  encoder_raw_values.resize(num_encoder_conf_total);

  //std::cout << "Used Encoder channels: " << std::endl << num_encoder_conf_total << std::endl;

  while (failures <= max_repeats) {
  
    if (!old_board){
      good_reading = (!nmmi_api_->getEncoderRawValues(&file_descriptors_.at(connected_devices_.at(id)), id, 
                    num_encoder_conf_total, encoder_raw_values));  
    }
    else {
      short int meas[num_encoder_conf_total];
      for(int i = 0; i < num_encoder_conf_total; i++){
        meas[i] = 0;
      }
      
      good_reading = (nmmi_api_->getEncoderStandardValues(&file_descriptors_.at(connected_devices_.at(id)), id, meas) > 0);  
      for (int i=0; i < num_encoder_conf_total;i++){
        encoder_raw_values[i] = meas[i];
      }
    }
    
    if (good_reading) {
      // Unpack adc_raw_values vector
      for (int i = 0; i < num_encoder_conf_total; i++) {
        enc[i] = encoder_raw_values.at(i);
      }
    }
    else {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

bool nmmiCommunicationHandler::getEncoderRawvaluesCallback(nmmi_srvs::GetEncoderRawValuesRequest &request, nmmi_srvs::GetEncoderRawValuesResponse &response){
  ROS_ERROR_STREAM_COND_NAMED(request.max_repeats < 0, "NMMI communication_handler", "Device [" << request.id << "] has request service with non-valid 'max_request' [" << request.max_repeats << "].");
  if (!isInConnectedSet(request.id)) {
    response.success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request.id)));

  response.failures = 0;  // need to return true even if 'get_values' is set to false
  if (request.get_values) {
    response.failures = getEncoderRawvalues(request.id, request.max_repeats, request.num_encoder_conf_total, request.old_board, response.enc_raw);  // blocks while reading
  }

  response.stamp = ros::Time::now();
  response.success = isReliable(response.failures, request.max_repeats);
  return true;
}

bool nmmiCommunicationHandler::getEncoderconfCallback(nmmi_srvs::GetEncoderMapRequest &request, nmmi_srvs::GetEncoderMapResponse &response){
  uint8_t num_encoder_lines = 2;
  uint8_t num_encoder_per_line = 5;
  bool old_board = false;

  std::vector<uint8_t> enc_map_v(num_encoder_lines*num_encoder_per_line);     // Size equal to max number of encoders available

  ROS_ERROR_STREAM_COND_NAMED(request.id < 0, "NMMI communication_handler", "Device [" << request.id << "] does not exist.");
  if (!isInConnectedSet(request.id)) {
    response.success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request.id)));

  response.failures = nmmi_api_->getEncoderConf(&file_descriptors_.at(connected_devices_.at(request.id)), request.id, old_board, num_encoder_lines, num_encoder_per_line, enc_map_v);  // blocks while reading
  response.num_encoder_lines = num_encoder_lines;
  response.num_encoder_per_line = num_encoder_per_line;
  response.old_board = old_board;
  response.map.resize(response.num_encoder_lines * response.num_encoder_per_line);
  for (int i=0; i< response.num_encoder_lines * response.num_encoder_per_line; i++){
    response.map.at(i) = enc_map_v.at(i);
  }
  response.success = isReliable(response.failures, 3);

  return true;
}

int nmmiCommunicationHandler::getIMUvalues(const int &id, const int &max_repeats, const nmmi_msgs::ImuTable &imu_t, 
                                          const bool &custom_read_timeout, std::vector<float> &acc, std::vector<float> &gyro, 
                                          std::vector<float> &mag, std::vector<float> &quat, std::vector<float> &temp) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  bool good_reading = false;
  std::vector<float> imu_values;

  // Read imu table (cast from std::vector to uint8 array)
  acc.resize(imu_t.n_imu*3);
  gyro.resize(imu_t.n_imu*3);
  mag.resize(imu_t.n_imu*3);
  quat.resize(imu_t.n_imu*4);
  temp.resize(imu_t.n_imu);
  imu_values.resize(imu_t.n_imu*(3*3+4+1));


  //std::cout << "IMU TABLE: " << std::endl << imu_t << std::endl;

  while (failures <= max_repeats) {

//ros::Time rostime_begin = ros::Time::now();
  
      good_reading = (!nmmi_api_->getIMUValues(&file_descriptors_.at(connected_devices_.at(id)), id, 
                    imu_t.imu_table, imu_t.mag_cal, imu_t.n_imu, custom_read_timeout, imu_values));

//ros::Duration rostime_dur = ros::Time::now() - rostime_begin;
//std::cout << "(ID " << id << ") PERIOD: " << rostime_dur << " GR: " << good_reading << std::endl;
  
    if (good_reading) {
      // Unpack imu_values vector
      for (int i = 0; i < imu_t.n_imu; i++) {
             
        if (imu_t.imu_table.at(5*i + 0)) {
          for (int j=0; j<3; j++) {
            acc[3*i + j] = imu_values.at((3*3+4+1)*i + j);
          }
        }
        if (imu_t.imu_table.at(5*i + 1)) {
          for (int j=0; j<3; j++) {
            gyro[3*i + j] = imu_values.at((3*3+4+1)*i + j + 3);
          }
        }
        if (imu_t.imu_table.at(5*i + 2)) {
          for (int j=0; j<3; j++) {
            mag[3*i + j] = imu_values.at((3*3+4+1)*i + j + 6);
          }
        }
        if (imu_t.imu_table.at(5*i + 3)) {
          for (int j=0; j<4; j++) {
            quat[4*i + j] = imu_values.at((3*3+4+1)*i + j + 9);
          }
        }
        if (imu_t.imu_table.at(5*i + 4)) {
            temp[i] = imu_values.at((3*3+4+1)*i + 13);
        }
      }
    }
    else {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

bool nmmiCommunicationHandler::getIMUvaluesCallback(nmmi_srvs::GetIMUValuesRequest &request, nmmi_srvs::GetIMUValuesResponse &response) {
  ROS_ERROR_STREAM_COND_NAMED(request.max_repeats < 0, "NMMI communication_handler", "Device [" << request.id << "] has request service with non-valid 'max_request' [" << request.max_repeats << "].");
  if (!isInConnectedSet(request.id)) {
    response.success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request.id)));

  response.failures = 0;  // need to return true even if 'get_values' is set to false
  if (request.get_values) {
    response.failures = getIMUvalues(request.id, request.max_repeats, request.imu_table, request.custom_read_timeout, response.acc, response.gyro, response.mag, response.quat, response.temp);  // blocks while reading
  }

  response.stamp = ros::Time::now();
  response.success = isReliable(response.failures, request.max_repeats);
  return true;
}

bool nmmiCommunicationHandler::getIMUparamCallback(nmmi_srvs::GetIMUParamRequest &request, nmmi_srvs::GetIMUParamResponse &response) {
  bool old_board = false;

  ROS_ERROR_STREAM_COND_NAMED(request.id < 0, "NMMI communication_handler", "Device [" << request.id << "] does not exist.");
  if (!isInConnectedSet(request.id)) {
    response.success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request.id)));

  response.failures = nmmi_api_->getIMUParam(&file_descriptors_.at(connected_devices_.at(request.id)), request.id, old_board, response.message);  // blocks while reading
  response.old_board = old_board;
  response.success = isReliable(response.failures, 3);

  return true;
}

int nmmiCommunicationHandler::getParameters(const int &id, std::vector<int> &limits, std::vector<int> &resolutions) {
  std::vector<int> input_mode = {-1};
  std::vector<int> ctrl_mode = {-1};
  nmmi_api_->getParameters(&file_descriptors_.at(connected_devices_.at(id)), id, input_mode, ctrl_mode, resolutions, limits);
  if (!input_mode.front()) {  // at least input mode equals 0 is required, i.e. USB connected

    return 0;
  }
  return -1;
}

bool nmmiCommunicationHandler::initializeNMMIDeviceCallback(qb_device_srvs::InitializeDeviceRequest &request, qb_device_srvs::InitializeDeviceResponse &response) {

  ROS_ERROR_STREAM_COND_NAMED(request.max_repeats < 0, "NMMI communication_handler", "Device [" << request.id << "] has request service with non-valid 'max_request' [" << request.max_repeats << "].");
  std::vector<std::unique_lock<std::mutex>> serial_locks;  // need to lock on all the serial resources to scan for new ports/devices
  for (auto const &mutex : serial_protectors_) {
    serial_locks.push_back(std::unique_lock<std::mutex>(*mutex.second));
  }

  if (request.rescan) {
    // update connected devices
    getSerialPortsAndDevices(request.max_repeats);
  }
  if (!isInConnectedSet(request.id) || !isReliable(isConnected(request.id, request.max_repeats), request.max_repeats)) {
    ROS_ERROR_STREAM_NAMED("NMMI communication_handler", "[CommunicationHandler] fails while initializing device [" << request.id << "] because it is not connected.");
    response.message = "Device [" + std::to_string(request.id) + "] initialization fails because it is not connected.";
    response.success = false;
    return true;
  }
  
  if (getParameters(request.id, response.info.position_limits, response.info.encoder_resolutions)) {
    ROS_ERROR_STREAM_NAMED("communication_handler", "[CommunicationHandler] fails while initializing device [" << request.id << "] because it requires 'USB' input mode and 'Position' control mode.");
    response.message = "Device [" + std::to_string(request.id) + "] initialization fails because it requires 'USB' input mode and 'Position' control mode.";
    response.success = false;
    return true;
  }

  if (request.activate) {
    response.failures = activate(request.id, request.max_repeats);
    response.success = isReliable(response.failures, request.max_repeats);
    if (!response.success) {
      ROS_INFO_STREAM_NAMED("NMMI communication_handler", "[CommunicationHandler] has not initialized device [" << request.id << "] because it cannot activate its motors (please, check the motor positions).");
      response.message = "Device [" + std::to_string(request.id) + "] initialization fails because it cannot activate the device (please, check the motor positions).";
      return true;
    }
  }

  response.info.id = request.id;
  response.info.serial_port = connected_devices_.at(request.id);
  ROS_INFO_STREAM_NAMED("NMMI communication_handler", "[CommunicationHandler] has initialized device [" << request.id << "].");
  response.message = "Device [" + std::to_string(request.id) + "] initialization succeeds.";
  response.success = true;
  return true;
}

int nmmiCommunicationHandler::isConnected(const int &id, const int &max_repeats) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  /*
  while (failures <= max_repeats) {
    if (!nmmi_api_->getStatus(&file_descriptors_.at(connected_devices_.at(id)), id)) {
      failures++;
      continue;
    }
    break;
  }*/
  return failures;
}

bool nmmiCommunicationHandler::setCommandsCallback(qb_device_srvs::SetCommandsRequest &request, qb_device_srvs::SetCommandsResponse &response) {
 return qbDeviceCommunicationHandler::setCommandsCallback(request, response);
}