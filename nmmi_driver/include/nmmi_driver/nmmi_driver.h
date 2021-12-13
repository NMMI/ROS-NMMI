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

#ifndef NMMI_DRIVER_H
#define NMMI_DRIVER_H

// Standard libraries
#include <array>

// internal libraries
#include <qb_device_driver/qb_device_driver.h>
#include <cp_communications.h>

namespace nmmi_driver {
/**
 * This class wraps the NMMI device-independent API to easily use it within the Communication Handler ROS node.
 */
class nmmiAPI : public qb_device_driver::qbDeviceAPI{
 public:

  int getADCConf(comm_settings *file_descriptor, const int &id, uint8_t &tot_adc_channels, std::vector<uint8_t> &adc_map) {
    return commGetADCConf(file_descriptor, id, &tot_adc_channels, (uint8_t*)&adc_map[0]);
  }

  int getADCRawValues(comm_settings *file_descriptor, const int &id, const uint8_t & used_adc_channels, std::vector<int16_t> &adc_raw) {
    return commGetADCRawValues(file_descriptor, id, used_adc_channels, (int16_t*)&adc_raw[0]);
  }

  int getControlMode(comm_settings *file_descriptor, const int &id, uint8_t &control_mode) {
    std::vector<int> ctrl_mode = {-1};
    unsigned char parameter_buffer[5000];
    int res = commGetParamList(file_descriptor, id, 0, NULL, 0, 0, parameter_buffer);
    ros::Duration(0.001).sleep();  // unexpected behaviour with no sleep

    getParameter<uint8_t>(6, parameter_buffer, ctrl_mode);
    
    control_mode = ctrl_mode.front();
    return res;
  }

  int getEncoderConf(comm_settings *file_descriptor, const int &id, bool &old_board, uint8_t &num_encoder_lines, uint8_t &num_encoder_per_line, std::vector<uint8_t> &enc_map) {
    
    old_board = false;
    int res = commGetEncoderConf(file_descriptor, id, &num_encoder_lines, &num_encoder_per_line, (uint8_t*)&enc_map[0]);
    if (res < 0) {
      // If commGetEncoderConf returns -1, the connected board is a PSoC3 board instead of a STM32 or PSoC5 board
      // so set the old board flag
      num_encoder_lines = 1;
      short int measurements[6];    // Max number of enocers per line in old boards
      num_encoder_per_line = commGetMeasurements(file_descriptor, id, measurements);
      for (int i = 0; i < num_encoder_per_line; i++){
        enc_map[i] = 1;
      }
      old_board = true;
      return 0;
    }

    return res;
  }

  int getEncoderRawValues(comm_settings *file_descriptor, const int &id, const uint8_t & num_encoder_conf_total, std::vector<uint16_t> &encoder_raw) {
    return commGetEncoderRawValues(file_descriptor, id, num_encoder_conf_total, (uint16_t*)&encoder_raw[0]);
  }

  int getEncoderStandardValues(comm_settings *file_descriptor, const int &id, short int measurements[]) {
    return commGetMeasurements(file_descriptor, id, measurements);
  }

  int getIMUValues(comm_settings *file_descriptor, const int &id, std::vector<uint8_t> imu_table, std::vector<uint8_t> imus_magcal, int n_imu, const bool &custom_read_timeout, std::vector<float> &imu_values) {
    if (custom_read_timeout){
      long r_timeout = 1250*n_imu;    // [usec], e.g. a 4-imus board with all sensors ON takes 4700 us avg. to be read
      return commGetImuReadings(file_descriptor, id, (uint8_t*)&imu_table[0], (uint8_t*)&imus_magcal[0], n_imu, (float*)&imu_values[0], r_timeout);
    }
    // Default read timeout (READ_TIMEOUT macro)
    return commGetImuReadings(file_descriptor, id, (uint8_t*)&imu_table[0], (uint8_t*)&imus_magcal[0], n_imu, (float*)&imu_values[0]);
  }

  int getIMUParam(comm_settings *file_descriptor, const int &id, bool &old_board, std::string &aux_str) {
    uint8_t aux_string[2000];

    old_board = false;
    if (commGetIMUParamList(file_descriptor, id, 0, NULL, 0, 0, aux_string) < 0){
    // If commGetIMUParamList returns -1, the connected board is a PSoC3 board instead of a STM32 or PSoC5 board
    // so call the commGetParamList instead
    	commGetParamList(file_descriptor, id, 0, NULL, 0, 0, aux_string);
    	old_board = true;
  	}
 	
    aux_str.reserve( 2000 );
    for ( char value : aux_string ) aux_str += value;

    return 0;
  }

private:
  /**
   * Extract the specified parameter from the given buffer where all the device parameters are stored.
   * \tparam T The data type of the single field of the parameter to be retrieved, e.g. \p int32_t.
   * \param parameter_id The specific value of the parameter to be retrieved, mapped in the device firmware.
   * \param[out] parameter_vector The vector where the values are stored (\b note: it is initially cleared).
   * \sa getParameters()
   */
  template<class T>
  void getParameter(const int &parameter_id, unsigned char *parameter_buffer, std::vector<int> &parameter_vector) {
    parameter_vector.clear();
    int number_of_values = parameter_buffer[(parameter_id-1)*PARAM_BYTE_SLOT + 7];
    int value_size = sizeof(T);
    for (int i=0; i<number_of_values; i++) {
      T parameter_field = 0;
      for (int j=0; j<value_size; j++) {
        parameter_field += parameter_buffer[(parameter_id-1)*PARAM_BYTE_SLOT + 8 + i*value_size + value_size - j - 1] << (8 * j);
      }
      parameter_vector.push_back(parameter_field);
    }
  }

  /**
   * Extract the specified parameter from the given buffer where all the device parameters are stored.
   * \tparam T The data type of the single field of the parameter to be retrieved, e.g. \p float.
   * \param parameter_id The specific value of the parameter to be retrieved, mapped in the device firmware.
   * \param[out] parameter_vector The vector where the values are stored (\b note: it is initially cleared).
   * \sa getParameters()
   */
  template<class T>
  void getParameter(const int &parameter_id, unsigned char *parameter_buffer, std::vector<float> &parameter_vector) {
    parameter_vector.clear();
    int number_of_values = parameter_buffer[(parameter_id-1)*PARAM_BYTE_SLOT + 7];
    int value_size = sizeof(T);
    for (int i=0; i<number_of_values; i++) {
      std::vector<uint8_t> parameter_field(value_size, 0);
      for (int j=0; j<value_size; j++) {
        parameter_field.at(j) = parameter_buffer[(parameter_id-1)*PARAM_BYTE_SLOT + 8 + i*value_size + value_size - j - 1];
      }
      parameter_vector.push_back(*(reinterpret_cast<T*>(parameter_field.data())));
    }
  }  
};
typedef std::shared_ptr<nmmiAPI> nmmiAPIPtr;
}  // namespace nmmi_driver

#endif // NMMI_DRIVER_H