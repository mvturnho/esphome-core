#include "esphome/defines.h"

#ifdef USE_BINARY_SENSOR_MAP

#include "esphome/sensor/binary_sensor_map.h"

#include "esphome/log.h"

ESPHOME_NAMESPACE_BEGIN

namespace sensor {

static const char *TAG = "sensor.binary_sensor_map";

BinarySensorMap::BinarySensorMap(const std::string &name) : Sensor(name) {}

void BinarySensorMap::dump_config() {
  ESP_LOGD(TAG, "BINARY_SENSOR_MAP:");
  LOG_SENSOR("  ", "binary_sensor_map", this);
}

void BinarySensorMap::loop() {
  switch (this->sensor_type_) {
    case BINARY_SENSOR_MAP_TYPE_GROUP:
      this->process_group_();
      break;
    case BINARY_SENSOR_MAP_TYPE_SLIDER:
      this->process_slider_();
      break;
  }
}

void BinarySensorMap::process_group_() {
  float total_current_value = 0.0;
  uint8_t num_active_sensors = 0;
  bool touched = false;
  uint64_t mask = 0x00;
  uint8_t cur_sens = 0;
  // check all binary_sensors for its state. when active add its value to total_current_value.
  // create a bitmask for the binary_sensor status on all channels
  for (auto *bs : this->sensors_) {
    if (bs->binary_sensor->state) {
      touched = true;
      num_active_sensors++;
      total_current_value += bs->sensor_value;
      mask |= 1 << cur_sens;
    }
    cur_sens++;
  }
  // check if the sensor map was touched
  if (touched) {
    // did the bit_mask change or is it a new sensor touch
    if ((last_mask_ != mask) || !this->last_touched_) {
      this->last_touched_ = true;
      float publish_value = total_current_value / num_active_sensors;
      ESP_LOGD(TAG, "%s - touched value: %f", this->name_.c_str(), publish_value);
      this->publish_state(publish_value);
      last_mask_ = mask;
    }
  } else {
    // is this a new sensor release
    if (this->last_touched_) {
      this->last_touched_ = false;
      ESP_LOGD(TAG, "%s - release value: nan", this->name_.c_str());
      this->publish_state(NAN);
    }
  }
}

void BinarySensorMap::process_slider_() {
  uint64_t sensor_sum = 0;
  uint8_t max_index = 0;
  uint8_t non0_cnt = 0;
  uint32_t slide_pos_temp = this->last_position_;
  float pos = 0;
  for (int i = 0; i < this->sensors_.size(); i++) {
    if (i >= 2) {
      // find the max sum of three continuous values
      uint64_t neb_sum = this->sensors_[i - 2]->binary_sensor->state + this->sensors_[i - 1]->binary_sensor->state +
                         this->sensors_[i]->binary_sensor->state;
      if (neb_sum > sensor_sum) {
        // val_sum is the max value of neb_sum
        sensor_sum = neb_sum;
        max_index = i - 1;
        // Check the zero data number.
        non0_cnt = 0;
        for (int j = i - 2; j <= i; j++) {
          non0_cnt += (this->sensors_[j]->binary_sensor->state) ? 1 : 0;
        }
      }
    }
  }
  if (non0_cnt == 0) {
    // if the max value of neb_sum is zero, no pad is touched
    // slide_pos_temp = SLIDE_POS_INF;
  } else if (non0_cnt == 1) {  // only touch one pad
    // Check the active button number.
    uint8_t no_zero = 0;
    for (int i = 0; i < this->sensors_.size(); i++) {
      if (this->sensors_[i]->binary_sensor->state) {
        no_zero++;
      }
    }
    // if (no_zero > non0_cnt), May be duplex slider board. TOUCHPAD_DUPLEX_SLIDER.
    // If duplex slider board, should not identify one button touched.
    if (no_zero <= non0_cnt) {  // Linear slider. TOUCHPAD_LINEAR_SLIDER
      for (int i = max_index - 1; i <= max_index + 1; i++) {
        if (this->sensors_[i]->binary_sensor->state) {
          if (i == this->sensors_.size() - 1) {
            slide_pos_temp = this->max_value_;
          } else {
            slide_pos_temp = (uint32_t)(i * this->scale_);
          }
          break;
        }
      }
    }
  } else if (non0_cnt == 2) {  // Only touch two pad
    if (!this->sensors_[max_index - 1]) {
      // return the corresponding position.
      pos = ((max_index + 1) * this->sensors_[max_index + 1]->binary_sensor->state +
             (max_index) * this->sensors_[max_index]->binary_sensor->state) *
            this->scale_;
      slide_pos_temp = (uint32_t)(pos / sensor_sum);
    } else if (!this->sensors_[max_index + 1]) {
      // return the corresponding position.
      pos = ((max_index - 1) * this->sensors_[max_index - 1]->binary_sensor->state +
             (max_index) * this->sensors_[max_index]->binary_sensor->state) *
            this->scale_;
      slide_pos_temp = (uint32_t)(pos / sensor_sum);
    } else {
      // slide_pos_temp = tp_slide->slide_pos;
    }
  } else {
    // return the corresponding position.
    pos = ((max_index - 1) * this->sensors_[max_index - 1]->binary_sensor->state +
           (max_index) * this->sensors_[max_index]->binary_sensor->state +
           (max_index + 1) * this->sensors_[max_index + 1]->binary_sensor->state) *
          this->scale_;
    slide_pos_temp = (uint32_t)(pos / sensor_sum);
  }
  ESP_LOGD(TAG, "%s - slider pos: %f", this->name_.c_str(), pos);
  this->publish_state(pos);
}

float BinarySensorMap::get_setup_priority() const { return setup_priority::HARDWARE_LATE; }

void BinarySensorMap::add_sensor(binary_sensor::BinarySensor *sensor, float value) {
  BinarySensorMapChannel *sensor_channel = new BinarySensorMapChannel;
  sensor_channel->binary_sensor = sensor;
  sensor_channel->sensor_value = value;
  this->sensors_.push_back(sensor_channel);
}

void BinarySensorMap::set_sensor_type(uint8_t sensor_type) { this->sensor_type_ = sensor_type; }

}  // namespace sensor

ESPHOME_NAMESPACE_END

#endif  // USE_BINARY_SENSOR_MAP
