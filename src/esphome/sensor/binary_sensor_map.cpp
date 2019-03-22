#include "esphome/defines.h"

#ifdef USE_BINARY_SENSOR_MAP

#include "esphome/sensor/binary_sensor_map.h"
#include "esphome/helpers.h"
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
      ESP_LOGD(TAG, "%s - touched value: %.2f", this->name_.c_str(), publish_value);
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
  int32_t pos = this->get_position_();
  //  ESP_LOGD(TAG, "%d ", pos);
  //  if (last_touched_ ) {
  //    ESP_LOGD(TAG, "%d ", pos);
  // this->publish_state(pos);
  //    this->last_position_ = pos;
  //}
}

int32_t BinarySensorMap::get_position_() {
  this->last_touched_ = false;
  float val = 0;
  uint8_t num_channels = 0;
  uint8_t channel_index = 0;
  for (auto *bs : this->sensors_) {
    if (bs->binary_sensor->state) {
      this->last_touched_ = true;
      val += bs->sensor_value;
      num_channels++;
    } else if (this->last_touched_) {
      ESP_LOGD(TAG, "%.2f, %.2f", val, (float) val / num_channels);
      return val / num_channels;
    }
  }
  return 0;
}

float BinarySensorMap::get_setup_priority() const { return setup_priority::HARDWARE_LATE; }

void BinarySensorMap::add_sensor(binary_sensor::BinarySensor *sensor, float value) {
  ESP_LOGD(TAG, "add sensor value %.0f", value);
  BinarySensorMapChannel *sensor_channel = new BinarySensorMapChannel;
  sensor_channel->binary_sensor = sensor;
  sensor_channel->sensor_value = value;
  this->sensors_.push_back(sensor_channel);
}

void BinarySensorMap::add_sensor(binary_sensor::BinarySensor *sensor) {
  float value = (float) this->num_channels_++;
  this->add_sensor(sensor, value);
}

void BinarySensorMap::set_sensor_type(uint8_t sensor_type) { this->sensor_type_ = sensor_type; }

}  // namespace sensor

ESPHOME_NAMESPACE_END

#endif  // USE_BINARY_SENSOR_MAP
