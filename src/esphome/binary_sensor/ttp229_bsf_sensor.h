#ifndef ESPHOME_BINARY_SENSOR_TTP229_BSF_H
#define ESPHOME_BINARY_SENSOR_TTP229_BSF_H

#include "esphome/defines.h"

#ifdef USE_TTP229_BSF

#include "esphome/binary_sensor/binary_sensor.h"
#include "esphome/component.h"
#include "esphome/esphal.h"

ESPHOME_NAMESPACE_BEGIN

namespace binary_sensor {

class TTP229Channel : public binary_sensor::BinarySensor {
 public:
  TTP229Channel(const std::string &name, int channel_num = 0);
  void process(const uint16_t *data);

 protected:
  int channel_ = 0;
};

class TTP229BSFComponent : public Component {
 public:
  TTP229BSFComponent(GPIOPin *sdo_pin, GPIOPin *scl_pin);
  binary_sensor::TTP229Channel *add_channel(binary_sensor::TTP229Channel *channel);
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void loop() override;

 protected:
  GPIOPin *sdo_pin_;
  GPIOPin *scl_pin_;
  std::vector<TTP229Channel *> channels_{};
  uint16_t lasttouched_{0};
  uint16_t currtouched_{0};
  void process_(uint16_t *data);
  bool GetBit();
  uint16_t read_data_(uint8_t num_bits);
};

}  // namespace binary_sensor

ESPHOME_NAMESPACE_END

#endif  // USE_TTP229_BSF

#endif  // ESPHOME_BINARY_SENSOR_TTP229_BSF_H