#ifndef ESPHOME_SENSOR_BINARY_SENSOR_MAP_H
#define ESPHOME_SENSOR_BINARY_SENSOR_MAP_H

#include "esphome/defines.h"

#ifdef USE_BINARY_SENSOR_MAP

#include "esphome/component.h"
#include "esphome/sensor/sensor.h"
#include "esphome/binary_sensor/binary_sensor.h"

ESPHOME_NAMESPACE_BEGIN

namespace sensor {

enum BinarySensorMapType {
  BINARY_SENSOR_MAP_TYPE_GROUP,
};

struct BinarySensorMapChannel {
  binary_sensor::BinarySensor *binary_sensor;
  float sensor_value = 0;
};

/** Class to group binary_sensors to one Sensor.
 *
 * Each binary sensor represents a float value in the group.
 */
class BinarySensorMap : public Sensor, public Component {
 public:
  BinarySensorMap(const std::string &name);
  void dump_config() override;
  /**
   * The loop checks all binary_sensor states
   * When the binary_sensor reports a true value for its state, then the float value it represents is added to the
   * total_current_value
   *
   * Only when the  total_current_value changed and at least one sensor reports an active state we publish the sensors
   * average value. When the value changed and no sensors ar active we publish NAN.
   * */
  void loop() override;
  float get_setup_priority() const override;
  /** Add binary_sensors to the group.
   * Each binary_sensor represents a float value when its state is true
   *
   * @param *sensor The binary sensor.
   * @param value  The value this binary_sensor represents
   */
  void add_sensor(binary_sensor::BinarySensor *sensor, float value);
  void set_sensor_type(BinarySensorMapType sensor_type);

 protected:
  std::vector<BinarySensorMapChannel *> sensors_{};
  BinarySensorMapType sensor_type_{BINARY_SENSOR_MAP_TYPE_GROUP};
  bool last_any_active_{false};
  // this gives max 46 channels per binary_sensor_map
  uint64_t last_mask_{0x00};
  /**
   * methods to process the types of binary_sensor_maps
   * GROUP: process_group_() just map to a value
   * */
  void process_group_();
};

}  // namespace sensor

ESPHOME_NAMESPACE_END

#endif  // USE_BINARY_SENSOR_MAP

#endif  // ESPHOME_SENSOR_BINARY_SENSOR_MAP_H