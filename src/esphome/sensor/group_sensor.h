#ifndef ESPHOME_SENSOR_GROUP_SENSOR_COMPONENT_H
#define ESPHOME_SENSOR_GROUP_SENSOR_COMPONENT_H

#include "esphome/defines.h"

#ifdef USE_GROUP_SENSOR

#include "esphome/component.h"
#include "esphome/sensor/sensor.h"
#include "esphome/binary_sensor/binary_sensor.h"

ESPHOME_NAMESPACE_BEGIN

namespace sensor {

class BinarySensorChannel {
  public:
  BinarySensorChannel(const binary_sensor::BinarySensor *sensor, float value);
   binary_sensor::BinarySensor *binary_sensor;
   float value;
};


/// This class lets you add binarysensors to a group.
/// eacht binary sensor is the associated with a float value.
class GroupSensorComponent : public Sensor , public Component {
 public:
  GroupSensorComponent(const std::string &name);

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override;

  void add_sensor(binary_sensor::BinarySensor *sensor, float value);
 protected:
  std::vector<BinarySensorChannel *> sensors_{};
};

}  // namespace sensor

ESPHOME_NAMESPACE_END

#endif  // GROUP_SENSOR

#endif  // ESPHOME_SENSOR_GROUP_SENSOR_COMPONENT_H
