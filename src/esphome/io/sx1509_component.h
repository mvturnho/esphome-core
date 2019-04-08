#ifndef ESPHOME_SX1509_COMPONENT_H
#define ESPHOME_SX1509_COMPONENT_H

#include "esphome/defines.h"

#ifdef USE_SX1509

#include "esphome/component.h"
#include "esphome/esphal.h"
#include "esphome/i2c_component.h"

ESPHOME_NAMESPACE_BEGIN

namespace io {

#define INTERNAL_CLOCK_2MHZ	2

/// Modes for SX1509 pins
enum SX1509GPIOMode {
  SX1509_INPUT = INPUT,
  SX1509_INPUT_PULLUP = INPUT_PULLUP,
  SX1509_OUTPUT = OUTPUT,
};

class SX1509GPIOInputPin;
class SX1509GPIOOutputPin;

class SX1509Component : public Component, public I2CDevice {
 public:
  SX1509Component(I2CComponent *parent, uint8_t address);

  /** Make a GPIOPin that can be used in other components.
   *
   * Note that in some cases this component might not work with incompatible other integrations
   * because for performance reasons the values are only sent once every loop cycle in a batch.
   * For example, OneWire sensors are not supported.
   *
   * @param pin The pin number to use. 0-7 for SX1509, 0-15
   * @param mode The pin mode to use. Only supported ones are SX1509_INPUT, SX1509_INPUT_PULLUP.
   * @param inverted If the pin should invert all incoming and outgoing values.
   * @return An SX1509GPIOPin instance.
   */
  SX1509GPIOInputPin make_input_pin(uint8_t pin, uint8_t mode = SX1509_INPUT, bool inverted = false);

  /** Make a GPIOPin that can be used in other components.
   *
   * Note that in some cases this component might not work with incompatible other integrations
   * because for performance reasons the values are only sent once every loop cycle in a batch.
   * For example, OneWire sensors are not supported.
   *
   * @param pin The pin number to use. 0-7 for SX1509, 0-15
   * @param inverted If the pin should invert all incoming and outgoing values.
   * @return An SX1509GPIOPin instance.
   */
  SX1509GPIOOutputPin make_output_pin(uint8_t pin, bool inverted = false);

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  /// Check i2c availability and setup masks
  void setup() override;
  /// Helper function to read the value of a pin.
  bool digital_read(uint8_t pin);
  /// Helper function to write the value of a pin.
  void digital_write(uint8_t pin, bool value);
  /// Helper function to set the pin mode of a pin.
  void pin_mode(uint8_t pin, uint8_t mode);

  float get_setup_priority() const override;

  void dump_config() override;

 protected:
  bool read_gpio_();

  bool write_gpio_();

  uint16_t ddr_mask_{0x00};
  uint16_t input_mask_{0x00};
  uint16_t port_mask_{0x00};
};

/// Helper class to expose a SX1509 pin as an internal input GPIO pin.
class SX1509GPIOInputPin : public GPIOInputPin {
 public:
  SX1509GPIOInputPin(SX1509Component *parent, uint8_t pin, uint8_t mode, bool inverted = false);

  GPIOPin *copy() const override;

  void setup() override;
  void pin_mode(uint8_t mode) override;
  bool digital_read() override;
  void digital_write(bool value) override;

 protected:
  SX1509Component *parent_;
};

/// Helper class to expose a SX1509 pin as an internal output GPIO pin.
class SX1509GPIOOutputPin : public GPIOOutputPin {
 public:
  SX1509GPIOOutputPin(SX1509Component *parent, uint8_t pin, uint8_t mode, bool inverted = false);

  GPIOPin *copy() const override;

  void setup() override;
  void pin_mode(uint8_t mode) override;
  bool digital_read() override;
  void digital_write(bool value) override;

 protected:
  SX1509Component *parent_;
};

}  // namespace io

ESPHOME_NAMESPACE_END

#endif  // USE_SX1509

#endif  // ESPHOME_SX1509_COMPONENT_H
