#ifndef ESPHOME_ILI9341_H
#define ESPHOME_ILI9341_H

#include "esphome/defines.h"

#ifdef USE_ILI9341

#include "esphome/spi_component.h"
#include "esphome/display/display.h"

ESPHOME_NAMESPACE_BEGIN

namespace display {

class ILI9341 : public PollingComponent, public SPIDevice, public DisplayBuffer {
 public:
  ILI9341(SPIComponent *parent, GPIOPin *cs, GPIOPin *dc_pin, uint32_t update_interval);
  float get_setup_priority() const override;
  void set_reset_pin(const GPIOOutputPin &reset);
  void set_busy_pin(const GPIOInputPin &busy);

  bool is_device_msb_first() override;
  void command(uint8_t value);
  void data(uint8_t value);

  virtual void display() = 0;

  void update() override;

  void fill(int color) override;

 protected:
  void draw_absolute_pixel_internal(int x, int y, int color) override;

  bool wait_until_idle_();

  void setup_pins_();

  uint32_t get_buffer_length_();

  bool is_device_high_speed() override;

  void start_command_();
  void end_command_();
  void start_data_();
  void end_data_();

  GPIOPin *reset_pin_{nullptr};
  GPIOPin *dc_pin_;
  GPIOPin *busy_pin_{nullptr};
};

enum ILI9341TypeAModel {
  ILI9341_1_54_IN = 0,
  ILI9341_2_13_IN,
  ILI9341_2_9_IN,
};

class ILI9341TypeA : public ILI9341 {
 public:
  ILI9341TypeA(SPIComponent *parent, GPIOPin *cs, GPIOPin *dc_pin, ILI9341TypeAModel model,
                       uint32_t update_interval);

  void setup() override;

  void dump_config() override;

  void display() override;

  void set_full_update_every(uint32_t full_update_every);

 protected:
  void write_lut_(const uint8_t *lut);

  int get_width_internal() override;

  int get_height_internal() override;

  uint32_t full_update_every_{30};
  uint32_t at_update_{0};
  ILI9341TypeAModel model_;
};

}  // namespace display

ESPHOME_NAMESPACE_END

#endif  // USE_ILI9341

#endif  // ESPHOME_ILI9341_H
