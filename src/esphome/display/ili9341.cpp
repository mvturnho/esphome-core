#include "esphome/defines.h"

#ifdef USE_ILI9341

#include "esphome/display/ili9341.h"
#include "esphome/log.h"

ESPHOME_NAMESPACE_BEGIN

namespace display {

static const char *TAG = "display.ili9341";

// not in .text section since only 30 bytes
static const uint8_t FULL_UPDATE_LUT[30] = {0x02, 0x02, 0x01, 0x11, 0x12, 0x12, 0x22, 0x22, 0x66, 0x69,
                                            0x69, 0x59, 0x58, 0x99, 0x99, 0x88, 0x00, 0x00, 0x00, 0x00,
                                            0xF8, 0xB4, 0x13, 0x51, 0x35, 0x51, 0x51, 0x19, 0x01, 0x00};

static const uint8_t PARTIAL_UPDATE_LUT[30] = {0x10, 0x18, 0x18, 0x08, 0x18, 0x18, 0x08, 0x00, 0x00, 0x00,
                                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                               0x13, 0x14, 0x44, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void ILI9341::setup_pins_() {
  this->init_internal_(this->get_buffer_length_());
  this->dc_pin_->setup();  // OUTPUT
  this->dc_pin_->digital_write(false);
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();  // OUTPUT
    this->reset_pin_->digital_write(true);
  }
  if (this->busy_pin_ != nullptr) {
    this->busy_pin_->setup();  // INPUT
  }
  this->spi_setup();

  // Reset
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->digital_write(false);
    delay(200);
    this->reset_pin_->digital_write(true);
    delay(200);
  }
}
float ILI9341::get_setup_priority() const { return setup_priority::POST_HARDWARE; }
void ILI9341::command(uint8_t value) {
  this->start_command_();
  this->write_byte(value);
  this->end_command_();
}
void ILI9341::data(uint8_t value) {
  this->start_data_();
  this->write_byte(value);
  this->end_data_();
}
bool ILI9341::is_device_msb_first() { return true; }
bool ILI9341::wait_until_idle_() {
  if (this->busy_pin_ == nullptr) {
    return true;
  }

  const uint32_t start = millis();
  while (this->busy_pin_->digital_read()) {
    if (millis() - start > 1000) {
      ESP_LOGE(TAG, "Timeout while displaying image!");
      return false;
    }
    delay(10);
  }
  return true;
}
void ILI9341::set_reset_pin(const GPIOOutputPin &reset) { this->reset_pin_ = reset.copy(); }
void ILI9341::set_busy_pin(const GPIOInputPin &busy) { this->busy_pin_ = busy.copy(); }
void ILI9341::update() {
  this->do_update_();
  this->display();
}
void ILI9341::fill(int color) {
  // flip logic
  const uint8_t fill = color ? 0x00 : 0xFF;
  for (uint32_t i = 0; i < this->get_buffer_length_(); i++)
    this->buffer_[i] = fill;
}
void HOT ILI9341::draw_absolute_pixel_internal(int x, int y, int color) {
  if (x >= this->get_width_internal() || y >= this->get_height_internal() || x < 0 || y < 0)
    return;

  const uint32_t pos = (x + y * this->get_width_internal()) / 8u;
  const uint8_t subpos = x & 0x07;
  // flip logic
  if (!color)
    this->buffer_[pos] |= 0x80 >> subpos;
  else
    this->buffer_[pos] &= ~(0x80 >> subpos);
}
uint32_t ILI9341::get_buffer_length_() { return this->get_width_internal() * this->get_height_internal() / 8u; }
ILI9341::ILI9341(SPIComponent *parent, GPIOPin *cs, GPIOPin *dc_pin, uint32_t update_interval)
    : PollingComponent(update_interval), SPIDevice(parent, cs), dc_pin_(dc_pin) {}
bool ILI9341::is_device_high_speed() { return true; }
void ILI9341::start_command_() {
  this->dc_pin_->digital_write(false);
  this->enable();
}
void ILI9341::end_command_() { this->disable(); }
void ILI9341::start_data_() {
  this->dc_pin_->digital_write(true);
  this->enable();
}
void ILI9341::end_data_() { this->disable(); }

// ========================================================
//                          Type A
// ========================================================

void ILI9341TypeA::setup() {
  this->setup_pins_();

  //this->command(ILI9341_COMMAND_DRIVER_OUTPUT_CONTROL);
  this->data(this->get_height_internal() - 1);
  this->data((this->get_height_internal() - 1) >> 8);
  this->data(0x00);  // ? GD = 0, SM = 0, TB = 0

  //this->command(ILI9341_COMMAND_BOOSTER_SOFT_START_CONTROL);  // ?
  this->data(0xD7);
  this->data(0xD6);
  this->data(0x9D);

  //this->command(ILI9341_COMMAND_WRITE_VCOM_REGISTER);  // ?
  this->data(0xA8);

  //this->command(ILI9341_COMMAND_SET_DUMMY_LINE_PERIOD);  // ?
  this->data(0x1A);

  //this->command(ILI9341_COMMAND_SET_GATE_TIME);  // 2µs per row
  this->data(0x08);

  //this->command(ILI9341_COMMAND_DATA_ENTRY_MODE_SETTING);
  this->data(0x03);  // from top left to bottom right
}
void ILI9341TypeA::dump_config() {
  LOG_DISPLAY("", "Waveshare E-Paper", this);
  switch (this->model_) {
    case ILI9341_1_54_IN:
      ESP_LOGCONFIG(TAG, "  Model: 1.54in");
      break;
    case ILI9341_2_13_IN:
      ESP_LOGCONFIG(TAG, "  Model: 2.13in");
      break;
    case ILI9341_2_9_IN:
      ESP_LOGCONFIG(TAG, "  Model: 2.9in");
      break;
  }
  ESP_LOGCONFIG(TAG, "  Full Update Every: %u", this->full_update_every_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  LOG_UPDATE_INTERVAL(this);
}
void HOT ILI9341TypeA::display() {
  if (!this->wait_until_idle_()) {
    this->status_set_warning();
    return;
  }

  if (this->full_update_every_ >= 2) {
    bool prev_full_update = this->at_update_ == 1;
    bool full_update = this->at_update_ == 0;
    if (full_update != prev_full_update) {
      this->write_lut_(full_update ? FULL_UPDATE_LUT : PARTIAL_UPDATE_LUT);
    }
    this->at_update_ = (this->at_update_ + 1) % this->full_update_every_;
  }

  // Set x & y regions we want to write to (full)
  //this->command(ILI9341_COMMAND_SET_RAM_X_ADDRESS_START_END_POSITION);
  this->data(0x00);
  this->data((this->get_width_internal() - 1) >> 3);
  //this->command(ILI9341_COMMAND_SET_RAM_Y_ADDRESS_START_END_POSITION);
  this->data(0x00);
  this->data(0x00);
  this->data(this->get_height_internal() - 1);
  this->data((this->get_height_internal() - 1) >> 8);

  //this->command(ILI9341_COMMAND_SET_RAM_X_ADDRESS_COUNTER);
  this->data(0x00);
  //this->command(ILI9341_COMMAND_SET_RAM_Y_ADDRESS_COUNTER);
  this->data(0x00);
  this->data(0x00);

  if (!this->wait_until_idle_()) {
    this->status_set_warning();
    return;
  }

  //this->command(ILI9341_COMMAND_WRITE_RAM);
  this->start_data_();
  this->write_array(this->buffer_, this->get_buffer_length_());
  this->end_data_();

  //this->command(ILI9341_COMMAND_DISPLAY_UPDATE_CONTROL_2);
  this->data(0xC4);
  //this->command(ILI9341_COMMAND_MASTER_ACTIVATION);
  //this->command(ILI9341_COMMAND_TERMINATE_FRAME_READ_WRITE);

  this->status_clear_warning();
}
int ILI9341TypeA::get_width_internal() {
  switch (this->model_) {
    case ILI9341_1_54_IN:
      return 200;
    case ILI9341_2_13_IN:
      return 128;
    case ILI9341_2_9_IN:
      return 128;
  }
  return 0;
}
int ILI9341TypeA::get_height_internal() {
  switch (this->model_) {
    case ILI9341_1_54_IN:
      return 200;
    case ILI9341_2_13_IN:
      return 250;
    case ILI9341_2_9_IN:
      return 296;
  }
  return 0;
}
void ILI9341TypeA::write_lut_(const uint8_t *lut) {
  //this->command(ILI9341_COMMAND_WRITE_LUT_REGISTER);
  for (uint8_t i = 0; i < 30; i++)
    this->data(lut[i]);
}
ILI9341TypeA::ILI9341TypeA(SPIComponent *parent, GPIOPin *cs, GPIOPin *dc_pin,
                                           ILI9341TypeAModel model, uint32_t update_interval)
    : ILI9341(parent, cs, dc_pin, update_interval), model_(model) {}
void ILI9341TypeA::set_full_update_every(uint32_t full_update_every) {
  this->full_update_every_ = full_update_every;
}

}  // namespace display

ESPHOME_NAMESPACE_END

#endif  // USE_ILI9341
