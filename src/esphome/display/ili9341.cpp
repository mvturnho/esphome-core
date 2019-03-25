#include "esphome/defines.h"

#ifdef USE_ILI9341

#include "esphome/display/ili9341.h"
#include "esphome/log.h"

ESPHOME_NAMESPACE_BEGIN

namespace display {

static const char *TAG = "display.ili9341";

static const uint8_t PROGMEM init_cmd[] = {
    0xEF,
    3,
    0x03,
    0x80,
    0x02,
    0xCF,
    3,
    0x00,
    0xC1,
    0x30,
    0xED,
    4,
    0x64,
    0x03,
    0x12,
    0x81,
    0xE8,
    3,
    0x85,
    0x00,
    0x78,
    0xCB,
    5,
    0x39,
    0x2C,
    0x00,
    0x34,
    0x02,
    0xF7,
    1,
    0x20,
    0xEA,
    2,
    0x00,
    0x00,
    ILI9341_PWCTR1,
    1,
    0x23,  // Power control VRH[5:0]
    ILI9341_PWCTR2,
    1,
    0x10,  // Power control SAP[2:0];BT[3:0]
    ILI9341_VMCTR1,
    2,
    0x3e,
    0x28,  // VCM control
    ILI9341_VMCTR2,
    1,
    0x86,  // VCM control2
    ILI9341_MADCTL,
    1,
    0x48,  // Memory Access Control
    ILI9341_VSCRSADD,
    1,
    0x00,  // Vertical scroll zero
    ILI9341_PIXFMT,
    1,
    0x55,
    ILI9341_FRMCTR1,
    2,
    0x00,
    0x18,
    ILI9341_DFUNCTR,
    3,
    0x08,
    0x82,
    0x27,  // Display Function Control
    0xF2,
    1,
    0x00,  // 3Gamma Function Disable
    ILI9341_GAMMASET,
    1,
    0x01,  // Gamma curve selected
    ILI9341_GMCTRP1,
    15,
    0x0F,
    0x31,
    0x2B,
    0x0C,
    0x0E,
    0x08,  // Set Gamma
    0x4E,
    0xF1,
    0x37,
    0x07,
    0x10,
    0x03,
    0x0E,
    0x09,
    0x00,
    ILI9341_GMCTRN1,
    15,
    0x00,
    0x0E,
    0x14,
    0x03,
    0x11,
    0x07,  // Set Gamma
    0x31,
    0xC1,
    0x48,
    0x08,
    0x0F,
    0x0C,
    0x31,
    0x36,
    0x0F,
    ILI9341_SLPOUT,
    0x80,  // Exit Sleep
    ILI9341_DISPON,
    0x80,  // Display on
    0x00   // End of list
};

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
ILI9341TypeA::ILI9341TypeA(SPIComponent *parent, GPIOPin *cs, GPIOPin *dc_pin, uint32_t update_interval)
    : ILI9341(parent, cs, dc_pin, update_interval) {}
void ILI9341TypeA::set_full_update_every(uint32_t full_update_every) { this->full_update_every_ = full_update_every; }

void ILI9341TypeA::setup() {
  uint8_t cmd, x, numArgs;
  const uint8_t *addr = init_cmd;
  this->setup_pins_();

  this->start_command_();
  if (this->reset_pin_ < 0) {
    this->command(ILI9341_SWRESET);
    delay(150);
  }
  while ((cmd = pgm_read_byte(addr++)) > 0) {
    this->command(cmd);
    x = pgm_read_byte(addr++);
    numArgs = x & 0x7F;
    while (numArgs--) {
      this->data(pgm_read_byte(addr++));
      if (x & 0x80)
        delay(150);
    }
  }

  ledcSetup(BLK_PWM_CHANNEL, 10000, 8);
  ledcAttachPin(TFT_LED_PIN, BLK_PWM_CHANNEL);
  ledcWrite(BLK_PWM_CHANNEL, 80);

  draw_pixel_at(10,10,100);
  filled_circle(50,50,40,COLOR_ON);
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
    // if (full_update != prev_full_update) {
    //   this->write_lut_(full_update ? FULL_UPDATE_LUT : PARTIAL_UPDATE_LUT);
    // }
    this->at_update_ = (this->at_update_ + 1) % this->full_update_every_;
  }

  // Set x & y regions we want to write to (full)
  // this->command(ILI9341_COMMAND_SET_RAM_X_ADDRESS_START_END_POSITION);
  this->data(0x00);
  this->data((this->get_width_internal() - 1) >> 3);
  // this->command(ILI9341_COMMAND_SET_RAM_Y_ADDRESS_START_END_POSITION);
  this->data(0x00);
  this->data(0x00);
  this->data(this->get_height_internal() - 1);
  this->data((this->get_height_internal() - 1) >> 8);

  // this->command(ILI9341_COMMAND_SET_RAM_X_ADDRESS_COUNTER);
  this->data(0x00);
  // this->command(ILI9341_COMMAND_SET_RAM_Y_ADDRESS_COUNTER);
  this->data(0x00);
  this->data(0x00);

  if (!this->wait_until_idle_()) {
    this->status_set_warning();
    return;
  }

  // this->command(ILI9341_COMMAND_WRITE_RAM);
  this->start_data_();
  this->write_array(this->buffer_, this->get_buffer_length_());
  this->end_data_();

  // this->command(ILI9341_COMMAND_DISPLAY_UPDATE_CONTROL_2);
  this->data(0xC4);
  // this->command(ILI9341_COMMAND_MASTER_ACTIVATION);
  // this->command(ILI9341_COMMAND_TERMINATE_FRAME_READ_WRITE);

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
  // this->command(ILI9341_COMMAND_WRITE_LUT_REGISTER);
  for (uint8_t i = 0; i < 30; i++)
    this->data(lut[i]);
}

}  // namespace display

ESPHOME_NAMESPACE_END

#endif  // USE_ILI9341
