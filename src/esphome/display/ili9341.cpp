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

//here we decide how pixel data lands in the buffer.
//in dtsplay we should retreive as we stored the data.
void HOT ILI9341::draw_absolute_pixel_internal(int x, int y, int color) {
  if (x >= this->get_width_internal() || y >= this->get_height_internal() || x < 0 || y < 0)
    return;

  uint16_t pos = x + (y / 8) * this->get_width_internal();
  uint8_t subpos = y & 0x07;
  if (color) {
    this->buffer_[pos] |= (color << subpos);
  } else {
    this->buffer_[pos] &= ~(color << subpos);
  }
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

void ILI9341::set_address_(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
  this->command(0x2A);
  this->start_data_();
  this->write_byte(x1 >> 8);
  this->write_byte(x1);
  this->write_byte(x2 >> 8);
  this->write_byte(x2);
  this->end_data_();

  this->command(0x2B);
  this->start_data_();
  this->write_byte(y1 >> 8);
  this->write_byte(y1);
  this->write_byte(y2 >> 8);
  this->write_byte(y2);
  this->end_data_();

  this->command(0x2C);  // meory write
}

// ========================================================
//                          Type A
// ========================================================
ILI9341TypeA::ILI9341TypeA(SPIComponent *parent, GPIOPin *cs, GPIOPin *dc_pin, uint32_t update_interval)
    : ILI9341(parent, cs, dc_pin, update_interval) {}

void ILI9341TypeA::setup() {
  uint8_t cmd, x, numArgs;
  const uint8_t *addr = init_cmd;
  this->setup_pins_();

  // this->start_command_();
  if (this->reset_pin_ < 0) {
    this->command(ILI9341_SWRESET);
    delay(150);
  }
  while ((cmd = pgm_read_byte(addr++)) > 0) {
    this->command(cmd);
    x = pgm_read_byte(addr++);
    numArgs = x & 0x7F;
    this->start_data_();
    while (numArgs--) {
      this->write_byte(pgm_read_byte(addr++));
      if (x & 0x80)
        delay(150);
    }
    this->end_data_();
  }

  this->command(ILI9341_MADCTL);
  this->data(MADCTL_BGR);

  // ledcSetup(BLK_PWM_CHANNEL, 10000, 8);
  // ledcAttachPin(TFT_LED_PIN, BLK_PWM_CHANNEL);
  // ledcWrite(BLK_PWM_CHANNEL, 80);
}

void ILI9341TypeA::dump_config() {
  LOG_DISPLAY("", "ILI9341 ", this);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  LOG_UPDATE_INTERVAL(this);
}
void HOT ILI9341TypeA::display() {
  ESP_LOGD(TAG, "update display");
  ESP_LOGD(TAG, "width:%d height:%d", this->get_width_internal(), this->get_height_internal());
  this->fill(0x00);
  uint8_t red = 0xF8;
  this->line(10, 10, 100, 200, red);
  for (uint16_t x = 0; x < this->width_; x++) {
    this->set_address_(x, x, x + 1, x + 1);
    this->data(red >> 8);
    this->data(red);
    this->set_address_(x + 1, x, x + 2, x + 1);
    this->data(red >> 8);
    this->data(red);
  }

  // if (!this->wait_until_idle_()) {
    //panic result.
  //set the adress window to write to
  this->set_address_(0,0,this->get_width_internal(), this->get_height_internal());
  //start write ram command
  this->command(ILI9341_RAMWR); 
  //wrx < this->get_width_internal() / 16
  for (uint16_t y = 0; y < this->get_height_internal(); y++) {
    for (uint16_t x = 0; x < this->get_width_internal() / 16; x++) {
      uint16_t pixel_data = this->buffer_[x + (y * this->get_width_internal())];
      this->data(pixel_data >> 8);
      this->data(pixel_data);
      feed_wdt();
    }
  }

  ESP_LOGD(TAG, "done");

  this->status_clear_warning();
  return;
}
int ILI9341TypeA::get_width_internal() { return width_; }
int ILI9341TypeA::get_height_internal() { return height_; }

}  // namespace display

ESPHOME_NAMESPACE_END

#endif  // USE_ILI9341
