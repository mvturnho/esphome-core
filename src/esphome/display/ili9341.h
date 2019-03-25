#ifndef ESPHOME_ILI9341_H
#define ESPHOME_ILI9341_H

#include "esphome/defines.h"

#ifdef USE_ILI9341

#include "esphome/spi_component.h"
#include "esphome/display/display.h"

ESPHOME_NAMESPACE_BEGIN

namespace display {
#define TFT_LED_PIN 32
#define TFT_DC_PIN 27
#define TFT_CS_PIN 14
#define TFT_MOSI_PIN 23
#define TFT_CLK_PIN 18
#define TFT_RST_PIN 33
#define TFT_MISO_PIN 19

#define BLK_PWM_CHANNEL 7

#define ILI9341_TFTWIDTH 240   ///< ILI9341 max TFT width
#define ILI9341_TFTHEIGHT 320  ///< ILI9341 max TFT height

#define ILI9341_NOP 0x00      ///< No-op register
#define ILI9341_SWRESET 0x01  ///< Software reset register
#define ILI9341_RDDID 0x04    ///< Read display identification information
#define ILI9341_RDDST 0x09    ///< Read Display Status

#define ILI9341_SLPIN 0x10   ///< Enter Sleep Mode
#define ILI9341_SLPOUT 0x11  ///< Sleep Out
#define ILI9341_PTLON 0x12   ///< Partial Mode ON
#define ILI9341_NORON 0x13   ///< Normal Display Mode ON

#define ILI9341_RDMODE 0x0A      ///< Read Display Power Mode
#define ILI9341_RDMADCTL 0x0B    ///< Read Display MADCTL
#define ILI9341_RDPIXFMT 0x0C    ///< Read Display Pixel Format
#define ILI9341_RDIMGFMT 0x0D    ///< Read Display Image Format
#define ILI9341_RDSELFDIAG 0x0F  ///< Read Display Self-Diagnostic Result

#define ILI9341_INVOFF 0x20    ///< Display Inversion OFF
#define ILI9341_INVON 0x21     ///< Display Inversion ON
#define ILI9341_GAMMASET 0x26  ///< Gamma Set
#define ILI9341_DISPOFF 0x28   ///< Display OFF
#define ILI9341_DISPON 0x29    ///< Display ON

#define ILI9341_CASET 0x2A  ///< Column Address Set
#define ILI9341_PASET 0x2B  ///< Page Address Set
#define ILI9341_RAMWR 0x2C  ///< Memory Write
#define ILI9341_RAMRD 0x2E  ///< Memory Read

#define ILI9341_PTLAR 0x30     ///< Partial Area
#define ILI9341_MADCTL 0x36    ///< Memory Access Control
#define ILI9341_VSCRSADD 0x37  ///< Vertical Scrolling Start Address
#define ILI9341_PIXFMT 0x3A    ///< COLMOD: Pixel Format Set

#define ILI9341_FRMCTR1 0xB1  ///< Frame Rate Control (In Normal Mode/Full Colors)
#define ILI9341_FRMCTR2 0xB2  ///< Frame Rate Control (In Idle Mode/8 colors)
#define ILI9341_FRMCTR3 0xB3  ///< Frame Rate control (In Partial Mode/Full Colors)
#define ILI9341_INVCTR 0xB4   ///< Display Inversion Control
#define ILI9341_DFUNCTR 0xB6  ///< Display Function Control

#define ILI9341_PWCTR1 0xC0  ///< Power Control 1
#define ILI9341_PWCTR2 0xC1  ///< Power Control 2
#define ILI9341_PWCTR3 0xC2  ///< Power Control 3
#define ILI9341_PWCTR4 0xC3  ///< Power Control 4
#define ILI9341_PWCTR5 0xC4  ///< Power Control 5
#define ILI9341_VMCTR1 0xC5  ///< VCOM Control 1
#define ILI9341_VMCTR2 0xC7  ///< VCOM Control 2

#define ILI9341_RDID1 0xDA  ///< Read ID 1
#define ILI9341_RDID2 0xDB  ///< Read ID 2
#define ILI9341_RDID3 0xDC  ///< Read ID 3
#define ILI9341_RDID4 0xDD  ///< Read ID 4

#define ILI9341_GMCTRP1 0xE0  ///< Positive Gamma Correction
#define ILI9341_GMCTRN1 0xE1  ///< Negative Gamma Correction

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
  ILI9341TypeA(SPIComponent *parent, GPIOPin *cs, GPIOPin *dc_pin, uint32_t update_interval);

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
