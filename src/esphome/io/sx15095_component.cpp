#include "esphome/defines.h"

#ifdef USE_SX1509

#include "esphome/io/sx1509_component.h"
#include "esphome/log.h"
#include "sx1509_registers.h"

ESPHOME_NAMESPACE_BEGIN

namespace io {

static const char *TAG = "io.SX1509";

SX1509Component::SX1509Component(I2CComponent *parent, uint8_t address) : Component(), I2CDevice(parent, address) {}

void SX1509Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SX1509...");
  ESP_LOGCONFIG(TAG, "    Address: 0x%02X", this->address_);

  // Software reset command sequence:
  this->write_byte(REG_RESET, 0x12);
  this->write_byte(REG_RESET, 0x34);

  uint16_t testRegisters;
  if(!this->read_byte_16(REG_INTERRUPT_MASK_A, &testRegisters)) {
    ESP_LOGE(TAG, "SX1509 not available under 0x%02X", this->address_);
    this->mark_failed();
    return;
  } else if (testRegisters == 0xFF00)	{
		// Set the clock to a default of 2MHz using internal
		clock(INTERNAL_CLOCK_2MHZ);
		configClock(oscSource, oscPinFunction, oscFreqOut, oscDivider);
		return 1;
	}
}
void SX1509Component::dump_config() {
  ESP_LOGCONFIG(TAG, "SX1509:");
  ESP_LOGCONFIG(TAG, "    Address: 0x%02X", this->address_);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with SX1509 failed!");
  }
}
bool SX1509Component::digital_read(uint8_t pin) {
  this->read_gpio_();
  return this->input_mask_ & (1 << pin);
}
void SX1509Component::digital_write(uint8_t pin, bool value) {
  if (value) {
    this->port_mask_ |= (1 << pin);
  } else {
    this->port_mask_ &= ~(1 << pin);
  }

  this->write_gpio_();
}
void SX1509Component::pin_mode(uint8_t pin, uint8_t mode) {
  switch (mode) {
    case SX1509_INPUT:
      this->ddr_mask_ &= ~(1 << pin);
      this->port_mask_ &= ~(1 << pin);
      break;
    case SX1509_INPUT_PULLUP:
      this->ddr_mask_ &= ~(1 << pin);
      this->port_mask_ |= (1 << pin);
      break;
    case SX1509_OUTPUT:
      this->ddr_mask_ |= (1 << pin);
      this->port_mask_ &= ~(1 << pin);
      break;
    default:
      break;
  }

  this->write_gpio_();
}

bool SX1509Component::read_gpio_() {
  if (this->is_failed())
    return false;

  uint8_t data;
  if (!this->parent_->raw_receive(this->address_, &data, 1)) {
    this->status_set_warning();
    return false;
  }
  this->input_mask_ = data;

  this->status_clear_warning();
  return true;
}

bool SX1509Component::write_gpio_() {
  if (this->is_failed())
    return false;

  uint16_t value = (this->input_mask_ & ~this->ddr_mask_) | this->port_mask_;

  this->parent_->raw_begin_transmission(this->address_);
  uint8_t data = value & 0xFF;
  this->parent_->raw_write(this->address_, &data, 1);

  if (!this->parent_->raw_end_transmission(this->address_)) {
    this->status_set_warning();
    return false;
  }
  this->status_clear_warning();
  return true;
}

SX1509GPIOInputPin SX1509Component::make_input_pin(uint8_t pin, uint8_t mode, bool inverted) {
  return {this, pin, mode, inverted};
}

SX1509GPIOOutputPin SX1509Component::make_output_pin(uint8_t pin, bool inverted) {
  return {this, pin, SX1509_OUTPUT, inverted};
}

float SX1509Component::get_setup_priority() const { return setup_priority::HARDWARE; }

void SX1509GPIOInputPin::setup() { this->pin_mode(this->mode_); }

bool SX1509GPIOInputPin::digital_read() { return this->parent_->digital_read(this->pin_) != this->inverted_; }

void SX1509GPIOInputPin::digital_write(bool value) {
  this->parent_->digital_write(this->pin_, value != this->inverted_);
}

SX1509GPIOInputPin::SX1509GPIOInputPin(SX1509Component *parent, uint8_t pin, uint8_t mode, bool inverted)
    : GPIOInputPin(pin, mode, inverted), parent_(parent) {}

GPIOPin *SX1509GPIOInputPin::copy() const { return new SX1509GPIOInputPin(*this); }

void SX1509GPIOInputPin::pin_mode(uint8_t mode) { this->parent_->pin_mode(this->pin_, mode); }

void SX1509GPIOOutputPin::setup() { this->pin_mode(this->mode_); }

bool SX1509GPIOOutputPin::digital_read() { return this->parent_->digital_read(this->pin_) != this->inverted_; }

void SX1509GPIOOutputPin::digital_write(bool value) {
  this->parent_->digital_write(this->pin_, value != this->inverted_);
}

SX1509GPIOOutputPin::SX1509GPIOOutputPin(SX1509Component *parent, uint8_t pin, uint8_t mode, bool inverted)
    : GPIOOutputPin(pin, mode, inverted), parent_(parent) {}

GPIOPin *SX1509GPIOOutputPin::copy() const { return new SX1509GPIOOutputPin(*this); }

void SX1509GPIOOutputPin::pin_mode(uint8_t mode) { this->parent_->pin_mode(this->pin_, mode); }

}  // namespace io

ESPHOME_NAMESPACE_END

#endif  // USE_SX1509
