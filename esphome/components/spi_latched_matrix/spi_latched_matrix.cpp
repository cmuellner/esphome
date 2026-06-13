#include "spi_latched_matrix.h"

#include <algorithm>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::spi_latched_matrix {

static const char *const TAG = "spi_latched_matrix";

void SPILatchedMatrix::setup() {
  this->spi_setup();

  this->latch_pin_->setup();
  this->latch_pin_->digital_write(false);

  if (this->enable_pin_ != nullptr) {
    this->enable_pin_->setup();
    this->set_enable_(true);
  }

  const uint32_t pixel_count = static_cast<uint32_t>(this->width_) * this->height_;
  this->init_internal_(pixel_count);
  this->transfer_buffer_size_ = (pixel_count + 7) / 8;
  this->transfer_buffer_ = std::make_unique<uint8_t[]>(this->transfer_buffer_size_);
  this->display();
  if (this->gray_levels_ > 1)
    this->high_freq_.start();
}

void SPILatchedMatrix::dump_config() {
  ESP_LOGCONFIG(TAG,
                "SPI Latched Matrix:\n"
                "  Width: %d\n"
                "  Height: %d\n"
                "  Threshold: %u\n"
                "  Gray Levels: %u\n"
                "  Refresh Interval: %u us",
                this->width_, this->height_, this->threshold_, this->gray_levels_,
                static_cast<unsigned>(this->refresh_interval_us_));
  LOG_PIN("  Latch Pin: ", this->latch_pin_);
  LOG_PIN("  Enable Pin: ", this->enable_pin_);
  ESP_LOGCONFIG(TAG, "  Inverted Enable: %s", YESNO(this->invert_enable_));
  LOG_UPDATE_INTERVAL(this);
  LOG_SPI_DEVICE(this);
}

void SPILatchedMatrix::loop() {
  if (this->gray_levels_ <= 1)
    return;

  const uint32_t now = micros();
  if (now - this->last_refresh_us_ < this->refresh_interval_us_)
    return;
  this->last_refresh_us_ = now;

  this->render_(this->pwm_counter_);

  const uint8_t step = std::max<uint8_t>(1, 256 / this->gray_levels_);
  this->pwm_counter_ += step;
}

void SPILatchedMatrix::update() {
  this->do_update_();
  if (this->gray_levels_ <= 1)
    this->display();
}

void SPILatchedMatrix::display() { this->render_(this->threshold_ - 1); }

void SPILatchedMatrix::render_(uint8_t pwm_threshold) {
  if (this->buffer_ == nullptr || this->transfer_buffer_ == nullptr || this->transfer_buffer_size_ == 0)
    return;

  std::memset(this->transfer_buffer_.get(), 0, this->transfer_buffer_size_);
  const int pixel_count = this->width_ * this->height_;

  // DisplayBuffer stores logical pixels; pixel_mapper_ converts them to the
  // bit positions expected by the external shift-register chain.
  for (int y = 0; y < this->height_; y++) {
    for (int x = 0; x < this->width_; x++) {
      const int logical_index = y * this->width_ + x;
      const uint8_t brightness = this->buffer_[logical_index];
      if (brightness < this->threshold_ || brightness <= pwm_threshold)
        continue;

      const int physical_index = this->pixel_index_(x, y);
      if (physical_index < 0 || physical_index >= pixel_count)
        continue;

      this->transfer_buffer_[physical_index >> 3] |= 0x80U >> (physical_index & 7);
    }
  }

  // Keep the latch low while shifting, then raise it to present the new frame.
  this->latch_pin_->digital_write(false);
  this->enable();
  this->write_array(this->transfer_buffer_.get(), this->transfer_buffer_size_);
  this->disable();
  this->pulse_latch_();
}

void HOT SPILatchedMatrix::draw_absolute_pixel_internal(int x, int y, Color color) {
  if (x < 0 || x >= this->width_ || y < 0 || y >= this->height_ || this->buffer_ == nullptr)
    return;

  this->buffer_[y * this->width_ + x] = this->color_to_grayscale_(color);
}

int SPILatchedMatrix::pixel_index_(int x, int y) const {
  if (this->pixel_mapper_)
    return this->pixel_mapper_(x, y);
  return y * this->width_ + x;
}

uint8_t SPILatchedMatrix::color_to_grayscale_(Color color) const {
  return std::max({color.red, color.green, color.blue, color.white});
}

void SPILatchedMatrix::set_enable_(bool enable) {
  if (this->enable_pin_ == nullptr)
    return;
  this->enable_pin_->digital_write(enable != this->invert_enable_);
}

void SPILatchedMatrix::pulse_latch_() { this->latch_pin_->digital_write(true); }

}  // namespace esphome::spi_latched_matrix
