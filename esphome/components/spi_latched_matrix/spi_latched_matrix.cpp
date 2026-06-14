#include "spi_latched_matrix.h"

#include <algorithm>
#include <cstring>
#include <limits>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::spi_latched_matrix {

static const char *const TAG = "spi_latched_matrix";
static constexpr uint16_t INVALID_PIXEL_INDEX = std::numeric_limits<uint16_t>::max();

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
  if (this->gray_levels_ > 1) {
    this->pwm_buffers_[0] = std::make_unique<uint8_t[]>(this->transfer_buffer_size_ * this->gray_levels_);
    this->pwm_buffers_[1] = std::make_unique<uint8_t[]>(this->transfer_buffer_size_ * this->gray_levels_);
  }
  this->build_pixel_map_();
  this->display();
  if (this->gray_levels_ > 1) {
    this->high_freq_.start();
  }
}

void SPILatchedMatrix::dump_config() {
  ESP_LOGCONFIG(TAG,
                "SPI Latched Matrix:\n"
                "  Width: %d\n"
                "  Height: %d\n"
                "  Threshold: %u\n"
                "  Max Brightness: %u%%\n"
                "  Gray Levels: %u\n"
                "  Refresh Interval: %u us",
                this->width_, this->height_, this->threshold_,
                static_cast<unsigned>((this->max_brightness_ * 100U + 127U) / 255U), this->gray_levels_,
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

  this->write_next_pwm_frame_();
}

void SPILatchedMatrix::update() {
  this->do_update_();
  if (this->gray_levels_ > 1)
    this->build_pwm_frames_();
  else
    this->display();
}

void SPILatchedMatrix::display() {
  if (this->gray_levels_ > 1) {
    this->build_pwm_frames_();
    this->write_next_pwm_frame_();
  } else {
    this->render_(this->threshold_ - 1);
  }
}

void SPILatchedMatrix::fill(Color color) {
  if (this->buffer_ == nullptr)
    return;

  if (this->is_clipping() || this->get_rotation() != display::DISPLAY_ROTATION_0_DEGREES) {
    display::Display::fill(color);
    return;
  }

  std::memset(this->buffer_, this->color_to_grayscale_(color), this->width_ * this->height_);
}

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

      const int physical_index =
          this->pixel_map_ != nullptr ? this->pixel_map_[logical_index] : this->pixel_index_(x, y);
      if (physical_index < 0 || physical_index >= pixel_count)
        continue;

      this->transfer_buffer_[physical_index >> 3] |= 0x80U >> (physical_index & 7);
    }
  }

  this->write_frame_(this->transfer_buffer_.get());
}

void SPILatchedMatrix::write_next_pwm_frame_() {
  if (this->pwm_buffers_[this->active_pwm_buffer_] == nullptr || this->transfer_buffer_size_ == 0)
    return;

  this->write_frame_(this->pwm_buffers_[this->active_pwm_buffer_].get() +
                     this->pwm_phase_ * this->transfer_buffer_size_);
  this->pwm_phase_++;
  if (this->pwm_phase_ >= this->gray_levels_)
    this->pwm_phase_ = 0;
}

void SPILatchedMatrix::write_frame_(const uint8_t *frame) {
  if (frame == nullptr)
    return;

  // Keep the latch low while shifting, then raise it to present the new frame.
  this->latch_pin_->digital_write(false);
  this->enable();
  this->write_array(frame, this->transfer_buffer_size_);
  this->disable();
  this->pulse_latch_();
}

void HOT SPILatchedMatrix::draw_absolute_pixel_internal(int x, int y, Color color) {
  if (x < 0 || x >= this->width_ || y < 0 || y >= this->height_ || this->buffer_ == nullptr)
    return;

  this->buffer_[y * this->width_ + x] = this->color_to_grayscale_(color);
}

void SPILatchedMatrix::build_pixel_map_() {
  const uint32_t pixel_count = static_cast<uint32_t>(this->width_) * this->height_;
  if (pixel_count > INVALID_PIXEL_INDEX) {
    this->pixel_map_.reset();
    return;
  }

  this->pixel_map_ = std::make_unique<uint16_t[]>(pixel_count);
  for (int y = 0; y < this->height_; y++) {
    for (int x = 0; x < this->width_; x++) {
      const int logical_index = y * this->width_ + x;
      const int physical_index = this->pixel_index_(x, y);
      this->pixel_map_[logical_index] = physical_index >= 0 && physical_index < static_cast<int>(pixel_count)
                                            ? static_cast<uint16_t>(physical_index)
                                            : INVALID_PIXEL_INDEX;
    }
  }
}

void SPILatchedMatrix::build_pwm_frames_() {
  auto *build_buffer = this->pwm_buffers_[this->build_pwm_buffer_].get();
  if (this->buffer_ == nullptr || build_buffer == nullptr || this->transfer_buffer_size_ == 0)
    return;

  std::memset(build_buffer, 0, this->transfer_buffer_size_ * this->gray_levels_);
  const int pixel_count = this->width_ * this->height_;

  for (int y = 0; y < this->height_; y++) {
    for (int x = 0; x < this->width_; x++) {
      const int logical_index = y * this->width_ + x;
      const uint8_t brightness = this->buffer_[logical_index];
      if (brightness < this->threshold_)
        continue;

      const int physical_index =
          this->pixel_map_ != nullptr ? this->pixel_map_[logical_index] : this->pixel_index_(x, y);
      if (physical_index < 0 || physical_index >= pixel_count)
        continue;

      const uint16_t on_slots =
          std::max<uint16_t>(1, (static_cast<uint16_t>(brightness) * this->gray_levels_ + 127U) / 255U);
      for (uint8_t phase = 0; phase < this->gray_levels_; phase++) {
        const uint16_t current = static_cast<uint16_t>(phase) * on_slots / this->gray_levels_;
        const uint16_t next = static_cast<uint16_t>(phase + 1U) * on_slots / this->gray_levels_;
        if (current == next)
          continue;
        build_buffer[phase * this->transfer_buffer_size_ + (physical_index >> 3)] |= 0x80U >> (physical_index & 7);
      }
    }
  }

  std::swap(this->active_pwm_buffer_, this->build_pwm_buffer_);
}

int SPILatchedMatrix::pixel_index_(int x, int y) const {
  if (this->pixel_mapper_)
    return this->pixel_mapper_(x, y);
  return y * this->width_ + x;
}

uint8_t SPILatchedMatrix::color_to_grayscale_(Color color) const {
  const uint8_t brightness = std::max({color.red, color.green, color.blue, color.white});
  return static_cast<uint8_t>(brightness * this->max_brightness_ / 255U);
}

void SPILatchedMatrix::set_enable_(bool enable) {
  if (this->enable_pin_ == nullptr)
    return;
  this->enable_pin_->digital_write(enable != this->invert_enable_);
}

void SPILatchedMatrix::pulse_latch_() { this->latch_pin_->digital_write(true); }

}  // namespace esphome::spi_latched_matrix
