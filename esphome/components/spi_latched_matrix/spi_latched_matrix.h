#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <utility>

#include "esphome/components/display/display_buffer.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/color.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"

namespace esphome::spi_latched_matrix {

class SPILatchedMatrix : public display::DisplayBuffer,
                         public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                               spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_10MHZ> {
 public:
  SPILatchedMatrix(int width, int height) : width_(width), height_(height) {}

  void set_latch_pin(GPIOPin *latch_pin) { this->latch_pin_ = latch_pin; }
  void set_enable_pin(GPIOPin *enable_pin) { this->enable_pin_ = enable_pin; }
  void set_invert_enable(bool invert_enable) { this->invert_enable_ = invert_enable; }
  void set_threshold(uint8_t threshold) { this->threshold_ = threshold; }
  void set_gray_levels(uint8_t gray_levels) { this->gray_levels_ = gray_levels; }
  void set_refresh_interval_us(uint32_t refresh_interval_us) { this->refresh_interval_us_ = refresh_interval_us; }
  void set_pixel_mapper(std::function<int(int, int)> &&pixel_mapper) { this->pixel_mapper_ = std::move(pixel_mapper); }

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void display();
  display::DisplayType get_display_type() override { return display::DISPLAY_TYPE_GRAYSCALE; }

 protected:
  int get_width_internal() override { return this->width_; }
  int get_height_internal() override { return this->height_; }
  void draw_absolute_pixel_internal(int x, int y, Color color) override;

  int pixel_index_(int x, int y) const;
  uint8_t color_to_grayscale_(Color color) const;
  void render_(uint8_t pwm_threshold);
  void set_enable_(bool enable);
  void pulse_latch_();

  int width_;
  int height_;
  GPIOPin *latch_pin_{nullptr};
  GPIOPin *enable_pin_{nullptr};
  bool invert_enable_{false};
  uint8_t threshold_{1};
  uint8_t gray_levels_{1};
  uint32_t refresh_interval_us_{0};
  uint32_t last_refresh_us_{0};
  uint8_t pwm_counter_{0};
  HighFrequencyLoopRequester high_freq_;
  std::function<int(int, int)> pixel_mapper_{};
  std::unique_ptr<uint8_t[]> transfer_buffer_;
  size_t transfer_buffer_size_{0};
};

}  // namespace esphome::spi_latched_matrix
