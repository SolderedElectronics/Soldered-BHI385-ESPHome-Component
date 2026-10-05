/**
 * @file bhi385.h
 * @brief Public API for the bhi385 ESPHome sensor component
 * @author Soldered Electronics
 */

#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#ifdef USE_EVENT
#include "esphome/components/event/event.h"
#endif

#include "bhi385_defs.h"

namespace esphome {
namespace bhi385 {

/// Size of the buffer the Bosch API reads the FIFOs into, same as the Soldered ESP-IDF component.
static const size_t FIFO_WORK_BUFFER_SIZE = 2048;

/**
 * @brief Soldered BHI385 Smart IMU breakout board.
 *
 * Thin ESPHome wrapper around Bosch Sensortec's BHI385 SensorAPI, vendored unmodified in bhi385_sensorapi/ (same copy
 * the Soldered ESP-IDF component ships) and built as a separate ESP-IDF component. The Bosch calls go through
 * soldered_bhi385_bridge.c, see soldered_bhi385_bridge.h for why. The BHI385 has no non-volatile program memory, so
 * setup() uploads Bosch's bsxsam firmware image to its RAM on every boot.
 *
 * The FIFOs are drained in loop() so that tap and wrist gesture events fire as soon as they happen. Continuous
 * values (acceleration, angular rate, rotation, step count) are cached and published on the update interval. No INT
 * pin needs to be wired.
 */
class BHI385Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_accel_x_sensor(sensor::Sensor *sensor) { this->accel_x_sensor_ = sensor; }
  void set_accel_y_sensor(sensor::Sensor *sensor) { this->accel_y_sensor_ = sensor; }
  void set_accel_z_sensor(sensor::Sensor *sensor) { this->accel_z_sensor_ = sensor; }
  void set_gyro_x_sensor(sensor::Sensor *sensor) { this->gyro_x_sensor_ = sensor; }
  void set_gyro_y_sensor(sensor::Sensor *sensor) { this->gyro_y_sensor_ = sensor; }
  void set_gyro_z_sensor(sensor::Sensor *sensor) { this->gyro_z_sensor_ = sensor; }
  void set_quaternion_w_sensor(sensor::Sensor *sensor) { this->quaternion_w_sensor_ = sensor; }
  void set_quaternion_x_sensor(sensor::Sensor *sensor) { this->quaternion_x_sensor_ = sensor; }
  void set_quaternion_y_sensor(sensor::Sensor *sensor) { this->quaternion_y_sensor_ = sensor; }
  void set_quaternion_z_sensor(sensor::Sensor *sensor) { this->quaternion_z_sensor_ = sensor; }
  void set_roll_sensor(sensor::Sensor *sensor) { this->roll_sensor_ = sensor; }
  void set_pitch_sensor(sensor::Sensor *sensor) { this->pitch_sensor_ = sensor; }
  void set_yaw_sensor(sensor::Sensor *sensor) { this->yaw_sensor_ = sensor; }
  void set_step_count_sensor(sensor::Sensor *sensor) { this->step_count_sensor_ = sensor; }

#ifdef USE_EVENT
  void set_tap_event(event::Event *event) { this->tap_event_ = event; }
  void set_wrist_gesture_event(event::Event *event) { this->wrist_gesture_event_ = event; }
#endif

  /// Output data rate in Hz of the accelerometer, gyroscope and game rotation vector virtual sensors.
  void set_sample_rate(float sample_rate) { this->sample_rate_ = sample_rate; }

 protected:
  /// Read callback handed to the Bosch API.
  static BHI385_INTF_RET_TYPE i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr);
  /// Write callback handed to the Bosch API.
  static BHI385_INTF_RET_TYPE i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr);
  /// Delay callback handed to the Bosch API.
  static void delay_us(uint32_t period_us, void *intf_ptr);

  /// FIFO parse callbacks, registered with this component as their callback_ref.
  static void on_accel(const struct bhi385_fifo_parse_data_info *info, void *ref);
  static void on_gyro(const struct bhi385_fifo_parse_data_info *info, void *ref);
  static void on_rotation(const struct bhi385_fifo_parse_data_info *info, void *ref);
  static void on_step_count(const struct bhi385_fifo_parse_data_info *info, void *ref);
  static void on_tap(const struct bhi385_fifo_parse_data_info *info, void *ref);
  static void on_wrist_gesture(const struct bhi385_fifo_parse_data_info *info, void *ref);

  bool uses_accel_() const {
    return this->accel_x_sensor_ != nullptr || this->accel_y_sensor_ != nullptr || this->accel_z_sensor_ != nullptr;
  }
  bool uses_gyro_() const {
    return this->gyro_x_sensor_ != nullptr || this->gyro_y_sensor_ != nullptr || this->gyro_z_sensor_ != nullptr;
  }
  bool uses_rotation_() const {
    return this->quaternion_w_sensor_ != nullptr || this->quaternion_x_sensor_ != nullptr ||
           this->quaternion_y_sensor_ != nullptr || this->quaternion_z_sensor_ != nullptr ||
           this->roll_sensor_ != nullptr || this->pitch_sensor_ != nullptr || this->yaw_sensor_ != nullptr;
  }

  /// Uploads the firmware image to RAM and boots it.
  bool load_firmware_();
  /// Registers a FIFO parse callback for a virtual sensor and enables it at the given rate.
  bool enable_virtual_sensor_(uint8_t sensor_id, float sample_rate, bhi385_fifo_parse_callback_t callback,
                              const char *name);
  /// Reads the current dynamic range of a physical sensor (g for the accelerometer, deg/s for the gyroscope).
  bool read_physical_range_(uint8_t phys_sensor_id, float *range);
  /// Reads and parses everything waiting in the wake-up and non-wake-up FIFOs.
  void drain_fifo_();

  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *gyro_x_sensor_{nullptr};
  sensor::Sensor *gyro_y_sensor_{nullptr};
  sensor::Sensor *gyro_z_sensor_{nullptr};
  sensor::Sensor *quaternion_w_sensor_{nullptr};
  sensor::Sensor *quaternion_x_sensor_{nullptr};
  sensor::Sensor *quaternion_y_sensor_{nullptr};
  sensor::Sensor *quaternion_z_sensor_{nullptr};
  sensor::Sensor *roll_sensor_{nullptr};
  sensor::Sensor *pitch_sensor_{nullptr};
  sensor::Sensor *yaw_sensor_{nullptr};
  sensor::Sensor *step_count_sensor_{nullptr};

#ifdef USE_EVENT
  event::Event *tap_event_{nullptr};
  event::Event *wrist_gesture_event_{nullptr};
#endif

  float sample_rate_{50.0f};

  /// Latest samples, filled by the FIFO callbacks and published on update().
  float accel_[3]{};
  float gyro_[3]{};
  float quaternion_[4]{};  // w, x, y, z
  uint32_t step_count_{0};
  bool accel_valid_{false};
  bool gyro_valid_{false};
  bool rotation_valid_{false};
  bool step_count_valid_{false};

  float accel_range_g_{0.0f};
  float gyro_range_dps_{0.0f};
  uint16_t kernel_version_{0};

  struct bhi385_dev dev_ {};
  uint8_t fifo_work_buffer_[FIFO_WORK_BUFFER_SIZE];
};

}  // namespace bhi385
}  // namespace esphome
