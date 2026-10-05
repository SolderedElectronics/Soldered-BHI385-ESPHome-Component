/**
 * @file bhi385.cpp
 * @brief Implementation for the bhi385 ESPHome sensor component
 * @author Soldered Electronics
 */

#include "bhi385.h"

#include <cmath>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "bhi385_virtual_sensor_conf_param.h"
#include "soldered_bhi385_bridge.h"

namespace esphome {
namespace bhi385 {

static const char *const TAG = "bhi385";

/// Earth's standard gravity, used to convert g to m/s^2.
static const float GRAVITY_EARTH = 9.80665f;

/// Max bytes per I2C transaction the Bosch HIF layer chunks reads and writes into (ESP-IDF component's value).
static const uint32_t READ_WRITE_LEN = 256;

/// Fixed rates of the event-style virtual sensors, from Bosch's sensor_list.txt for the bsxsam image (min == max).
static const float STEP_COUNTER_RATE = 1.0f;
static const float MULTI_TAP_RATE = 1.5625f;
static const float WRIST_GESTURE_RATE = 1.0f;

/// Converts radians to degrees.
static const float RAD_TO_DEGREES = 180.0f / static_cast<float>(M_PI);

/// Raw values are signed 16 bit, scaled to the physical sensor's current dynamic range.
static const float RAW_FULL_SCALE = 32768.0f;
/// Quaternion components are Q14 fixed point.
static const float QUATERNION_SCALE = 16384.0f;

/// FIFO payload sizes, including the sensor ID byte.
static const uint8_t XYZ_EVENT_SIZE = 7;
static const uint8_t QUATERNION_EVENT_SIZE = 11;
static const uint8_t STEP_COUNT_EVENT_SIZE = 5;
static const uint8_t SINGLE_BYTE_EVENT_SIZE = 2;

/// Event types fired by the tap and wrist gesture events, indexed like soldered_bhi385_tap_index() and
/// soldered_bhi385_wrist_gesture_index(). Must match EVENT_TYPES in event.py.
static const char *const TAP_EVENT_TYPES[] = {"single_tap", "double_tap", "triple_tap"};
static const char *const WRIST_GESTURE_EVENT_TYPES[] = {"shake_jiggle", "flick_in", "flick_out"};

/* ---- Bosch API bus glue ---- */

BHI385_INTF_RET_TYPE BHI385Component::i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr) {
  auto *self = static_cast<BHI385Component *>(intf_ptr);
  return self->read_register(reg, data, len) == i2c::ERROR_OK ? 0 : -1;
}

BHI385_INTF_RET_TYPE BHI385Component::i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr) {
  auto *self = static_cast<BHI385Component *>(intf_ptr);
  // The Bosch HIF layer never asks for more than READ_WRITE_LEN at a time.
  if (len > READ_WRITE_LEN)
    return -1;

  uint8_t buf[1 + READ_WRITE_LEN];
  buf[0] = reg;
  std::memcpy(&buf[1], data, len);
  // The firmware upload is several hundred of these writes back to back, keep the watchdog fed.
  App.feed_wdt();
  return self->write(buf, len + 1) == i2c::ERROR_OK ? 0 : -1;
}

void BHI385Component::delay_us(uint32_t period_us, void *intf_ptr) {
  // The firmware boot sequence waits up to 50 ms at a time. Sleep for the millisecond part instead of busy-waiting.
  if (period_us >= 1000) {
    delay(period_us / 1000);
    App.feed_wdt();
    period_us %= 1000;
  }
  if (period_us > 0)
    delayMicroseconds(period_us);
}

/* ---- FIFO parse callbacks ---- */

void BHI385Component::on_accel(const struct bhi385_fifo_parse_data_info *info, void *ref) {
  auto *self = static_cast<BHI385Component *>(ref);
  if (info->data_size != XYZ_EVENT_SIZE)
    return;
  // Same layout bhi385_event_data_parse_xyz() reads: three little-endian int16.
  const uint8_t *data = info->data_ptr;
  const float scale = self->accel_range_g_ / RAW_FULL_SCALE * GRAVITY_EARTH;
  self->accel_[0] = BHI385_LE2S16(data) * scale;
  self->accel_[1] = BHI385_LE2S16(data + 2) * scale;
  self->accel_[2] = BHI385_LE2S16(data + 4) * scale;
  self->accel_valid_ = true;
}

void BHI385Component::on_gyro(const struct bhi385_fifo_parse_data_info *info, void *ref) {
  auto *self = static_cast<BHI385Component *>(ref);
  if (info->data_size != XYZ_EVENT_SIZE)
    return;
  const uint8_t *data = info->data_ptr;
  const float scale = self->gyro_range_dps_ / RAW_FULL_SCALE;
  self->gyro_[0] = BHI385_LE2S16(data) * scale;
  self->gyro_[1] = BHI385_LE2S16(data + 2) * scale;
  self->gyro_[2] = BHI385_LE2S16(data + 4) * scale;
  self->gyro_valid_ = true;
}

void BHI385Component::on_rotation(const struct bhi385_fifo_parse_data_info *info, void *ref) {
  auto *self = static_cast<BHI385Component *>(ref);
  if (info->data_size != QUATERNION_EVENT_SIZE)
    return;
  // Same layout bhi385_event_data_parse_quaternion() reads: little-endian int16 x, y, z, w, then u16 accuracy.
  const uint8_t *data = info->data_ptr;
  self->quaternion_[0] = BHI385_LE2S16(data + 6) / QUATERNION_SCALE;
  self->quaternion_[1] = BHI385_LE2S16(data) / QUATERNION_SCALE;
  self->quaternion_[2] = BHI385_LE2S16(data + 2) / QUATERNION_SCALE;
  self->quaternion_[3] = BHI385_LE2S16(data + 4) / QUATERNION_SCALE;
  self->rotation_valid_ = true;
}

void BHI385Component::on_step_count(const struct bhi385_fifo_parse_data_info *info, void *ref) {
  auto *self = static_cast<BHI385Component *>(ref);
  if (info->data_size != STEP_COUNT_EVENT_SIZE)
    return;
  // No dedicated parsing helper for this sensor, Bosch's own step_counter example reads the raw u32 directly.
  self->step_count_ = BHI385_LE2U32(info->data_ptr);
  self->step_count_valid_ = true;
}

void BHI385Component::on_tap(const struct bhi385_fifo_parse_data_info *info, void *ref) {
#ifdef USE_EVENT
  auto *self = static_cast<BHI385Component *>(ref);
  if (info->data_size != SINGLE_BYTE_EVENT_SIZE || self->tap_event_ == nullptr)
    return;
  const int index = soldered_bhi385_tap_index(info->data_ptr);
  if (index < 0) {
    ESP_LOGW(TAG, "Unknown tap type 0x%02X", info->data_ptr[0]);
    return;
  }
  self->tap_event_->trigger(TAP_EVENT_TYPES[index]);
#endif
}

void BHI385Component::on_wrist_gesture(const struct bhi385_fifo_parse_data_info *info, void *ref) {
#ifdef USE_EVENT
  auto *self = static_cast<BHI385Component *>(ref);
  if (info->data_size != SINGLE_BYTE_EVENT_SIZE || self->wrist_gesture_event_ == nullptr)
    return;
  const int index = soldered_bhi385_wrist_gesture_index(info->data_ptr);
  if (index < 0) {
    ESP_LOGW(TAG, "Unknown wrist gesture 0x%02X", info->data_ptr[0]);
    return;
  }
  self->wrist_gesture_event_->trigger(WRIST_GESTURE_EVENT_TYPES[index]);
#endif
}

/* ---- Component ---- */

void BHI385Component::setup() {
  ESP_LOGCONFIG(TAG, "Running setup");

  int8_t rslt = soldered_bhi385_hif_init(BHI385Component::i2c_read, BHI385Component::i2c_write,
                                         BHI385Component::delay_us, READ_WRITE_LEN, this, &this->dev_);
  if (rslt != BHI385_OK) {
    ESP_LOGE(TAG, "Bosch HIF init failed (%d)", rslt);
    this->mark_failed();
    return;
  }

  if (soldered_bhi385_soft_reset(&this->dev_) != BHI385_OK) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
    this->mark_failed();
    return;
  }

  uint8_t chip_id = 0;
  if (soldered_bhi385_get_chip_id(&chip_id, &this->dev_) != BHI385_OK) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
    this->mark_failed();
    return;
  }
  if (chip_id != BHI385_CHIP_ID) {
    ESP_LOGE(TAG, "Wrong chip ID: 0x%02X, expected 0x%02X", chip_id, BHI385_CHIP_ID);
    this->mark_failed();
    return;
  }

  // The status/debug FIFO isn't used. The wake-up and non-wake-up FIFOs are polled, the INT pin is left unused.
  if (soldered_bhi385_set_host_interrupt_ctrl(BHI385_ICTL_DISABLE_STATUS_FIFO | BHI385_ICTL_DISABLE_DEBUG,
                                              &this->dev_) != BHI385_OK ||
      soldered_bhi385_set_host_intf_ctrl(0, &this->dev_) != BHI385_OK) {
    ESP_LOGE(TAG, "Host interface configuration failed");
    this->mark_failed();
    return;
  }

  if (!this->load_firmware_()) {
    this->mark_failed();
    return;
  }

  // One FIFO drain plus a virtual sensor list update is what every Bosch example runs right after boot, before
  // enabling any virtual sensor.
  rslt = soldered_bhi385_get_and_process_fifo(this->fifo_work_buffer_, sizeof(this->fifo_work_buffer_), &this->dev_);
  if (rslt != BHI385_OK || soldered_bhi385_update_virtual_sensor_list(&this->dev_) != BHI385_OK) {
    ESP_LOGE(TAG, "Reading the virtual sensor list failed");
    this->mark_failed();
    return;
  }

  bool ok = true;
  if (this->uses_accel_()) {
    ok &= this->read_physical_range_(BHI385_PHYS_SENSOR_ID_ACCELEROMETER, &this->accel_range_g_) &&
          this->enable_virtual_sensor_(BHI385_SENSOR_ID_ACC, this->sample_rate_, BHI385Component::on_accel,
                                       "accelerometer");
  }
  if (this->uses_gyro_()) {
    ok &=
        this->read_physical_range_(BHI385_PHYS_SENSOR_ID_GYROSCOPE, &this->gyro_range_dps_) &&
        this->enable_virtual_sensor_(BHI385_SENSOR_ID_GYRO, this->sample_rate_, BHI385Component::on_gyro, "gyroscope");
  }
  if (this->uses_rotation_()) {
    ok &= this->enable_virtual_sensor_(BHI385_SENSOR_ID_GAMERV, this->sample_rate_, BHI385Component::on_rotation,
                                       "game rotation vector");
  }
  if (this->step_count_sensor_ != nullptr) {
    ok &= this->enable_virtual_sensor_(BHI385_SENSOR_ID_STC_LP, STEP_COUNTER_RATE, BHI385Component::on_step_count,
                                       "step counter");
  }
#ifdef USE_EVENT
  if (this->tap_event_ != nullptr) {
    // Detect every tap pattern the firmware supports, same as Bosch's multi_tap_param example.
    ok &= soldered_bhi385_enable_all_taps(&this->dev_) == BHI385_OK &&
          this->enable_virtual_sensor_(SOLDERED_BHI385_SENSOR_ID_MULTI_TAP, MULTI_TAP_RATE, BHI385Component::on_tap,
                                       "multi-tap detector");
  }
  if (this->wrist_gesture_event_ != nullptr) {
    ok &= this->enable_virtual_sensor_(BHI385_SENSOR_ID_WRIST_GEST_DETECT_LP_WU, WRIST_GESTURE_RATE,
                                       BHI385Component::on_wrist_gesture, "wrist gesture detector");
  }
#endif

  if (!ok)
    this->status_set_error(LOG_STR("Enabling one or more virtual sensors failed"));
}

bool BHI385Component::load_firmware_() {
  uint8_t boot_status = 0;
  if (soldered_bhi385_get_boot_status(&boot_status, &this->dev_) != BHI385_OK) {
    ESP_LOGE(TAG, "Reading boot status failed");
    return false;
  }
  if ((boot_status & BHI385_BST_HOST_INTERFACE_READY) == 0) {
    ESP_LOGE(TAG, "Host interface not ready for firmware upload (boot status 0x%02X)", boot_status);
    return false;
  }

  ESP_LOGD(TAG, "Uploading %" PRIu32 " byte firmware image", soldered_bhi385_firmware_size());
  const uint32_t start = millis();
  int8_t rslt = soldered_bhi385_load_firmware(&this->dev_);
  if (rslt != BHI385_OK) {
    ESP_LOGE(TAG, "Firmware upload or boot failed (%d)", rslt);
    return false;
  }
  if (soldered_bhi385_get_kernel_version(&this->kernel_version_, &this->dev_) != BHI385_OK ||
      this->kernel_version_ == 0) {
    ESP_LOGE(TAG, "Firmware did not boot");
    return false;
  }
  ESP_LOGD(TAG, "Firmware uploaded and booted in %" PRIu32 " ms", millis() - start);
  return true;
}

bool BHI385Component::enable_virtual_sensor_(uint8_t sensor_id, float sample_rate,
                                             bhi385_fifo_parse_callback_t callback, const char *name) {
  if (!soldered_bhi385_is_sensor_available(sensor_id, &this->dev_)) {
    ESP_LOGE(TAG, "The firmware has no %s (virtual sensor %u)", name, sensor_id);
    return false;
  }
  if (soldered_bhi385_register_fifo_parse_callback(sensor_id, callback, this, &this->dev_) != BHI385_OK) {
    ESP_LOGE(TAG, "Registering the %s callback failed", name);
    return false;
  }
  struct bhi385_virtual_sensor_conf_param_conf conf {};
  conf.sample_rate = sample_rate;
  conf.latency = 0;
  if (bhi385_virtual_sensor_conf_param_set_cfg(sensor_id, &conf, &this->dev_) != BHI385_OK) {
    ESP_LOGE(TAG, "Enabling the %s failed", name);
    return false;
  }
  return true;
}

bool BHI385Component::read_physical_range_(uint8_t phys_sensor_id, float *range) {
  uint16_t raw_range = 0;
  if (soldered_bhi385_get_physical_sensor_range(phys_sensor_id, &raw_range, &this->dev_) != BHI385_OK) {
    ESP_LOGE(TAG, "Reading physical sensor %u info failed", phys_sensor_id);
    return false;
  }
  *range = raw_range;
  return true;
}

void BHI385Component::drain_fifo_() {
  int8_t rslt =
      soldered_bhi385_get_and_process_fifo(this->fifo_work_buffer_, sizeof(this->fifo_work_buffer_), &this->dev_);
  if (rslt != BHI385_OK) {
    this->status_set_warning();
    ESP_LOGV(TAG, "Reading FIFO failed (%d)", rslt);
    return;
  }
  this->status_clear_warning();
}

void BHI385Component::loop() { this->drain_fifo_(); }

void BHI385Component::update() {
  if (this->accel_valid_) {
    if (this->accel_x_sensor_ != nullptr)
      this->accel_x_sensor_->publish_state(this->accel_[0]);
    if (this->accel_y_sensor_ != nullptr)
      this->accel_y_sensor_->publish_state(this->accel_[1]);
    if (this->accel_z_sensor_ != nullptr)
      this->accel_z_sensor_->publish_state(this->accel_[2]);
  }

  if (this->gyro_valid_) {
    if (this->gyro_x_sensor_ != nullptr)
      this->gyro_x_sensor_->publish_state(this->gyro_[0]);
    if (this->gyro_y_sensor_ != nullptr)
      this->gyro_y_sensor_->publish_state(this->gyro_[1]);
    if (this->gyro_z_sensor_ != nullptr)
      this->gyro_z_sensor_->publish_state(this->gyro_[2]);
  }

  if (this->rotation_valid_) {
    const float w = this->quaternion_[0];
    const float x = this->quaternion_[1];
    const float y = this->quaternion_[2];
    const float z = this->quaternion_[3];
    if (this->quaternion_w_sensor_ != nullptr)
      this->quaternion_w_sensor_->publish_state(w);
    if (this->quaternion_x_sensor_ != nullptr)
      this->quaternion_x_sensor_->publish_state(x);
    if (this->quaternion_y_sensor_ != nullptr)
      this->quaternion_y_sensor_->publish_state(y);
    if (this->quaternion_z_sensor_ != nullptr)
      this->quaternion_z_sensor_->publish_state(z);

    // Tait-Bryan angles (Z-Y-X order) from the unit quaternion.
    if (this->roll_sensor_ != nullptr)
      this->roll_sensor_->publish_state(atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD_TO_DEGREES);
    if (this->pitch_sensor_ != nullptr) {
      const float sin_pitch = clamp(2.0f * (w * y - z * x), -1.0f, 1.0f);
      this->pitch_sensor_->publish_state(asinf(sin_pitch) * RAD_TO_DEGREES);
    }
    if (this->yaw_sensor_ != nullptr)
      this->yaw_sensor_->publish_state(atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * RAD_TO_DEGREES);
  }

  if (this->step_count_valid_ && this->step_count_sensor_ != nullptr)
    this->step_count_sensor_->publish_state(this->step_count_);
}

void BHI385Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BHI385:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
    return;
  }
  ESP_LOGCONFIG(TAG,
                "  Firmware kernel version: %u\n"
                "  Sample rate: %.4f Hz",
                this->kernel_version_, this->sample_rate_);
  if (this->uses_accel_())
    ESP_LOGCONFIG(TAG, "  Accelerometer range: +/-%.0f g", this->accel_range_g_);
  if (this->uses_gyro_())
    ESP_LOGCONFIG(TAG, "  Gyroscope range: +/-%.0f deg/s", this->gyro_range_dps_);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Accel X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Accel Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Accel Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
  LOG_SENSOR("  ", "Quaternion W", this->quaternion_w_sensor_);
  LOG_SENSOR("  ", "Quaternion X", this->quaternion_x_sensor_);
  LOG_SENSOR("  ", "Quaternion Y", this->quaternion_y_sensor_);
  LOG_SENSOR("  ", "Quaternion Z", this->quaternion_z_sensor_);
  LOG_SENSOR("  ", "Roll", this->roll_sensor_);
  LOG_SENSOR("  ", "Pitch", this->pitch_sensor_);
  LOG_SENSOR("  ", "Yaw", this->yaw_sensor_);
  LOG_SENSOR("  ", "Step Count", this->step_count_sensor_);
#ifdef USE_EVENT
  LOG_EVENT("  ", "Tap", this->tap_event_);
  LOG_EVENT("  ", "Wrist Gesture", this->wrist_gesture_event_);
#endif
}

}  // namespace bhi385
}  // namespace esphome
