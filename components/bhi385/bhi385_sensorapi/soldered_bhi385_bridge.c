/**
 * @file soldered_bhi385_bridge.c
 * @brief C bridge between the bhi385 ESPHome component and the vendored Bosch BHI385 SensorAPI
 * @author Soldered Electronics
 */

#include "soldered_bhi385_bridge.h"

#include "bhi385.h"
#include "bhi385_event_data.h"
#include "bhi385_hif.h"
#include "bhi385_multi_tap_param.h"
#include "bhi385_system_param.h"

// Bosch's bsxsam firmware: accelerometer + gyroscope with BSX sensor fusion (game rotation vector), plus the step
// counter, multi-tap and wrist gesture detectors. Same image the Soldered ESP-IDF component uses.
#include "Bosch_Shuttle3_BHI385_bsxsam.fw.h"

const uint8_t SOLDERED_BHI385_SENSOR_ID_MULTI_TAP = BHI385_SENSOR_ID_MULTI_TAP;

int8_t soldered_bhi385_hif_init(bhi385_read_fptr_t read, bhi385_write_fptr_t write, bhi385_delay_us_fptr_t delay_us,
                                uint32_t read_write_len, void *intf_ptr, struct bhi385_dev *dev) {
  return bhi385_hif_init(BHI385_I2C_INTERFACE, read, write, delay_us, read_write_len, intf_ptr, &dev->hif);
}

int8_t soldered_bhi385_soft_reset(struct bhi385_dev *dev) { return bhi385_soft_reset(dev); }

int8_t soldered_bhi385_get_chip_id(uint8_t *chip_id, struct bhi385_dev *dev) {
  return bhi385_get_chip_id(chip_id, dev);
}

int8_t soldered_bhi385_set_host_interrupt_ctrl(uint8_t hintr_ctrl, struct bhi385_dev *dev) {
  return bhi385_set_host_interrupt_ctrl(hintr_ctrl, dev);
}

int8_t soldered_bhi385_set_host_intf_ctrl(uint8_t hintf_ctrl, struct bhi385_dev *dev) {
  return bhi385_set_host_intf_ctrl(hintf_ctrl, dev);
}

int8_t soldered_bhi385_get_boot_status(uint8_t *boot_status, struct bhi385_dev *dev) {
  return bhi385_get_boot_status(boot_status, dev);
}

int8_t soldered_bhi385_load_firmware(struct bhi385_dev *dev) {
  int8_t rslt = bhi385_upload_firmware_to_ram(bhi385_firmware_image, sizeof(bhi385_firmware_image), dev);
  if (rslt != BHI385_OK)
    return rslt;
  return bhi385_boot_from_ram(dev);
}

uint32_t soldered_bhi385_firmware_size(void) { return sizeof(bhi385_firmware_image); }

int8_t soldered_bhi385_get_kernel_version(uint16_t *kernel_version, struct bhi385_dev *dev) {
  return bhi385_get_kernel_version(kernel_version, dev);
}

int8_t soldered_bhi385_get_and_process_fifo(uint8_t *work_buffer, uint32_t buffer_size, struct bhi385_dev *dev) {
  return bhi385_get_and_process_fifo(work_buffer, buffer_size, dev);
}

int8_t soldered_bhi385_update_virtual_sensor_list(struct bhi385_dev *dev) {
  return bhi385_update_virtual_sensor_list(dev);
}

bool soldered_bhi385_is_sensor_available(uint8_t sensor_id, const struct bhi385_dev *dev) {
  return bhi385_is_sensor_available(sensor_id, dev) != 0;
}

int8_t soldered_bhi385_register_fifo_parse_callback(uint8_t sensor_id, bhi385_fifo_parse_callback_t callback,
                                                    void *callback_ref, struct bhi385_dev *dev) {
  return bhi385_register_fifo_parse_callback(sensor_id, callback, callback_ref, dev);
}

int8_t soldered_bhi385_get_physical_sensor_range(uint8_t phys_sensor_id, uint16_t *range, struct bhi385_dev *dev) {
  struct bhi385_system_param_phys_sensor_info info;
  int8_t rslt = bhi385_system_param_get_physical_sensor_info(phys_sensor_id, &info, dev);
  if (rslt == BHI385_OK)
    *range = info.curr_range.u16_val;
  return rslt;
}

int8_t soldered_bhi385_enable_all_taps(struct bhi385_dev *dev) {
  bhi385_event_data_multi_tap tap_config = BHI385_TRIPLE_DOUBLE_SINGLE_TAP;
  return bhi385_multi_tap_param_set_config(&tap_config, dev);
}

int soldered_bhi385_tap_index(const uint8_t *data) {
  uint8_t tap = BHI385_NO_TAP;
  bhi385_event_data_multi_tap_parsing(data, &tap);
  // Each event reports exactly one tap pattern.
  switch (tap) {
    case BHI385_SINGLE_TAP:
      return 0;
    case BHI385_DOUBLE_TAP:
      return 1;
    case BHI385_TRIPLE_TAP:
      return 2;
    default:
      return -1;
  }
}

int soldered_bhi385_wrist_gesture_index(const uint8_t *data) {
  bhi385_event_data_wrist_gesture_detect_t gesture;
  bhi385_event_data_wrist_gesture_detect_parsing(data, &gesture);
  switch (gesture.wrist_gesture) {
    case BHI385_WRIST_SHAKE_JIGGLE:
      return 0;
    case BHI385_FLICK_IN:
      return 1;
    case BHI385_FLICK_OUT:
      return 2;
    default:
      return -1;
  }
}
