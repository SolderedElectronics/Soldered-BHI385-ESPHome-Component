/**
 * @file soldered_bhi385_bridge.h
 * @brief C bridge between the bhi385 ESPHome component and the vendored Bosch BHI385 SensorAPI
 * @author Soldered Electronics
 *
 * Most Bosch headers end up including bhi385_event_data_defs.h, whose string tables use C99 designated array
 * initializers with gaps. Those are valid C but not C++ (GCC: "non-trivial designated initializers not supported"),
 * so the C++ component can't include them. This file only depends on bhi385_defs.h, which is C++ safe, and forwards
 * to the Bosch functions the component needs from a C translation unit. The vendored files stay unmodified.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bhi385_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/// BHI385_SENSOR_ID_MULTI_TAP, which is defined in a header C++ can't include.
extern const uint8_t SOLDERED_BHI385_SENSOR_ID_MULTI_TAP;

/// bhi385_hif_init() for I2C.
int8_t soldered_bhi385_hif_init(bhi385_read_fptr_t read, bhi385_write_fptr_t write, bhi385_delay_us_fptr_t delay_us,
                                uint32_t read_write_len, void *intf_ptr, struct bhi385_dev *dev);

int8_t soldered_bhi385_soft_reset(struct bhi385_dev *dev);
int8_t soldered_bhi385_get_chip_id(uint8_t *chip_id, struct bhi385_dev *dev);
int8_t soldered_bhi385_set_host_interrupt_ctrl(uint8_t hintr_ctrl, struct bhi385_dev *dev);
int8_t soldered_bhi385_set_host_intf_ctrl(uint8_t hintf_ctrl, struct bhi385_dev *dev);
int8_t soldered_bhi385_get_boot_status(uint8_t *boot_status, struct bhi385_dev *dev);

/// Uploads the bsxsam firmware image to program RAM and boots it.
int8_t soldered_bhi385_load_firmware(struct bhi385_dev *dev);
/// Size of the bsxsam firmware image in bytes.
uint32_t soldered_bhi385_firmware_size(void);

int8_t soldered_bhi385_get_kernel_version(uint16_t *kernel_version, struct bhi385_dev *dev);
int8_t soldered_bhi385_get_and_process_fifo(uint8_t *work_buffer, uint32_t buffer_size, struct bhi385_dev *dev);
int8_t soldered_bhi385_update_virtual_sensor_list(struct bhi385_dev *dev);
bool soldered_bhi385_is_sensor_available(uint8_t sensor_id, const struct bhi385_dev *dev);
int8_t soldered_bhi385_register_fifo_parse_callback(uint8_t sensor_id, bhi385_fifo_parse_callback_t callback,
                                                    void *callback_ref, struct bhi385_dev *dev);

/// Reads the current dynamic range of a physical sensor (g for the accelerometer, deg/s for the gyroscope).
int8_t soldered_bhi385_get_physical_sensor_range(uint8_t phys_sensor_id, uint16_t *range, struct bhi385_dev *dev);

/// Configures the multi-tap detector to report single, double and triple taps, like Bosch's multi_tap_param example.
int8_t soldered_bhi385_enable_all_taps(struct bhi385_dev *dev);

/// Maps a multi-tap FIFO payload to 0 (single), 1 (double) or 2 (triple) tap, -1 for anything else.
int soldered_bhi385_tap_index(const uint8_t *data);
/// Maps a wrist gesture FIFO payload to 0 (shake/jiggle), 1 (flick in) or 2 (flick out), -1 for anything else.
int soldered_bhi385_wrist_gesture_index(const uint8_t *data);

#ifdef __cplusplus
}
#endif
