# Soldered BHI385 Smart IMU ESPHome Component

| ![BHI385 Smart IMU Breakout](https://soldered.com/cdn/shop/files/333375_photo_7dd34a.webp?v=1785740755&width=3840) |
| :--------------------------------------------------------------------------------------------------------------: |
|                             [BHI385 Smart IMU Breakout](https://www.solde.red/333375)                             |

This 6-axis IMU pairs a 3-axis accelerometer and 3-axis gyroscope (up to ±28 g / ±2000 dps) with an onboard 32-bit ARC EM4 processor that runs sensor fusion locally, outputting higher-level features like 3D orientation, step counting, tap/gesture detection, and activity classification instead of raw data alone. It communicates over I2C at 3.3V, drawing under 500 µA for 50 Hz sensor fusion. Part of the [Qwiic ecosystem](https://soldered.com/collections/qwiic-ecosystem).

External ESPHome component for the Soldered BHI385 Smart IMU breakout. Communication is I2C only, default address `0x29`. **ESP32 family only** (any variant, `esp-idf` or `arduino` framework).

The BHI385 has no non-volatile program memory: on every boot the component uploads Bosch's ~85 KB `bsxsam` firmware image to the sensor over I2C, then enables only the virtual sensors your YAML uses. Upload time scales with the I2C clock: at `400kHz` it should take around two seconds, at ESPHome's ESP32 default of `50kHz` around eight times longer. Set `frequency: 400kHz` on the `i2c:` bus.

## Repository Contents

- **components/bhi385/** - the ESPHome external component (Python config + C++ implementation)
- **components/bhi385/bhi385_sensorapi/** - Bosch's BHI385 SensorAPI and firmware image (vendored unmodified) plus a small C bridge, built as a separate ESP-IDF component
- **examples/** - example YAML configs showing how to use the component

## Usage

Reference this repo directly from your own ESPHome YAML (no need to clone it locally):

```yaml
external_components:
  - source: github://SolderedElectronics/Soldered-BHI385-ESPHome-Component
    components: [bhi385]

i2c:
  sda: GPIO8
  scl: GPIO9
  frequency: 400kHz

sensor:
  - platform: bhi385
    update_interval: 1s
    accel_x:
      name: "BHI385 Accel X"
    accel_y:
      name: "BHI385 Accel Y"
    accel_z:
      name: "BHI385 Accel Z"
    roll:
      name: "BHI385 Roll"
    pitch:
      name: "BHI385 Pitch"
    step_count:
      name: "BHI385 Step Count"

event:
  - platform: bhi385
    type: tap
    name: "BHI385 Tap"
```

See [`examples/basic.yaml`](examples/basic.yaml) and [`examples/gestures.yaml`](examples/gestures.yaml) for full working examples.

### Configuration variables

The `sensor` platform entry owns the sensor configuration. Include it even if you only use events.

- **address** (*Optional*, int): I2C address of the sensor. Defaults to `0x29`.
- **update_interval** (*Optional*, [Time](https://esphome.io/guides/configuration-types#config-time)): how often the
  latest values are published. Defaults to `60s`.
- **sample_rate** (*Optional*, float): output data rate in Hz of the accelerometer, gyroscope and game rotation vector,
  `1`-`400`. The firmware rounds it up to the next rate it supports (1.5625 Hz × 2ⁿ). Defaults to `50`.
- **accel_x**, **accel_y**, **accel_z** (*Optional*): [Sensors](https://esphome.io/components/sensor/) publishing
  acceleration in m/s². Uses Bosch's corrected (calibrated) accelerometer output at the firmware's default range.
- **gyro_x**, **gyro_y**, **gyro_z** (*Optional*): [Sensors](https://esphome.io/components/sensor/) publishing angular
  rate in °/s. Uses Bosch's corrected (calibrated) gyroscope output at the firmware's default range.
- **quaternion_w**, **quaternion_x**, **quaternion_y**, **quaternion_z** (*Optional*):
  [Sensors](https://esphome.io/components/sensor/) publishing the game rotation vector, a unit quaternion fused from
  the accelerometer and gyroscope on the sensor itself.
- **roll**, **pitch**, **yaw** (*Optional*): [Sensors](https://esphome.io/components/sensor/) publishing the game
  rotation vector as Tait-Bryan angles in degrees (Z-Y-X order). The BHI385 has no magnetometer, so `yaw` is relative
  to the heading at boot and slowly drifts. `roll` and `pitch` are referenced to gravity and don't drift.
- **step_count** (*Optional*): [Sensor](https://esphome.io/components/sensor/) publishing the on-chip low-power step
  counter. The count goes back to 0 on every boot, because the firmware is reloaded during setup.

The `bhi385` component also provides an `event` platform:

- **platform: bhi385** (required)
- **type** (*Required*, string): one of
  - `tap`: fires `single_tap`, `double_tap` or `triple_tap`.
  - `wrist_gesture`: fires `shake_jiggle`, `flick_in` or `flick_out`. The gesture detector assumes the board is worn
    on the left wrist (firmware default).
- **name** (*Required*, string)
- All other options from [Event](https://esphome.io/components/event/), e.g. `on_event`.

The sensor's FIFOs are read continuously in the main loop, so events fire as soon as the sensor reports them,
independent of `update_interval`. Continuous values (acceleration, angular rate, orientation, step count) are cached
and the latest sample is published once per `update_interval`. No INT pin needs to be wired.

### Examples

- [`examples/basic.yaml`](examples/basic.yaml) - reads acceleration, angular rate, orientation (quaternion and
  roll/pitch/yaw) and step count.
- [`examples/gestures.yaml`](examples/gestures.yaml) - fires and logs tap and wrist gesture events.

The examples are written for an ESP32-S3 board (`esp32-s3-devkitc-1`) with the sensor on `GPIO8` (SDA) / `GPIO9` (SCL).

### Hardware design

You can find hardware design for this board in the _Soldered BHI385 Smart IMU Breakout_ hardware repository.

### Documentation

Access library documentation [here](https://docs.soldered.com/).

### About Soldered

<img src="https://raw.githubusercontent.com/SolderedElectronics/Soldered-Generic-Arduino-Library/dev/extras/Soldered-logo-color.png" alt="soldered-logo" width="500"/>

At Soldered, we design and manufacture a wide selection of electronic products to help you turn your ideas into acts and bring you one step closer to your final project. Our products are intented for makers and crafted in-house by our experienced team in Osijek, Croatia. We believe that sharing is a crucial element for improvement and innovation, and we work hard to stay connected with all our makers regardless of their skill or experience level. Therefore, all our products are open-source. Finally, we always have your back. If you face any problem concerning either your shopping experience or your electronics project, our team will help you deal with it, offering efficient customer service and cost-free technical support anytime. Some of those might be useful for you:

- [Web Store](https://www.soldered.com/shop)
- [Tutorials & Projects](https://soldered.com/learn)
- [Documentation](https://docs.soldered.com)

### Original source

This component is possible thanks to Bosch Sensortec's official [BHI385_SensorAPI](https://github.com/boschsensortec/BHI385_SensorAPI) and its `bsxsam` firmware image, vendored unmodified under `components/bhi385/bhi385_sensorapi/` from the [Soldered BHI385 ESP-IDF component](https://github.com/SolderedElectronics/Soldered-BHI385-ESP-IDF-Component) and distributed here under its original BSD-3-Clause license (see `components/bhi385/bhi385_sensorapi/BOSCH_BHI385_LICENSE`). Thank you, Bosch Sensortec.

### Open-source license

Soldered invests vast amounts of time into hardware & software for these products, which are all open-source. Please support future development by buying one of our products.

Check license details in the LICENSE file. Long story short, use these open-source files for any purpose you want to, as long as you apply the same open-source licence to it and disclose the original source. No warranty - all designs in this repository are distributed in the hope that they will be useful, but without any warranty. They are provided "AS IS", therefore without warranty of any kind, either expressed or implied. The entire quality and performance of what you do with the contents of this repository are your responsibility. In no event, Soldered (TAVU) will be liable for your damages, losses, including any general, special, incidental or consequential damage arising out of the use or inability to use the contents of this repository.

Code vendored under `components/bhi385/bhi385_sensorapi/` (except `soldered_bhi385_bridge.*`) remains Bosch Sensortec's own BSD-3-Clause licensed work - see `components/bhi385/bhi385_sensorapi/BOSCH_BHI385_LICENSE`.

## Have fun!

And thank you from your fellow makers at Soldered Electronics.
