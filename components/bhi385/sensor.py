from pathlib import Path

import esphome.codegen as cg
from esphome.components import i2c, sensor
from esphome.components.esp32 import add_idf_component
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_SAMPLE_RATE,
    ICON_COUNTER,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_DEGREE_PER_SECOND,
    UNIT_DEGREES,
    UNIT_METER_PER_SECOND_SQUARED,
)

from . import BHI385Component

CODEOWNERS = ["@SolderedElectronics"]
DEPENDENCIES = ["i2c"]

CONF_ACCEL_X = "accel_x"
CONF_ACCEL_Y = "accel_y"
CONF_ACCEL_Z = "accel_z"
CONF_GYRO_X = "gyro_x"
CONF_GYRO_Y = "gyro_y"
CONF_GYRO_Z = "gyro_z"
CONF_QUATERNION_W = "quaternion_w"
CONF_QUATERNION_X = "quaternion_x"
CONF_QUATERNION_Y = "quaternion_y"
CONF_QUATERNION_Z = "quaternion_z"
CONF_ROLL = "roll"
CONF_PITCH = "pitch"
CONF_YAW = "yaw"
CONF_STEP_COUNT = "step_count"

ACCEL_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
    icon="mdi:axis-arrow",
    accuracy_decimals=3,
    state_class=STATE_CLASS_MEASUREMENT,
)
GYRO_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREE_PER_SECOND,
    icon="mdi:rotate-orbit",
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)
QUATERNION_SCHEMA = sensor.sensor_schema(
    icon="mdi:rotate-3d-variant",
    accuracy_decimals=4,
    state_class=STATE_CLASS_MEASUREMENT,
)
ANGLE_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREES,
    icon="mdi:angle-acute",
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
)

# (YAML key, schema); the C++ setter is set_<key>_sensor().
SENSORS = [
    (CONF_ACCEL_X, ACCEL_SCHEMA),
    (CONF_ACCEL_Y, ACCEL_SCHEMA),
    (CONF_ACCEL_Z, ACCEL_SCHEMA),
    (CONF_GYRO_X, GYRO_SCHEMA),
    (CONF_GYRO_Y, GYRO_SCHEMA),
    (CONF_GYRO_Z, GYRO_SCHEMA),
    (CONF_QUATERNION_W, QUATERNION_SCHEMA),
    (CONF_QUATERNION_X, QUATERNION_SCHEMA),
    (CONF_QUATERNION_Y, QUATERNION_SCHEMA),
    (CONF_QUATERNION_Z, QUATERNION_SCHEMA),
    (CONF_ROLL, ANGLE_SCHEMA),
    (CONF_PITCH, ANGLE_SCHEMA),
    (CONF_YAW, ANGLE_SCHEMA),
    (
        CONF_STEP_COUNT,
        sensor.sensor_schema(
            icon=ICON_COUNTER,
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
    ),
]

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BHI385Component),
            # Output data rate of the accelerometer, gyroscope and game rotation vector virtual sensors. The firmware
            # rounds it up to the next rate it supports (1.5625 Hz * 2^n).
            cv.Optional(CONF_SAMPLE_RATE, default=50.0): cv.float_range(min=1.0, max=400.0),
            **{cv.Optional(key): schema for key, schema in SENSORS},
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x29)),
    # The firmware image alone is ~80 KB and is uploaded in 256 byte I2C writes, which needs the ESP32's I2C driver.
    cv.only_on_esp32,
)


async def to_code(config):
    # The Bosch SensorAPI and the firmware image are built as a separate ESP-IDF component, see its CMakeLists.txt.
    add_idf_component(
        name="bhi385_sensorapi",
        path=str(Path(__file__).parent / "bhi385_sensorapi"),
    )

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))

    for key, _ in SENSORS:
        if conf := config.get(key):
            sens = await sensor.new_sensor(conf)
            cg.add(getattr(var, f"set_{key}_sensor")(sens))
