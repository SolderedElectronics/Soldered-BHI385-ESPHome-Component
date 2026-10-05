import esphome.codegen as cg
from esphome.components import i2c

CODEOWNERS = ["@SolderedElectronics"]

CONF_BHI385_ID = "bhi385_id"

bhi385_ns = cg.esphome_ns.namespace("bhi385")
BHI385Component = bhi385_ns.class_("BHI385Component", cg.PollingComponent, i2c.I2CDevice)
