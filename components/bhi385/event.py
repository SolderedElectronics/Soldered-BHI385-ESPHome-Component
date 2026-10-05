import esphome.codegen as cg
from esphome.components import event
import esphome.config_validation as cv
from esphome.const import CONF_TYPE

from . import CONF_BHI385_ID, BHI385Component

CODEOWNERS = ["@SolderedElectronics"]

TYPE_TAP = "tap"
TYPE_WRIST_GESTURE = "wrist_gesture"

# Event types fired by each entity, in the same order as the C++ side expects them.
EVENT_TYPES = {
    TYPE_TAP: ["single_tap", "double_tap", "triple_tap"],
    TYPE_WRIST_GESTURE: ["shake_jiggle", "flick_in", "flick_out"],
}


def _event_schema(icon):
    return event.event_schema(event.Event, icon=icon).extend(
        {
            cv.GenerateID(CONF_BHI385_ID): cv.use_id(BHI385Component),
        }
    )


CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_TAP: _event_schema("mdi:gesture-tap"),
        TYPE_WRIST_GESTURE: _event_schema("mdi:hand-wave"),
    },
    key=CONF_TYPE,
    lower=True,
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BHI385_ID])
    var = await event.new_event(config, event_types=EVENT_TYPES[config[CONF_TYPE]])
    cg.add(getattr(hub, f"set_{config[CONF_TYPE]}_event")(var))
