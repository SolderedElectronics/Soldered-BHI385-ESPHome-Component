# Bosch BHI385 SensorAPI (vendored unmodified) plus the Soldered C bridge the ESPHome component calls into.
# Built as its own ESP-IDF component so these files are compiled as C and stay out of ESPHome's generated esphome.h:
# bhi385_event_data_defs.h uses C99 designated array initializers that don't compile as C++.
idf_component_register(SRCS "bhi385.c"
                            "bhi385_hif.c"
                            "bhi385_event_data.c"
                            "bhi385_system_param.c"
                            "bhi385_virtual_sensor_info_param.c"
                            "bhi385_virtual_sensor_conf_param.c"
                            "bhi385_multi_tap_param.c"
                            "bhi385_activity_param.c"
                            "soldered_bhi385_bridge.c"
                       INCLUDE_DIRS ".")
