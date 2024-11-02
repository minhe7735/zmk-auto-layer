zephyr_library()

if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

  zephyr_library_sources_ifdef(CONFIG_ZMK_INPUT_BEHAVIOR_AUTO_LAYER src/input_behavior_auto_layer.c)

  zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
endif()
