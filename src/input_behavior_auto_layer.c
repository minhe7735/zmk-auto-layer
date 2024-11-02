#define DT_DRV_COMPAT zmk_input_behavior_auto_layer

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>
#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/keycode_state_changed.h>

#define TOG_LAYER_MAX_EXCLUDED_POSITIONS 8

typedef struct {
  int32_t require_prior_idle_ms;
  uint32_t excluded_positions[TOG_LAYER_MAX_EXCLUDED_POSITIONS];
  uint8_t num_positions;
} behavior_auto_layer_config_t;

typedef struct {
  uint8_t toggle_layer;
  bool is_active;
  const struct device *dev;
  int64_t last_tapped_timestamp;
} behavior_auto_layer_data_t;

static inline bool is_position_excluded(const behavior_auto_layer_config_t *config, uint32_t position) {
  if (config->num_positions > TOG_LAYER_MAX_EXCLUDED_POSITIONS) {
    return false;
  }

  for (uint8_t i = 0; i < config->num_positions; i++) {
    if (config->excluded_positions[i] == position) {
      return true;
    }
  }
  return false;
}

static inline bool should_quick_tap(const behavior_auto_layer_config_t *config, 
                                    const behavior_auto_layer_data_t *data,
                                    int64_t timestamp) {
  return (data->last_tapped_timestamp + config->require_prior_idle_ms) > timestamp;
}

static int handle_position_state_changed(const zmk_event_t *eh) {
  const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
  const struct device *dev = DEVICE_DT_INST_GET(0);
  behavior_auto_layer_data_t *data = (behavior_auto_layer_data_t *)dev->data;
  const behavior_auto_layer_config_t *cfg = dev->config;

  if (ev->state && data->is_active && !is_position_excluded(cfg, ev->position)) {
    data->is_active = false;
    zmk_keymap_layer_deactivate(data->toggle_layer);
  }

  return ZMK_EV_EVENT_BUBBLE;
}

static int handle_keycode_state_changed(const zmk_event_t *eh) {
  const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
  const struct device *dev = DEVICE_DT_INST_GET(0);
  behavior_auto_layer_data_t *data = (behavior_auto_layer_data_t *)dev->data;

  if (ev->state && !is_mod(ev->usage_page, ev->keycode)) {
    data->last_tapped_timestamp = ev->timestamp;
  }
  return ZMK_EV_EVENT_BUBBLE;
}

static int auto_layer_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
  const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
  behavior_auto_layer_data_t *data = (behavior_auto_layer_data_t *)dev->data;
  const behavior_auto_layer_config_t *cfg = dev->config;

  data->toggle_layer = binding->param1;

  if (!data->is_active && !should_quick_tap(cfg, data, event.timestamp)) {
    data->is_active = true;
    zmk_keymap_layer_activate(data->toggle_layer);
  }

  return ZMK_BEHAVIOR_TRANSPARENT;
}

static int auto_layer_init(const struct device *dev) {
  behavior_auto_layer_data_t *data = dev->data;
  data->dev = dev;
  data->is_active = false;
  data->last_tapped_timestamp = 0;
  return 0;
}

static const struct behavior_driver_api auto_layer_driver_api = {
  .binding_pressed = auto_layer_binding_pressed,
};

ZMK_LISTENER(behavior_auto_layer, handle_position_state_changed);
ZMK_SUBSCRIPTION(behavior_auto_layer, zmk_position_state_changed);
ZMK_LISTENER(behavior_auto_layer_keycode, handle_keycode_state_changed);
ZMK_SUBSCRIPTION(behavior_auto_layer_keycode, zmk_keycode_state_changed);

#define TOG_LAYER_INST(n)                                                      \
static behavior_auto_layer_data_t behavior_auto_layer_data_##n = {};             \
static const behavior_auto_layer_config_t behavior_auto_layer_config_##n = {     \
  .require_prior_idle_ms = DT_PROP(DT_DRV_INST(0), require_prior_idle_ms),     \
  .excluded_positions = DT_INST_PROP(n, excluded_positions),                   \
  .num_positions = DT_INST_PROP_LEN(n, excluded_positions),                    \
};                                                                             \
BEHAVIOR_DT_INST_DEFINE(n, auto_layer_init, NULL,                               \
                        &behavior_auto_layer_data_##n,                          \
                        &behavior_auto_layer_config_##n,                        \
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,      \
                        &auto_layer_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TOG_LAYER_INST)
