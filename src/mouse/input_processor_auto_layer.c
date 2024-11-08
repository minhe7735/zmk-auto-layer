#define DT_DRV_COMPAT zmk_input_processor_auto_layer

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>
#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/keycode_state_changed.h>

LOG_MODULE_REGISTER(zmk_auto_layer, CONFIG_ZMK_LOG_LEVEL);

/* Constants and Types */
#define MAX_LAYERS ZMK_KEYMAP_LAYERS_LEN
#define SMALL_ARRAY_THRESHOLD 8

struct auto_layer_config {
  int32_t require_prior_idle_ms;
  const uint32_t *excluded_positions;
  size_t num_positions;
};

struct auto_layer_state {
  uint8_t toggle_layer;
  bool is_active;
  int64_t last_tapped_timestamp;
};

struct auto_layer_data {
  const struct device *dev;
  struct auto_layer_state state;
};

/* Static Work Queue Items */
static struct k_work_delayable layer_disable_works[MAX_LAYERS];

/* Optimized Position Search */
static inline bool position_is_excluded(const struct auto_layer_config *config, uint32_t position) {
  if (!config->excluded_positions || !config->num_positions) {
    return false;
  }

  // Use binary search for larger arrays
  if (config->num_positions > SMALL_ARRAY_THRESHOLD) {
    size_t left = 0;
    size_t right = config->num_positions - 1;

    while (left <= right) {
      size_t mid = (left + right) / 2;
      uint32_t mid_val = config->excluded_positions[mid];

      if (mid_val == position) {
        return true;
      }
      if (mid_val < position) {
        left = mid + 1;
      } else {
        right = mid - 1;
      }
    }
    return false;
  }

  // Linear search for small arrays (better cache locality)
  const uint32_t *end = config->excluded_positions + config->num_positions;
  for (const uint32_t *pos = config->excluded_positions; pos < end; pos++) {
    if (*pos == position) {
      return true;
    }
  }
  return false;
}

/* Timing Check */
static inline bool should_quick_tap(const struct auto_layer_config *config, 
                                    int64_t last_tapped, 
                                    int64_t current_time) {
  return (last_tapped + config->require_prior_idle_ms) > current_time;
}

/* Layer State Management */
static void update_layer_state(struct auto_layer_state *state, bool activate) {
  if (state->is_active == activate) {
    return;
  }

  state->is_active = activate;
  if (activate) {
    zmk_keymap_layer_activate(state->toggle_layer);
    LOG_DBG("Layer %d activated", state->toggle_layer);
  } else {
    zmk_keymap_layer_deactivate(state->toggle_layer);
    LOG_DBG("Layer %d deactivated", state->toggle_layer);
  }
}

/* Work Queue Callback */
static void layer_disable_callback(struct k_work *work) {
  struct k_work_delayable *d_work = k_work_delayable_from_work(work);
  int layer_index = ARRAY_INDEX(layer_disable_works, d_work);

  const struct device *dev = DEVICE_DT_INST_GET(0);
  struct auto_layer_data *data = (struct auto_layer_data *)dev->data;

  if (zmk_keymap_layer_active(layer_index)) {
    update_layer_state(&data->state, false);
  }
}

/* Event Handlers */
static int handle_position_state_changed(const zmk_event_t *eh) {
  const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
  if (!ev->state) {
    return ZMK_EV_EVENT_BUBBLE;
  }

  const struct device *dev = DEVICE_DT_INST_GET(0);
  struct auto_layer_data *data = (struct auto_layer_data *)dev->data;
  const struct auto_layer_config *cfg = dev->config;

  if (data->state.is_active && !position_is_excluded(cfg, ev->position)) {
    update_layer_state(&data->state, false);
  }

  return ZMK_EV_EVENT_BUBBLE;
}

static int handle_keycode_state_changed(const zmk_event_t *eh) {
  const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
  if (!ev->state || is_mod(ev->usage_page, ev->keycode)) {
    return ZMK_EV_EVENT_BUBBLE;
  }

  const struct device *dev = DEVICE_DT_INST_GET(0);
  struct auto_layer_data *data = (struct auto_layer_data *)dev->data;
  data->state.last_tapped_timestamp = ev->timestamp;

  return ZMK_EV_EVENT_BUBBLE;
}

/* Driver Implementation */
static int auto_layer_handle_event(const struct device *dev,
                                   struct input_event *event,
                                   uint32_t param1,
                                   uint32_t param2,
                                   struct zmk_input_processor_state *state) {
  if (param1 >= MAX_LAYERS) {
    LOG_ERR("Invalid layer index: %d", param1);
    return -EINVAL;
  }

  struct auto_layer_data *data = (struct auto_layer_data *)dev->data;
  const struct auto_layer_config *cfg = dev->config;

  data->state.toggle_layer = param1;

  if (!data->state.is_active && 
    !should_quick_tap(cfg, data->state.last_tapped_timestamp, k_uptime_get())) {
    update_layer_state(&data->state, true);
  }

  if (param2 > 0) {
    k_work_reschedule(&layer_disable_works[param1], K_MSEC(param2));
  }

  return 0;
}

static int auto_layer_init(const struct device *dev) {
  struct auto_layer_data *data = dev->data;
  data->dev = dev;
  data->state = (struct auto_layer_state){0};

  for (int i = 0; i < MAX_LAYERS; i++) {
    k_work_init_delayable(&layer_disable_works[i], layer_disable_callback);
  }

  LOG_INF("Auto layer processor initialized");
  return 0;
}

/* Driver API */
static const struct zmk_input_processor_driver_api auto_layer_driver_api = {
  .handle_event = auto_layer_handle_event,
};

/* Event Listeners */
ZMK_LISTENER(processor_auto_layer, handle_position_state_changed);
ZMK_SUBSCRIPTION(processor_auto_layer, zmk_position_state_changed);
ZMK_LISTENER(processor_auto_layer_keycode, handle_keycode_state_changed);
ZMK_SUBSCRIPTION(processor_auto_layer_keycode, zmk_keycode_state_changed);

/* Device Instantiation */
#define AUTO_LAYER_INST(n)                                                        \
static struct auto_layer_data processor_auto_layer_data_##n = {};            \
static const uint32_t excluded_positions_##n[] =                             \
  DT_INST_PROP(n, excluded_positions);                                     \
static const struct auto_layer_config processor_auto_layer_config_##n = {    \
.require_prior_idle_ms =                                                 \
DT_PROP(DT_DRV_INST(0), require_prior_idle_ms),                     \
.excluded_positions = excluded_positions_##n,                            \
.num_positions = DT_INST_PROP_LEN(n, excluded_positions),               \
    };                                                                          \
DEVICE_DT_INST_DEFINE(n,                                                    \
                      auto_layer_init,                                        \
                      NULL,                                                   \
                      &processor_auto_layer_data_##n,                         \
                      &processor_auto_layer_config_##n,                       \
                      POST_KERNEL,                                            \
                      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                    \
                      &auto_layer_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AUTO_LAYER_INST)
