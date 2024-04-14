/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_behavior_sensor_push_key_press

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <drivers/behavior.h>

#include <zmk/behavior_queue.h>
#include <zmk/keymap.h>
#include <zmk/virtual_key_position.h>

LOG_MODULE_REGISTER(PUSH_KP, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define ZERO 0
#define POS 1
#define NEG -1

static int behavior_sensor_push_key_press_init(const struct device *dev) {
    LOG_DBG("init");
    return 0;
};

struct behavior_sensor_push_key_press_data {
    int32_t value[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
    int8_t state[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
};

struct behavior_sensor_push_key_press_config {
    int min_push;
    struct zmk_behavior_binding neg_binding;
    struct zmk_behavior_binding pos_binding;
};

int zmk_behavior_sensor_push_key_press_accept_data(
    struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event,
    const struct zmk_sensor_config *sensor_config, size_t channel_data_size,
    const struct zmk_sensor_channel_data *channel_data) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_sensor_push_key_press_data *data = dev->data;

    const struct sensor_value value = channel_data[0].value;
    int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    LOG_DBG("value: %d, old_state: %d, push negative keycode 0x%02X positive keycode 0x%02X",
            value.val1, data->state[sensor_index][event.layer], binding->param1, binding->param2);

    data->value[sensor_index][event.layer] = value.val1;
    return 0;
}

int zmk_behavior_sensor_push_key_press_process(struct zmk_behavior_binding *binding,
                                               struct zmk_behavior_binding_event event,
                                               enum behavior_sensor_binding_process_mode mode) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_sensor_push_key_press_config *cfg = dev->config;
    struct behavior_sensor_push_key_press_data *data = dev->data;

    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    if (mode != BEHAVIOR_SENSOR_BINDING_PROCESS_MODE_TRIGGER) {
        data->state[sensor_index][event.layer] = 0;
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    LOG_DBG("Sensor binding: %s", binding->behavior_dev);

    int32_t value = data->value[sensor_index][event.layer];
    int8_t old_state = data->state[sensor_index][event.layer];
    int8_t new_state = ZERO;
    if (old_state == ZERO && abs(value) < cfg->min_push) {
        // zero state.
    } else if (value > 0) {
        new_state = POS;
    } else if (value < 0) {
        new_state = NEG;
    }
    data->state[sensor_index][event.layer] = new_state;

    if (old_state == new_state) {
        LOG_DBG("keeping old state");
        return ZMK_BEHAVIOR_TRANSPARENT;
    } else {
        if (old_state != ZERO) {
            uint32_t key = old_state == POS ? binding->param2 : binding->param1;
            LOG_DBG("release: 0x%02X", key);
            struct zmk_behavior_binding binding =
                old_state == POS ? cfg->pos_binding : cfg->neg_binding;
            binding.param1 = key;
            behavior_keymap_binding_released(&binding, event);
        }
        if (new_state != ZERO) {
            uint32_t key = new_state == POS ? binding->param2 : binding->param1;
            LOG_DBG("press: 0x%02X", key);
            struct zmk_behavior_binding binding =
                new_state == POS ? cfg->pos_binding : cfg->neg_binding;
            binding.param1 = key;
            behavior_keymap_binding_pressed(&binding, event);
        }
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_sensor_push_key_press_driver_api = {
    .sensor_binding_accept_data = zmk_behavior_sensor_push_key_press_accept_data,
    .sensor_binding_process = zmk_behavior_sensor_push_key_press_process};

#define SENSOR_PUSH_KEY_PRESS_VAR_INST(n)                                                          \
    static struct behavior_sensor_push_key_press_config sensor_push_key_press_config_##n = {       \
        .neg_binding = {.behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 0))},   \
        .pos_binding = {.behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 1))},   \
        .min_push = DT_INST_PROP(n, min_push),                                                     \
    };                                                                                             \
    static struct behavior_sensor_push_key_press_data sensor_push_key_press_data_##n;              \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_sensor_push_key_press_init, NULL,                          \
                            &sensor_push_key_press_data_##n, &sensor_push_key_press_config_##n,    \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                      \
                            &behavior_sensor_push_key_press_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SENSOR_PUSH_KEY_PRESS_VAR_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */