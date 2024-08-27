/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_tb6612fng

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

#include <zmk/behavior.h>
#include <zmk/hid.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stdlib.h>

#include <zmk/drivers/tb6612fng.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// #if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct behavior_tb6612fng_config {
    const struct device *tb6612fng_dev;
    uint32_t vel_neutral;
    uint32_t vel_min_max;
};

struct behavior_tb6612fng_data {
    const struct device *dev;
};

static int tb6612fng_binding_pressed(struct zmk_behavior_binding *binding,
                                   struct zmk_behavior_binding_event event) {
    LOG_DBG("position %d tb6612fng: 0x%02X 0x%02X", event.position, binding->param1, binding->param2);
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    // struct behavior_tb6612fng_data *data = (struct behavior_tb6612fng_data *)dev->data;
    const struct behavior_tb6612fng_config *cfg = dev->config;

    int err = 0;
    const struct device *tb6612fng_dev = cfg->tb6612fng_dev;
    struct sensor_value val = { .val1 = 0, .val2 = 0 };

    uint32_t chan = binding->param1;
    int16_t vel = binding->param2 - cfg->vel_neutral;

    // crop min-max velocity value
    if (vel > 0 && vel > cfg->vel_min_max) {
        vel = cfg->vel_min_max;
    } else if (vel < 0 && vel < -cfg->vel_min_max) {
        vel = -cfg->vel_min_max;
    }

    // enable enable pin, disable iif vel == 0
    val.val1 = (!vel) ? 0 : 1;
    val.val2 = 0;
    err = sensor_attr_set(tb6612fng_dev, SENSOR_CHAN_ALL, 
                          (enum sensor_attribute) TB6612FNG_ATTR_ENABLE, &val);
    if (err) {
        LOG_WRN("Fail to sensor_attr_set TB6612FNG_ATTR_ENABLE");
        return -EIO;
    }

    // set velocity and inversr flag
    val.val1 = abs(vel);
    val.val2 = vel < 0;
    err = sensor_attr_set(tb6612fng_dev, SENSOR_CHAN_ALL, 
                          (enum sensor_attribute) TB6612FNG_ATTR_VELOCITY, &val);
    if (err) {
        LOG_WRN("Fail to sensor_attr_set TB6612FNG_ATTR_VELOCITY");
        return -EIO;
    }

    // call sync to latch velocity setting to driver
    //
    // ** NOTE**: val1 is ignored currently. only ONE channel is implemented.
    //
    val.val1 = chan;
    val.val2 = 0;
    err = sensor_attr_set(tb6612fng_dev, SENSOR_CHAN_ALL, 
                          (enum sensor_attribute) TB6612FNG_ATTR_SYNC, &val);
    if (err) {
        LOG_WRN("Fail to sensor_attr_set TB6612FNG_ATTR_SYNC");
        return -EIO;
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static int tb6612fng_binding_released(struct zmk_behavior_binding *binding,
                                    struct zmk_behavior_binding_event event) {
    LOG_DBG("position %d tb6612fng: 0x%02X 0x%02X", event.position, binding->param1, binding->param2);
    // const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int behavior_tb6612fng_init(const struct device *dev) {
    struct behavior_tb6612fng_data *data = dev->data;
    data->dev = dev;
    return 0;
};

static const struct behavior_driver_api behavior_tb6612fng_driver_api = {
    .binding_pressed = tb6612fng_binding_pressed,
    .binding_released = tb6612fng_binding_released,
};

#define ZMK_BEHAVIOR_TB6612FNG_PRIORITY 91

#define TB6612FNG_BEH_INST(n)                                               \
    static struct behavior_tb6612fng_data data_##n = {};                    \
    static struct behavior_tb6612fng_config config_##n = {                  \
        .tb6612fng_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, tb6612fng_dev)),  \
        .vel_neutral = DT_PROP(DT_DRV_INST(n), vel_neutral),                \
        .vel_min_max = DT_PROP(DT_DRV_INST(n), vel_min_max),                \
    };                                                                      \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_tb6612fng_init, NULL,               \
                            &data_##n, &config_##n,                         \
                            POST_KERNEL,                                    \
                            ZMK_BEHAVIOR_TB6612FNG_PRIORITY,                \
                            &behavior_tb6612fng_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TB6612FNG_BEH_INST)

// #endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
