/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT toshiba_tb6612fng

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/drivers/pinctrl.h>

#include <zmk/drivers/tb6612fng.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tb6612fng, CONFIG_TB6612FNG_LOG_LEVEL);

/* device data structure */
struct tb6612fng_data {
    const struct device *dev;
    bool ready; // whether init is finished successfully    
    uint32_t velocity;
    bool inverse;
};

/* device config data structure */
struct tb6612fng_config {
    bool has_enable;
    const struct gpio_dt_spec enable;
    bool has_ain1;
    const struct gpio_dt_spec ain1;
    bool has_ain2;
    const struct gpio_dt_spec ain2;
    bool has_bin1;
    const struct gpio_dt_spec bin1;
    bool has_bin2;
    const struct gpio_dt_spec bin2;
    size_t pwms_len;
    struct pwm_dt_spec pwms[];
};

static int enable_set_value(const struct device *dev, uint8_t value) {
    const struct tb6612fng_config *config = dev->config;
    // struct tb6612fng_data *data = dev->data;

    if (config->has_enable) {
        if (gpio_pin_set_dt(&config->enable, value ? 1 : 0)) {
            LOG_WRN("Failed to set enable pin");
            return -EIO;
        }
    }

    return 0;
}

static int pwm_set_value(struct pwm_dt_spec pwm, uint8_t value) {
    if (!pwm_is_ready_dt(&pwm)) {
        LOG_WRN("PWM is not ready");
        return -EIO;
    }
    uint32_t period = pwm.period;
    if (value) {
        uint32_t pulse = period / 256.0 * value;
        LOG_DBG("set pwm: chan %d val %d period %d pulse %d", pwm.channel, value, period, pulse);
        pwm_set_dt(&pwm, period, pulse);
        // const struct pwm_dt_spec *spec = &pwm;
        // pwm_set(spec->dev, spec->channel, period, pulse, spec->flags);
    }
    else {
        LOG_DBG("set pwm: chan %d val %d", pwm.channel, value);
        pwm_set_dt(&pwm, period, 0);
    }
    return 0;
}

static int velocity_set_value(const struct device *dev, uint16_t vel, bool inv) {
    // const struct tb6612fng_config *config = dev->config;
    struct tb6612fng_data *data = dev->data;

    LOG_DBG("set velocity: %d  direction: [%s]", vel, inv ? "-" : "+");
    data->velocity = vel;
    data->inverse = inv;

    return 0;
}

static int sync_set_value(const struct device *dev, uint16_t chan) {
    const struct tb6612fng_config *config = dev->config;
    struct tb6612fng_data *data = dev->data;

    uint8_t in1 = !!(data->inverse) ? 1 : 0;
    uint8_t in2 =  !(data->inverse) ? 1 : 0;
    LOG_DBG("sync input: %d, %d", in1, in2);

    if (chan == 0) {
        if (!config->has_ain1 || !config->has_ain2) {
            LOG_WRN("Missing config of ain1 or ain2 pin");
            return -EIO;
        }
        if (gpio_pin_set_dt(&config->ain1, in1)) {
            LOG_WRN("Failed to set ain1 pin");
            return -EIO;
        }
        if (gpio_pin_set_dt(&config->ain2, in2)) {
            LOG_WRN("Failed to set ain2 pin");
            return -EIO;
        }
    }
    else if (chan == 1) {
        if (!config->has_bin1 || !config->has_bin2) {
            LOG_WRN("Missing config of bin1 or bin2 pin");
            return -EIO;
        }
        if (gpio_pin_set_dt(&config->bin1, in1)) {
            LOG_WRN("Failed to set bin1 pin");
            return -EIO;
        }
        if (gpio_pin_set_dt(&config->bin2, in2)) {
            LOG_WRN("Failed to set bin2 pin");
            return -EIO;
        }
    }

    size_t ch_offset = chan;
    if (ch_offset > config->pwms_len) {
        LOG_ERR("channel offset exceeds pwms length");
        return -EIO;
    }

    if (pwm_set_value(config->pwms[ch_offset], data->velocity)) {
        LOG_WRN("Failed to set PWM pin of ch: %d", chan);
        return -EIO;
    }

    return 0;
}

static int tb6612fng_init(const struct device *dev) {
    struct tb6612fng_data *data = dev->data;
    const struct tb6612fng_config *config = dev->config;
    int err;

    // init device pointer
    data->dev = dev;

    if (config->has_enable) {
        if (gpio_pin_configure_dt(&config->enable, GPIO_OUTPUT_INACTIVE)) {
            LOG_ERR("Failed to configure output enable pin");
            return -EIO;
        }
    }

    if (config->has_ain1) {
        if (gpio_pin_configure_dt(&config->ain1, GPIO_OUTPUT_INACTIVE)) {
            LOG_ERR("Failed to configure output ain1 pin");
            return -EIO;
        }
    }

    if (config->has_ain2) {
        if (gpio_pin_configure_dt(&config->ain2, GPIO_OUTPUT_INACTIVE)) {
            LOG_ERR("Failed to configure output ain2 pin");
            return -EIO;
        }
    }

    if (config->has_bin1) {
        if (gpio_pin_configure_dt(&config->bin1, GPIO_OUTPUT_INACTIVE)) {
            LOG_ERR("Failed to configure output bin1 pin");
            return -EIO;
        }
    }

    if (config->has_bin2) {
        if (gpio_pin_configure_dt(&config->bin2, GPIO_OUTPUT_INACTIVE)) {
            LOG_ERR("Failed to configure output bin2 pin");
            return -EIO;
        }
    }

    enable_set_value(dev, 0);
    velocity_set_value(dev, 0, true);
    if (config->pwms_len > 0) {
        sync_set_value(dev, 0);
    }
    if (config->pwms_len > 2) {
        sync_set_value(dev, 1);
    }

    data->ready = true;
    return err;
}

static int tb6612fng_attr_set(const struct device *dev, enum sensor_channel chan,
                            enum sensor_attribute attr, const struct sensor_value *val) {
    struct tb6612fng_data *data = dev->data;
    int err;

    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    switch ((uint32_t)attr) {
    case TB6612FNG_ATTR_ENABLE:
        err = enable_set_value(dev, TB6612FNG_SVALUE_TO_ENABLE(*val));
        break;

	case TB6612FNG_ATTR_VELOCITY:
        err = velocity_set_value(dev, TB6612FNG_SVALUE_TO_VELOCITY_VEL(*val),
                                      TB6612FNG_SVALUE_TO_VELOCITY_INV(*val));
        break;

	case TB6612FNG_ATTR_SYNC:
        err = sync_set_value(dev, TB6612FNG_SVALUE_TO_SYNC(*val));
        break;

    default:
        LOG_ERR("Unknown attribute");
        err = -ENOTSUP;
    }

    return err;
}

static const struct sensor_driver_api tb6612fng_driver_api = {
    .attr_set = tb6612fng_attr_set,
};

#define GET_PWM_SPEC(n, prop, i) PWM_DT_SPEC_GET_BY_IDX(n, i)

#define TB6612FNG_DEFINE(n)                                                    \
    static struct tb6612fng_data data##n;                                      \
    static const struct tb6612fng_config config##n = {                         \
        .has_enable = COND_CODE_1(                                             \
                        DT_INST_NODE_HAS_PROP(n, enable_gpios),                \
                        (true), (false)),                                      \
        .enable = COND_CODE_1(                                                 \
                    DT_INST_NODE_HAS_PROP(n, enable_gpios),                    \
                    (GPIO_DT_SPEC_INST_GET(n, enable_gpios)),                  \
                    (NULL)),                                                   \
        .has_ain1 = COND_CODE_1(                                               \
                        DT_INST_NODE_HAS_PROP(n, ain1_gpios),                  \
                        (true), (false)),                                      \
        .ain1 = COND_CODE_1(                                                   \
                    DT_INST_NODE_HAS_PROP(n, ain1_gpios),                      \
                    (GPIO_DT_SPEC_INST_GET(n, ain1_gpios)),                    \
                    (NULL)),                                                   \
        .has_ain2 = COND_CODE_1(                                               \
                        DT_INST_NODE_HAS_PROP(n, ain2_gpios),                  \
                        (true), (false)),                                      \
        .ain2 = COND_CODE_1(                                                   \
                    DT_INST_NODE_HAS_PROP(n, ain2_gpios),                      \
                    (GPIO_DT_SPEC_INST_GET(n, ain2_gpios)),                    \
                    (NULL)),                                                   \
        .has_bin1 = COND_CODE_1(                                               \
                        DT_INST_NODE_HAS_PROP(n, bin1_gpios),                  \
                        (true), (false)),                                      \
        .bin1 = COND_CODE_1(                                                   \
                    DT_INST_NODE_HAS_PROP(n, bin1_gpios),                      \
                    (GPIO_DT_SPEC_INST_GET(n, bin1_gpios)),                    \
                    (NULL)),                                                   \
        .has_bin2 = COND_CODE_1(                                               \
                        DT_INST_NODE_HAS_PROP(n, bin2_gpios),                  \
                        (true), (false)),                                      \
        .bin2 = COND_CODE_1(                                                   \
                    DT_INST_NODE_HAS_PROP(n, bin2_gpios),                      \
                    (GPIO_DT_SPEC_INST_GET(n, bin2_gpios)),                    \
                    (NULL)),                                                   \
        .pwms_len = DT_INST_PROP_LEN(n, pwms),                                 \
        .pwms = {DT_INST_FOREACH_PROP_ELEM_SEP(n, pwms, GET_PWM_SPEC, (, ))},  \
    };                                                                         \
    DEVICE_DT_INST_DEFINE(n, tb6612fng_init, DEVICE_DT_INST_GET(n),            \
                          &data##n, &config##n,                                \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,            \
                          &tb6612fng_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TB6612FNG_DEFINE)
