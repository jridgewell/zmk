/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/dt-bindings/adc/adc.h>
#define DT_DRV_COMPAT ck_thb

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(THB, CONFIG_SENSOR_LOG_LEVEL);

#ifdef CONFIG_ADC_NRFX_SAADC
#define ADC_INPUT_POS_OFFSET SAADC_CH_PSELP_PSELP_AnalogInput0
#else
#define ADC_INPUT_POS_OFFSET 0
#endif

struct thb_config {
    // NOTE: we are assuming both channels using the same ADC, this should hold
    // for pretty much all use cases
    uint8_t channel_x;
    uint8_t channel_y;

    uint32_t min_mv;
    uint32_t max_mv;
    uint32_t freq;
};

struct thb_data {
    const struct device *adc;
    struct adc_sequence as;
    int16_t xy_raw[2];
#ifdef CONFIG_JOYSTICK_THB_TRIGGER
    const struct device *dev;
    sensor_trigger_handler_t trigger_handler;
    const struct sensor_trigger *trigger;
    int32_t timer_freq;
    struct k_timer timer;
    struct k_work work;
#endif
};

#ifdef CONFIG_JOYSTICK_THB_TRIGGER_DEDICATED_QUEUE
K_THREAD_STACK_DEFINE(thb_trigger_stack_area, CONFIG_THB_WORKQUEUE_STACK_SIZE);
static struct k_work_q thb_work_q;
static bool is_thb_work_q_ready = false;
#endif // CONFIG_JOYSTICK_THB_TRIGGER
       //
static char *sensor_channel_name(enum sensor_channel chan) {
    switch (chan) {
    case SENSOR_CHAN_ACCEL_X:
        return "SENSOR_CHAN_ACCEL_X";
    case SENSOR_CHAN_ACCEL_Y:
        return "SENSOR_CHAN_ACCEL_Y";
    case SENSOR_CHAN_ACCEL_Z:
        return "SENSOR_CHAN_ACCEL_Z";
    case SENSOR_CHAN_ACCEL_XYZ:
        return "SENSOR_CHAN_ACCEL_XYZ";
    case SENSOR_CHAN_GYRO_X:
        return "SENSOR_CHAN_GYRO_X";
    case SENSOR_CHAN_GYRO_Y:
        return "SENSOR_CHAN_GYRO_Y";
    case SENSOR_CHAN_GYRO_Z:
        return "SENSOR_CHAN_GYRO_Z";
    case SENSOR_CHAN_GYRO_XYZ:
        return "SENSOR_CHAN_GYRO_XYZ";
    case SENSOR_CHAN_MAGN_X:
        return "SENSOR_CHAN_MAGN_X";
    case SENSOR_CHAN_MAGN_Y:
        return "SENSOR_CHAN_MAGN_Y";
    case SENSOR_CHAN_MAGN_Z:
        return "SENSOR_CHAN_MAGN_Z";
    case SENSOR_CHAN_MAGN_XYZ:
        return "SENSOR_CHAN_MAGN_XYZ";
    case SENSOR_CHAN_DIE_TEMP:
        return "SENSOR_CHAN_DIE_TEMP";
    case SENSOR_CHAN_AMBIENT_TEMP:
        return "SENSOR_CHAN_AMBIENT_TEMP";
    case SENSOR_CHAN_PRESS:
        return "SENSOR_CHAN_PRESS";
    case SENSOR_CHAN_PROX:
        return "SENSOR_CHAN_PROX";
    case SENSOR_CHAN_HUMIDITY:
        return "SENSOR_CHAN_HUMIDITY";
    case SENSOR_CHAN_LIGHT:
        return "SENSOR_CHAN_LIGHT";
    case SENSOR_CHAN_IR:
        return "SENSOR_CHAN_IR";
    case SENSOR_CHAN_RED:
        return "SENSOR_CHAN_RED";
    case SENSOR_CHAN_GREEN:
        return "SENSOR_CHAN_GREEN";
    case SENSOR_CHAN_BLUE:
        return "SENSOR_CHAN_BLUE";
    case SENSOR_CHAN_ALTITUDE:
        return "SENSOR_CHAN_ALTITUDE";
    case SENSOR_CHAN_PM_1_0:
        return "SENSOR_CHAN_PM_1_0";
    case SENSOR_CHAN_PM_2_5:
        return "SENSOR_CHAN_PM_2_5";
    case SENSOR_CHAN_PM_10:
        return "SENSOR_CHAN_PM_10";
    case SENSOR_CHAN_DISTANCE:
        return "SENSOR_CHAN_DISTANCE";
    case SENSOR_CHAN_CO2:
        return "SENSOR_CHAN_CO2";
    case SENSOR_CHAN_VOC:
        return "SENSOR_CHAN_VOC";
    case SENSOR_CHAN_GAS_RES:
        return "SENSOR_CHAN_GAS_RES";
    case SENSOR_CHAN_VOLTAGE:
        return "SENSOR_CHAN_VOLTAGE";
    case SENSOR_CHAN_VSHUNT:
        return "SENSOR_CHAN_VSHUNT";
    case SENSOR_CHAN_CURRENT:
        return "SENSOR_CHAN_CURRENT";
    case SENSOR_CHAN_POWER:
        return "SENSOR_CHAN_POWER";
    case SENSOR_CHAN_RESISTANCE:
        return "SENSOR_CHAN_RESISTANCE";
    case SENSOR_CHAN_ROTATION:
        return "SENSOR_CHAN_ROTATION";
    case SENSOR_CHAN_POS_DX:
        return "SENSOR_CHAN_POS_DX";
    case SENSOR_CHAN_POS_DY:
        return "SENSOR_CHAN_POS_DY";
    case SENSOR_CHAN_POS_DZ:
        return "SENSOR_CHAN_POS_DZ";
    case SENSOR_CHAN_RPM:
        return "SENSOR_CHAN_RPM";
    case SENSOR_CHAN_GAUGE_VOLTAGE:
        return "SENSOR_CHAN_GAUGE_VOLTAGE";
    case SENSOR_CHAN_GAUGE_AVG_CURRENT:
        return "SENSOR_CHAN_GAUGE_AVG_CURRENT";
    case SENSOR_CHAN_GAUGE_STDBY_CURRENT:
        return "SENSOR_CHAN_GAUGE_STDBY_CURRENT";
    case SENSOR_CHAN_GAUGE_MAX_LOAD_CURRENT:
        return "SENSOR_CHAN_GAUGE_MAX_LOAD_CURRENT";
    case SENSOR_CHAN_GAUGE_TEMP:
        return "SENSOR_CHAN_GAUGE_TEMP";
    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
        return "SENSOR_CHAN_GAUGE_STATE_OF_CHARGE";
    case SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY:
        return "SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY";
    case SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY:
        return "SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY";
    case SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY:
        return "SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY";
    case SENSOR_CHAN_GAUGE_FULL_AVAIL_CAPACITY:
        return "SENSOR_CHAN_GAUGE_FULL_AVAIL_CAPACITY";
    case SENSOR_CHAN_GAUGE_AVG_POWER:
        return "SENSOR_CHAN_GAUGE_AVG_POWER";
    case SENSOR_CHAN_GAUGE_STATE_OF_HEALTH:
        return "SENSOR_CHAN_GAUGE_STATE_OF_HEALTH";
    case SENSOR_CHAN_GAUGE_TIME_TO_EMPTY:
        return "SENSOR_CHAN_GAUGE_TIME_TO_EMPTY";
    case SENSOR_CHAN_GAUGE_TIME_TO_FULL:
        return "SENSOR_CHAN_GAUGE_TIME_TO_FULL";
    case SENSOR_CHAN_GAUGE_CYCLE_COUNT:
        return "SENSOR_CHAN_GAUGE_CYCLE_COUNT";
    case SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE:
        return "SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE";
    case SENSOR_CHAN_GAUGE_DESIRED_VOLTAGE:
        return "SENSOR_CHAN_GAUGE_DESIRED_VOLTAGE";
    case SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT:
        return "SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT";
    case SENSOR_CHAN_ALL:
        return "SENSOR_CHAN_ALL";
    case SENSOR_CHAN_COMMON_COUNT:
        return "SENSOR_CHAN_COMMON_COUNT/SENSOR_CHAN_PRIV_START";
    case SENSOR_CHAN_MAX:
        return "SENSOR_CHAN_MAX";
    default:
        return "unknown";
    }
};

static int thb_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct thb_data *drv_data = dev->data;
    struct adc_sequence *as = &drv_data->as;

    if (chan != SENSOR_CHAN_POS_DX && chan != SENSOR_CHAN_POS_DY && chan != SENSOR_CHAN_ALL) {
        LOG_ERR("Selected channel is not supported: %s.", sensor_channel_name(chan));
        return -ENOTSUP;
    }

    int rc = 0;

    rc = adc_read(drv_data->adc, as);
    LOG_DBG("chan %s: read { x: %d, y: %d }", sensor_channel_name(chan), drv_data->xy_raw[0],
            drv_data->xy_raw[1]);
    // First read is setup as calibration
    as->calibrate = false;

    return rc;
}

static int thb_channel_get(const struct device *dev, enum sensor_channel chan,
                           struct sensor_value *val) {
    struct thb_data *drv_data = dev->data;
    const struct thb_config *drv_cfg = dev->config;
    struct adc_sequence *as = &drv_data->as;

    int32_t x_mv = drv_data->xy_raw[0];
    int32_t y_mv = drv_data->xy_raw[1];

    adc_raw_to_millivolts(adc_ref_internal(drv_data->adc), ADC_GAIN_1_6, as->resolution, &x_mv);
    adc_raw_to_millivolts(adc_ref_internal(drv_data->adc), ADC_GAIN_1_6, as->resolution, &y_mv);

    double out = 0.0;
    switch (chan) {
    // convert from millivolt to normalized output in [-1.0, 1.0]
    case SENSOR_CHAN_POS_DX:
        out = 2.0 * x_mv / (drv_cfg->max_mv - drv_cfg->min_mv) - 1.0;
        LOG_DBG("Joystick x chan = %f", out);
        sensor_value_from_double(val, out);
        break;
    case SENSOR_CHAN_POS_DY:
        out = 2.0 * y_mv / (drv_cfg->max_mv - drv_cfg->min_mv) - 1.0;
        LOG_DBG("Joystick y chan = %f", out);
        sensor_value_from_double(val, out);
        break;
    case SENSOR_CHAN_ALL:
        out = 2.0 * x_mv / (drv_cfg->max_mv - drv_cfg->min_mv) - 1.0;
        sensor_value_from_double(val, out);
        LOG_DBG("Joystick x chan = %f", out);
        out = 2.0 * y_mv / (drv_cfg->max_mv - drv_cfg->min_mv) - 1.0;
        sensor_value_from_double(val + 1, out);
        LOG_DBG("Joystick y chan = %f", out);
        break;
    default:
        LOG_DBG("unknown chan %s", sensor_channel_name(chan));
        return -ENOTSUP;
    }

    return 0;
}

#ifdef CONFIG_JOYSTICK_THB_TRIGGER
static int thb_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                           sensor_trigger_handler_t handler) {
    struct thb_data *drv_data = dev->data;
    enum sensor_channel chan = trig->chan;
    enum sensor_trigger_type type = trig->type;

    LOG_DBG("Setting trigger %d on chan %s", type, sensor_channel_name(chan));
    if (chan != SENSOR_CHAN_ROTATION || type != SENSOR_TRIG_DATA_READY) {
        return -ENOTSUP;
    }

    drv_data->trigger = trig;
    drv_data->trigger_handler = handler;

    return 0;
}

static int thb_attr_set(const struct device *dev, enum sensor_channel chan,
                        enum sensor_attribute attr, const struct sensor_value *val) {
    struct thb_data *drv_data = dev->data;
    uint32_t usec = 0;

    LOG_DBG("Setting attr %d on chan %s", attr, sensor_channel_name(chan));
    if (chan != SENSOR_CHAN_ALL || attr != SENSOR_ATTR_SAMPLING_FREQUENCY) {
        return -ENOTSUP;
    }

    if (val->val1 > 100000) {
        LOG_DBG("Sample rate should not exceed 100KHz");
        return -EINVAL;
    }

    drv_data->timer_freq = val->val1;
    if (drv_data->timer_freq != 0) {
        usec = 1000000UL / drv_data->timer_freq;
        LOG_DBG("Setting frequency to %dusec", usec);
        k_timer_start(&drv_data->timer, K_USEC(usec), K_USEC(usec));
    } else {
        LOG_DBG("stopping timer");
        k_timer_stop(&drv_data->timer);
    }

    return 0;
}

static int thb_attr_get(const struct device *dev, enum sensor_channel chan,
                        enum sensor_attribute attr, struct sensor_value *val) {
    struct thb_data *drv_data = dev->data;

    LOG_DBG("Getting attr %d on chan %s", attr, sensor_channel_name(chan));
    if (chan != SENSOR_CHAN_ALL || attr != SENSOR_ATTR_SAMPLING_FREQUENCY) {
        return -ENOTSUP;
    }

    val->val1 = drv_data->timer_freq;
    val->val2 = 0;

    return 0;
}

static void thb_timer_cb(struct k_timer *item) {
    struct thb_data *drv_data = CONTAINER_OF(item, struct thb_data, timer);
#if defined(CONFIG_JOYSTICK_THB_TRIGGER_DEDICATED_QUEUE)
    k_work_submit_to_queue(&thb_work_q, &drv_data->work);
#elif defined(CONFIG_JOYSTICK_THB_TRIGGER_SYSTEM_QUEUE)
    k_work_submit(&drv_data->work);
#endif
}

static void thb_work_fun(struct k_work *item) {
    struct thb_data *drv_data = CONTAINER_OF(item, struct thb_data, work);

    thb_sample_fetch(drv_data->dev, SENSOR_CHAN_ALL);

    if (drv_data->trigger_handler) {
        drv_data->trigger_handler(drv_data->dev, drv_data->trigger);
    }
}
#endif // CONFIG_JOYSTICK_THB_TRIGGER

static int thb_init(const struct device *dev) {
    struct thb_data *drv_data = dev->data;
    const struct thb_config *drv_cfg = dev->config;

    LOG_DBG("Init");
    if (drv_data->adc == NULL) {
        return -ENODEV;
    }

    struct adc_channel_cfg channel_cfg = {
        .gain = ADC_GAIN_1_6,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = drv_cfg->channel_x,
        .input_positive = ADC_INPUT_POS_OFFSET + drv_cfg->channel_x,
    };

    drv_data->dev = dev;
    int rc = adc_channel_setup(drv_data->adc, &channel_cfg);
    if (rc < 0) {
        LOG_DBG("AIN%u setup returned %d", drv_cfg->channel_x, rc);
        return rc;
    }

    channel_cfg.channel_id = drv_cfg->channel_y;
    channel_cfg.input_positive = ADC_INPUT_POS_OFFSET + drv_cfg->channel_y;

    rc = adc_channel_setup(drv_data->adc, &channel_cfg);
    if (rc < 0) {
        LOG_DBG("AIN%u setup returned %d", drv_cfg->channel_y, rc);
        return rc;
    }

    drv_data->as = (struct adc_sequence){
        .channels = BIT(drv_cfg->channel_x) | BIT(drv_cfg->channel_y),
        .buffer = drv_data->xy_raw,
        .buffer_size = sizeof(drv_data->xy_raw),
        .oversampling = 0,
        .resolution = 12,
        .calibrate = true,
    };

#ifdef CONFIG_JOYSTICK_THB_TRIGGER
#ifdef CONFIG_JOYSTICK_THB_TRIGGER_DEDICATED_QUEUE
    if (!is_thb_work_q_ready) {
        k_work_queue_start(&thb_work_q, thb_trigger_stack_area,
                           K_THREAD_STACK_SIZEOF(thb_trigger_stack_area),
                           CONFIG_THB_WORKQUEUE_PRIORITY, NULL);
        is_thb_work_q_ready = true;
    }
#endif
    k_work_init(&drv_data->work, thb_work_fun);
    uint32_t usec = 1000 / drv_cfg->freq;
    k_timer_init(&drv_data->timer, thb_timer_cb, NULL);
    k_timer_start(&drv_data->timer, K_MSEC(usec), K_MSEC(usec));
#endif

    LOG_DBG("Init done");
    return rc;
}

static const struct sensor_driver_api thb_driver_api = {
#ifdef CONFIG_JOYSTICK_THB_TRIGGER
    .trigger_set = thb_trigger_set,
    .attr_set = thb_attr_set,
    .attr_get = thb_attr_get,
#endif
    .sample_fetch = thb_sample_fetch,
    .channel_get = thb_channel_get,
};

#define THB_INST(n)                                                                                \
    static struct thb_data thb_data_##n = {                                                        \
        .adc = DEVICE_DT_GET(DT_INST_IO_CHANNELS_CTLR_BY_NAME(n, x_axis))};                        \
    static const struct thb_config thb_config_##n = {                                              \
        .channel_x = DT_INST_IO_CHANNELS_INPUT_BY_NAME(n, x_axis),                                 \
        .channel_y = DT_INST_IO_CHANNELS_INPUT_BY_NAME(n, y_axis),                                 \
        .max_mv = DT_INST_PROP(n, max_mv),                                                         \
        .min_mv = COND_CODE_0(DT_INST_NODE_HAS_PROP(n, min_mv), (0), (DT_INST_PROP(n, min_mv))),   \
        .freq = COND_CODE_0(DT_INST_NODE_HAS_PROP(n, freq), (100), (DT_INST_PROP(n, freq))),       \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, thb_init, NULL, &thb_data_##n, &thb_config_##n, POST_KERNEL,          \
                          CONFIG_SENSOR_INIT_PRIORITY, &thb_driver_api);

DT_INST_FOREACH_STATUS_OKAY(THB_INST)
