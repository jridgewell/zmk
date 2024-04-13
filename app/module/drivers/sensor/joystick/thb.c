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
    struct sensor_trigger trigger;
    int32_t trigger_freq;
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
static const char *sensor_channel_name[SENSOR_CHAN_COMMON_COUNT] = {
	[SENSOR_CHAN_ACCEL_X] = "accel_x",
	[SENSOR_CHAN_ACCEL_Y] = "accel_y",
	[SENSOR_CHAN_ACCEL_Z] = "accel_z",
	[SENSOR_CHAN_ACCEL_XYZ] = "accel_xyz",
	[SENSOR_CHAN_GYRO_X] = "gyro_x",
	[SENSOR_CHAN_GYRO_Y] = "gyro_y",
	[SENSOR_CHAN_GYRO_Z] = "gyro_z",
	[SENSOR_CHAN_GYRO_XYZ] = "gyro_xyz",
	[SENSOR_CHAN_MAGN_X] = "magn_x",
	[SENSOR_CHAN_MAGN_Y] = "magn_y",
	[SENSOR_CHAN_MAGN_Z] = "magn_z",
	[SENSOR_CHAN_MAGN_XYZ] = "magn_xyz",
	[SENSOR_CHAN_DIE_TEMP] = "die_temp",
	[SENSOR_CHAN_AMBIENT_TEMP] = "ambient_temp",
	[SENSOR_CHAN_PRESS] = "press",
	[SENSOR_CHAN_PROX] = "prox",
	[SENSOR_CHAN_HUMIDITY] = "humidity",
	[SENSOR_CHAN_LIGHT] = "light",
	[SENSOR_CHAN_IR] = "ir",
	[SENSOR_CHAN_RED] = "red",
	[SENSOR_CHAN_GREEN] = "green",
	[SENSOR_CHAN_BLUE] = "blue",
	[SENSOR_CHAN_ALTITUDE] = "altitude",
	[SENSOR_CHAN_PM_1_0] = "pm_1_0",
	[SENSOR_CHAN_PM_2_5] = "pm_2_5",
	[SENSOR_CHAN_PM_10] = "pm_10",
	[SENSOR_CHAN_DISTANCE] = "distance",
	[SENSOR_CHAN_CO2] = "co2",
	[SENSOR_CHAN_O2] = "o2",
	[SENSOR_CHAN_VOC] = "voc",
	[SENSOR_CHAN_GAS_RES] = "gas_resistance",
	[SENSOR_CHAN_VOLTAGE] = "voltage",
	[SENSOR_CHAN_CURRENT] = "current",
	[SENSOR_CHAN_POWER] = "power",
	[SENSOR_CHAN_RESISTANCE] = "resistance",
	[SENSOR_CHAN_ROTATION] = "rotation",
	[SENSOR_CHAN_POS_DX] = "pos_dx",
	[SENSOR_CHAN_POS_DY] = "pos_dy",
	[SENSOR_CHAN_POS_DZ] = "pos_dz",
	[SENSOR_CHAN_RPM] = "rpm",
	[SENSOR_CHAN_GAUGE_VOLTAGE] = "gauge_voltage",
	[SENSOR_CHAN_GAUGE_AVG_CURRENT] = "gauge_avg_current",
	[SENSOR_CHAN_GAUGE_STDBY_CURRENT] = "gauge_stdby_current",
	[SENSOR_CHAN_GAUGE_MAX_LOAD_CURRENT] = "gauge_max_load_current",
	[SENSOR_CHAN_GAUGE_TEMP] = "gauge_temp",
	[SENSOR_CHAN_GAUGE_STATE_OF_CHARGE] = "gauge_state_of_charge",
	[SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY] = "gauge_full_cap",
	[SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY] = "gauge_remaining_cap",
	[SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY] = "gauge_nominal_cap",
	[SENSOR_CHAN_GAUGE_FULL_AVAIL_CAPACITY] = "gauge_full_avail_cap",
	[SENSOR_CHAN_GAUGE_AVG_POWER] = "gauge_avg_power",
	[SENSOR_CHAN_GAUGE_STATE_OF_HEALTH] = "gauge_state_of_health",
	[SENSOR_CHAN_GAUGE_TIME_TO_EMPTY] = "gauge_time_to_empty",
	[SENSOR_CHAN_GAUGE_TIME_TO_FULL] = "gauge_time_to_full",
	[SENSOR_CHAN_GAUGE_CYCLE_COUNT] = "gauge_cycle_count",
	[SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE] = "gauge_design_voltage",
	[SENSOR_CHAN_GAUGE_DESIRED_VOLTAGE] = "gauge_desired_voltage",
	[SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT] = "gauge_desired_charging_current",
	[SENSOR_CHAN_ALL] = "all",
};

static int thb_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct thb_data *drv_data = dev->data;
    struct adc_sequence *as = &drv_data->as;

    if (chan != SENSOR_CHAN_POS_DX && chan != SENSOR_CHAN_POS_DY && chan != SENSOR_CHAN_ALL) {
        LOG_ERR("Selected channel is not supported: %d.", sensor_channel_name[chan]);
        return -ENOTSUP;
    }

    int rc = 0;

    rc = adc_read(drv_data->adc, as);
    LOG_DBG("chan %d: read { x: %d, y: %d }", sensor_channel_name[chan], drv_data->xy_raw[0],
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

    adc_raw_to_millivolts(adc_ref_internal(drv_data->adc), ADC_GAIN_1_3, as->resolution, &x_mv);
    adc_raw_to_millivolts(adc_ref_internal(drv_data->adc), ADC_GAIN_1_3, as->resolution, &y_mv);

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
        LOG_DBG("unknown chan %i", sensor_channel_name[chan]);
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

    LOG_DBG("Setting trigger %d on chan %d", type, sensor_channel_name[chan]);
    if (chan != SENSOR_CHAN_ALL || type != SENSOR_TRIG_DATA_READY) {
        return -ENOTSUP;
    }

    drv_data->trigger = *trig;
    drv_data->trigger_handler = handler;

    return 0;
}

static int thb_attr_set(const struct device *dev, enum sensor_channel chan,
                        enum sensor_attribute attr, const struct sensor_value *val) {
    struct thb_data *drv_data = dev->data;
    uint32_t usec = 0;

    LOG_DBG("Setting attr %d on chan %d", attr, sensor_channel_name[chan]);
    if (chan != SENSOR_CHAN_ALL || attr != SENSOR_ATTR_SAMPLING_FREQUENCY) {
        return -ENOTSUP;
    }

    if (val->val1 > 100000) {
        LOG_DBG("Sample rate should not exceed 100KHz");
        return -EINVAL;
    }

    drv_data->trigger_freq = val->val1;
    if (drv_data->trigger_freq != 0) {
        usec = 1000000UL / drv_data->trigger_freq;
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

    LOG_DBG("Getting attr %d on chan %d", attr, sensor_channel_name[chan]);
    if (chan != SENSOR_CHAN_ALL || attr != SENSOR_ATTR_SAMPLING_FREQUENCY) {
        return -ENOTSUP;
    }

    val->val1 = drv_data->trigger_freq;
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
        drv_data->trigger_handler(drv_data->dev, &drv_data->trigger);
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
        .gain = ADC_GAIN_1_3,
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
        .channels = BIT(drv_cfg->channel_y) | BIT(drv_cfg->channel_y),
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
    k_timer_user_data_set(&drv_data->timer, dev);
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
