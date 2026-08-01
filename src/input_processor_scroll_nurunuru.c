#define DT_DRV_COMPAT zmk_input_processor_scroll_nurunuru

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/input_processor.h>

LOG_MODULE_REGISTER(zmk_scroll_nurunuru, CONFIG_ZMK_SCROLL_NURUNURU_LOG_LEVEL);

#define FP_SCALE 1024
#define GAIN_SCALE 1000
#define REPORT_INTERVAL_MS 8
#define MAX_SAMPLE_MS 100

struct scroll_nurunuru_config {
    uint16_t base_gain;
    uint16_t max_gain;
    uint16_t acceleration_start;
    uint16_t acceleration_end;
    uint8_t queue_response;
    uint8_t velocity_response;
    uint16_t distance_divisor;
    uint16_t max_velocity;
    bool invert_horizontal;
    bool invert_vertical;
};

struct scroll_nurunuru_data {
    const struct device *dev;
    const struct device *input_device;
    struct k_mutex lock;
    struct k_work_delayable work;
    bool worker_running;
    int32_t pending_horizontal;
    int32_t pending_vertical;
    int32_t queued_horizontal_fp;
    int32_t queued_vertical_fp;
    int32_t velocity_horizontal_fp;
    int32_t velocity_vertical_fp;
    int32_t output_horizontal_fp;
    int32_t output_vertical_fp;
    uint32_t last_sample_ms;
};

static int32_t abs_i32(int32_t value) {
    return value == INT32_MIN ? INT32_MAX : (value < 0 ? -value : value);
}

static int8_t sign_i32(int32_t value) {
    return (value > 0) - (value < 0);
}

static int16_t clamp_i16(int32_t value) {
    return (int16_t)CLAMP(value, (int32_t)INT16_MIN, (int32_t)INT16_MAX);
}

static int32_t smoothstep_per_mille(int32_t x) {
    x = CLAMP(x, 0, GAIN_SCALE);
    int64_t x2 = ((int64_t)x * x) / GAIN_SCALE;
    return (int32_t)((x2 * (3 * GAIN_SCALE - 2 * x)) / GAIN_SCALE);
}

static int32_t calculate_gain(const struct scroll_nurunuru_config *config, int32_t speed) {
    if (speed <= config->acceleration_start) {
        return config->base_gain;
    }
    if (speed >= config->acceleration_end) {
        return config->max_gain;
    }

    int32_t range = config->acceleration_end - config->acceleration_start;
    int32_t progress = (int32_t)(((int64_t)(speed - config->acceleration_start) *
                                  GAIN_SCALE) /
                                 range);
    int32_t curve = smoothstep_per_mille(progress);
    return config->base_gain +
           (int32_t)(((int64_t)(config->max_gain - config->base_gain) * curve) /
                     GAIN_SCALE);
}

static int32_t calculate_distance_fp(int32_t raw_delta, int32_t gain,
                                     uint16_t distance_divisor) {
    int64_t distance = (int64_t)raw_delta * gain * FP_SCALE;
    distance /= (int64_t)GAIN_SCALE * MAX(distance_divisor, 1);
    return (int32_t)CLAMP(distance, (int64_t)INT32_MIN, (int64_t)INT32_MAX);
}

static int32_t saturating_add(int32_t a, int32_t b) {
    return (int32_t)CLAMP((int64_t)a + b, (int64_t)INT32_MIN, (int64_t)INT32_MAX);
}

static int32_t smooth_toward(int32_t current, int32_t target, uint8_t response) {
    response = CLAMP(response, 1, 100);
    int64_t difference = (int64_t)target - current;
    return current + (int32_t)((difference * response) / 100);
}

static void add_distance(int32_t raw_delta, int32_t distance_fp, int32_t *queue_fp,
                         int32_t *velocity_fp, int32_t *output_fp) {
    if (raw_delta == 0) {
        return;
    }

    /* A direction change cancels the old animation before starting the new one. */
    if ((*queue_fp != 0 && sign_i32(raw_delta) != sign_i32(*queue_fp)) ||
        (*velocity_fp != 0 && sign_i32(raw_delta) != sign_i32(*velocity_fp))) {
        *queue_fp = 0;
        *velocity_fp = 0;
        *output_fp = 0;
    }

    *queue_fp = saturating_add(*queue_fp, distance_fp);
}

/*
 * Drain a finite distance queue with a bounded ease-out animation.
 * New input extends the queue without resetting the current velocity.
 */
static int32_t animate_axis(int32_t *queue_fp, int32_t *velocity_fp,
                            const struct scroll_nurunuru_config *config) {
    if (*queue_fp == 0) {
        *velocity_fp = 0;
        return 0;
    }

    int32_t desired =
        (int32_t)(((int64_t)*queue_fp * config->queue_response) / 100);
    int32_t maximum = (int32_t)config->max_velocity * FP_SCALE;
    desired = CLAMP(desired, -maximum, maximum);

    *velocity_fp = smooth_toward(*velocity_fp, desired, config->velocity_response);

    /* Ensure a tiny queued remainder always makes progress. */
    if (*velocity_fp == 0) {
        *velocity_fp = sign_i32(*queue_fp);
    }

    int32_t step = *velocity_fp;
    if (sign_i32(step) != sign_i32(*queue_fp) || abs_i32(step) >= abs_i32(*queue_fp)) {
        step = *queue_fp;
        *queue_fp = 0;
        *velocity_fp = 0;
    } else {
        *queue_fp -= step;
    }

    return step;
}

static int16_t extract_output(int32_t *accumulator_fp) {
    int32_t output = *accumulator_fp / FP_SCALE;
    *accumulator_fp -= output * FP_SCALE;
    return clamp_i16(output);
}

static void send_scroll_events(const struct device *input_device, int16_t horizontal,
                               int16_t vertical) {
    if (input_device == NULL || (horizontal == 0 && vertical == 0)) {
        return;
    }

    if (horizontal != 0) {
        int ret = input_report_rel(input_device, INPUT_REL_HWHEEL, horizontal,
                                   vertical == 0, K_NO_WAIT);
        if (ret < 0) {
            LOG_WRN("Failed horizontal scroll report: %d", ret);
        }
    }
    if (vertical != 0) {
        int ret = input_report_rel(input_device, INPUT_REL_WHEEL, vertical, true, K_NO_WAIT);
        if (ret < 0) {
            LOG_WRN("Failed vertical scroll report: %d", ret);
        }
    }
}

static void scroll_nurunuru_work_callback(struct k_work *work) {
    struct k_work_delayable *delayable = k_work_delayable_from_work(work);
    struct scroll_nurunuru_data *data =
        CONTAINER_OF(delayable, struct scroll_nurunuru_data, work);
    const struct scroll_nurunuru_config *config = data->dev->config;

    k_mutex_lock(&data->lock, K_FOREVER);

    uint32_t now_ms = k_uptime_get_32();
    uint32_t dt_ms = data->last_sample_ms == 0 ? REPORT_INTERVAL_MS
                                               : now_ms - data->last_sample_ms;
    dt_ms = CLAMP(dt_ms, 1U, (uint32_t)MAX_SAMPLE_MS);
    data->last_sample_ms = now_ms;

    int32_t frame_horizontal = data->pending_horizontal;
    int32_t frame_vertical = data->pending_vertical;
    data->pending_horizontal = 0;
    data->pending_vertical = 0;

    int32_t speed = (int32_t)(((int64_t)MAX(abs_i32(frame_horizontal),
                                                    abs_i32(frame_vertical)) *
                               1000) /
                              dt_ms);
    int32_t gain = calculate_gain(config, speed);

    add_distance(frame_horizontal,
                 calculate_distance_fp(frame_horizontal, gain, config->distance_divisor),
                 &data->queued_horizontal_fp, &data->velocity_horizontal_fp,
                 &data->output_horizontal_fp);
    add_distance(frame_vertical,
                 calculate_distance_fp(frame_vertical, gain, config->distance_divisor),
                 &data->queued_vertical_fp, &data->velocity_vertical_fp,
                 &data->output_vertical_fp);

    data->output_horizontal_fp +=
        animate_axis(&data->queued_horizontal_fp, &data->velocity_horizontal_fp, config);
    data->output_vertical_fp +=
        animate_axis(&data->queued_vertical_fp, &data->velocity_vertical_fp, config);

    int16_t output_horizontal = extract_output(&data->output_horizontal_fp);
    int16_t output_vertical = extract_output(&data->output_vertical_fp);

    if (config->invert_horizontal) {
        output_horizontal = -output_horizontal;
    }
    if (config->invert_vertical) {
        output_vertical = -output_vertical;
    }

    bool continue_running = data->pending_horizontal != 0 || data->pending_vertical != 0 ||
                            data->queued_horizontal_fp != 0 ||
                            data->queued_vertical_fp != 0 ||
                            data->velocity_horizontal_fp != 0 ||
                            data->velocity_vertical_fp != 0;

    if (continue_running) {
        k_work_reschedule(&data->work, K_MSEC(REPORT_INTERVAL_MS));
    } else {
        data->worker_running = false;
        data->last_sample_ms = 0;
    }

    const struct device *input_device = data->input_device;
    LOG_DBG("raw=(%ld,%ld) speed=%ld gain=%ld queue=(%ld,%ld) velocity=(%ld,%ld) output=(%d,%d)",
            (long)frame_horizontal, (long)frame_vertical, (long)speed, (long)gain,
            (long)data->queued_horizontal_fp, (long)data->queued_vertical_fp,
            (long)data->velocity_horizontal_fp, (long)data->velocity_vertical_fp,
            output_horizontal, output_vertical);

    k_mutex_unlock(&data->lock);
    send_scroll_events(input_device, output_horizontal, output_vertical);
}

static int scroll_nurunuru_handle_event(
    const struct device *dev, struct input_event *event, uint32_t param1, uint32_t param2,
    struct zmk_input_processor_state *processor_state) {
    struct scroll_nurunuru_data *data = dev->data;
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(processor_state);

    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    uint16_t original_code = event->code;
    int32_t original_value = event->value;
    event->code = original_code == INPUT_REL_X ? INPUT_REL_WHEEL : INPUT_REL_HWHEEL;
    event->value = 0;

    if (original_value == 0) {
        return ZMK_INPUT_PROC_STOP;
    }

    k_mutex_lock(&data->lock, K_FOREVER);
    data->input_device = event->dev;
    if (original_code == INPUT_REL_X) {
        data->pending_vertical = saturating_add(data->pending_vertical, original_value);
    } else {
        data->pending_horizontal = saturating_add(data->pending_horizontal, original_value);
    }

    if (!data->worker_running) {
        data->worker_running = true;
        data->last_sample_ms = 0;
        k_work_reschedule(&data->work, K_MSEC(REPORT_INTERVAL_MS));
    }
    k_mutex_unlock(&data->lock);
    return ZMK_INPUT_PROC_STOP;
}

static int scroll_nurunuru_init(const struct device *dev) {
    struct scroll_nurunuru_data *data = dev->data;
    data->dev = dev;
    k_mutex_init(&data->lock);
    k_work_init_delayable(&data->work, scroll_nurunuru_work_callback);
    LOG_INF("zmk-scroll-nurunuru queued easing engine initialized");
    return 0;
}

static const struct zmk_input_processor_driver_api scroll_nurunuru_driver_api = {
    .handle_event = scroll_nurunuru_handle_event,
};

#define SCROLL_NURUNURU_INST(inst)                                                \
    static struct scroll_nurunuru_data scroll_nurunuru_data_##inst = {};          \
    static const struct scroll_nurunuru_config scroll_nurunuru_config_##inst = {  \
        .base_gain = DT_INST_PROP(inst, base_gain),                               \
        .max_gain = DT_INST_PROP(inst, max_gain),                                 \
        .acceleration_start = DT_INST_PROP(inst, acceleration_start),             \
        .acceleration_end = DT_INST_PROP(inst, acceleration_end),                 \
        .queue_response = DT_INST_PROP(inst, response),                           \
        .velocity_response = DT_INST_PROP(inst, input_filter_response),           \
        .distance_divisor = DT_INST_PROP(inst, distance_divisor),                 \
        .max_velocity = DT_INST_PROP(inst, max_velocity),                         \
        .invert_horizontal = DT_INST_PROP_OR(inst, invert_horizontal, false),     \
        .invert_vertical = DT_INST_PROP_OR(inst, invert_vertical, false),         \
    };                                                                            \
    DEVICE_DT_INST_DEFINE(inst, scroll_nurunuru_init, NULL,                       \
                          &scroll_nurunuru_data_##inst,                            \
                          &scroll_nurunuru_config_##inst, POST_KERNEL,             \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                     \
                          &scroll_nurunuru_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_NURUNURU_INST)
