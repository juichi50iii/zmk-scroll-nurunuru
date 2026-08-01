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
#define INPUT_RELEASE_MS 40
#define MAX_SAMPLE_MS 100
#define RAW_SCROLL_DIVISOR 24

struct scroll_nurunuru_config {
    uint16_t base_gain;
    uint16_t max_gain;
    uint16_t acceleration_start;
    uint16_t acceleration_end;
    uint8_t input_filter_response;
    uint8_t response;
    uint16_t drag_per_mille;
    uint16_t stop_threshold;
    uint16_t input_gap_hold_ms;
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
    int32_t velocity_horizontal_fp;
    int32_t velocity_vertical_fp;
    int32_t output_horizontal_fp;
    int32_t output_vertical_fp;
    int32_t target_horizontal_fp;
    int32_t target_vertical_fp;
    uint32_t last_input_ms;
    uint32_t last_sample_ms;
    uint32_t last_horizontal_input_ms;
    uint32_t last_vertical_input_ms;
};

static int32_t abs_i32(int32_t value) {
    if (value == INT32_MIN) {
        return INT32_MAX;
    }
    return value < 0 ? -value : value;
}

static int8_t sign_i32(int32_t value) {
    return (value > 0) - (value < 0);
}

static int16_t clamp_i16(int32_t value) {
    return (int16_t)CLAMP(value, (int32_t)INT16_MIN, (int32_t)INT16_MAX);
}

/* smoothstep(x) = x*x*(3-2*x), with x in 0..1000. */
static int32_t smoothstep_per_mille(int32_t x) {
    x = CLAMP(x, 0, GAIN_SCALE);
    int64_t x2 = ((int64_t)x * x) / GAIN_SCALE;
    return (int32_t)((x2 * (3 * GAIN_SCALE - 2 * x)) / GAIN_SCALE);
}

static int32_t calculate_gain(const struct scroll_nurunuru_config *config,
                              int32_t speed_counts_per_second) {
    if (speed_counts_per_second <= config->acceleration_start) {
        return config->base_gain;
    }
    if (speed_counts_per_second >= config->acceleration_end) {
        return config->max_gain;
    }

    int32_t range = config->acceleration_end - config->acceleration_start;
    int32_t position = speed_counts_per_second - config->acceleration_start;
    int32_t progress = (int32_t)(((int64_t)position * GAIN_SCALE) / range);
    int32_t curve = smoothstep_per_mille(progress);

    return config->base_gain +
           (int32_t)(((int64_t)(config->max_gain - config->base_gain) * curve) /
                     GAIN_SCALE);
}

/* Convert raw movement collected during dt into wheel units per 8 ms frame. */
static int32_t calculate_target_velocity_fp(int32_t raw_delta, uint32_t dt_ms,
                                            int32_t gain_per_mille) {
    if (raw_delta == 0 || dt_ms == 0) {
        return 0;
    }

    int64_t target = (int64_t)raw_delta * REPORT_INTERVAL_MS * FP_SCALE;
    target *= gain_per_mille;
    target /= (int64_t)dt_ms * RAW_SCROLL_DIVISOR * GAIN_SCALE;
    return (int32_t)CLAMP(target, (int64_t)INT32_MIN, (int64_t)INT32_MAX);
}

static int32_t smooth_toward(int32_t current, int32_t target, uint8_t response) {
    response = CLAMP(response, 1, 100);
    int64_t difference = (int64_t)target - current;
    return current + (int32_t)((difference * response) / 100);
}

static int32_t apply_drag(int32_t value, uint16_t drag_per_mille,
                          uint16_t stop_threshold) {
    int32_t result = (int32_t)(((int64_t)value * drag_per_mille) / GAIN_SCALE);
    return abs_i32(result) < stop_threshold ? 0 : result;
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

static void update_axis(int32_t frame, int32_t target, int32_t *velocity,
                        int32_t *output, uint8_t response, uint16_t drag,
                        uint16_t stop_threshold, bool input_active) {
    /* The first opposite-direction sample brakes this axis and is absorbed. */
    if (frame != 0 && *velocity != 0 && sign_i32(frame) != sign_i32(*velocity)) {
        *velocity = 0;
        *output = 0;
        return;
    }

    if (input_active) {
        *velocity = smooth_toward(*velocity, target, response);
    } else {
        *velocity = apply_drag(*velocity, drag, stop_threshold);
    }
}

static void scroll_nurunuru_work_callback(struct k_work *work) {
    struct k_work_delayable *delayable = k_work_delayable_from_work(work);
    struct scroll_nurunuru_data *data =
        CONTAINER_OF(delayable, struct scroll_nurunuru_data, work);
    const struct scroll_nurunuru_config *config = data->dev->config;
    int16_t output_horizontal;
    int16_t output_vertical;
    const struct device *input_device;

    k_mutex_lock(&data->lock, K_FOREVER);

    uint32_t now_ms = k_uptime_get_32();
    int32_t frame_horizontal = data->pending_horizontal;
    int32_t frame_vertical = data->pending_vertical;
    data->pending_horizontal = 0;
    data->pending_vertical = 0;

    bool input_active = frame_horizontal != 0 || frame_vertical != 0;
    uint32_t dt_ms = data->last_sample_ms == 0
                         ? REPORT_INTERVAL_MS
                         : now_ms - data->last_sample_ms;
    dt_ms = CLAMP(dt_ms, 1U, (uint32_t)MAX_SAMPLE_MS);

    if (input_active) {
        data->last_sample_ms = now_ms;
    }

    int32_t speed = (int32_t)(((int64_t)MAX(abs_i32(frame_horizontal),
                                                    abs_i32(frame_vertical)) *
                               1000) /
                              dt_ms);
    int32_t gain = calculate_gain(config, speed);
    if (frame_horizontal != 0) {
        int32_t raw_target =
            calculate_target_velocity_fp(frame_horizontal, dt_ms, gain);
        if (data->target_horizontal_fp != 0 &&
            sign_i32(raw_target) != sign_i32(data->target_horizontal_fp)) {
            data->target_horizontal_fp = raw_target;
        } else {
            data->target_horizontal_fp =
                smooth_toward(data->target_horizontal_fp, raw_target,
                              config->input_filter_response);
        }
        data->last_horizontal_input_ms = now_ms;
    }
    if (frame_vertical != 0) {
        int32_t raw_target =
            calculate_target_velocity_fp(frame_vertical, dt_ms, gain);
        if (data->target_vertical_fp != 0 &&
            sign_i32(raw_target) != sign_i32(data->target_vertical_fp)) {
            data->target_vertical_fp = raw_target;
        } else {
            data->target_vertical_fp =
                smooth_toward(data->target_vertical_fp, raw_target,
                              config->input_filter_response);
        }
        data->last_vertical_input_ms = now_ms;
    }

    bool horizontal_active =
        frame_horizontal != 0 ||
        (data->last_horizontal_input_ms != 0 &&
         (now_ms - data->last_horizontal_input_ms) < config->input_gap_hold_ms);
    bool vertical_active =
        frame_vertical != 0 ||
        (data->last_vertical_input_ms != 0 &&
         (now_ms - data->last_vertical_input_ms) < config->input_gap_hold_ms);

    update_axis(frame_horizontal, data->target_horizontal_fp, &data->velocity_horizontal_fp,
                &data->output_horizontal_fp, config->response, config->drag_per_mille,
                config->stop_threshold, horizontal_active);
    update_axis(frame_vertical, data->target_vertical_fp, &data->velocity_vertical_fp,
                &data->output_vertical_fp, config->response, config->drag_per_mille,
                config->stop_threshold, vertical_active);

    data->output_horizontal_fp += data->velocity_horizontal_fp;
    data->output_vertical_fp += data->velocity_vertical_fp;
    output_horizontal = extract_output(&data->output_horizontal_fp);
    output_vertical = extract_output(&data->output_vertical_fp);

    if (config->invert_horizontal) {
        output_horizontal = -output_horizontal;
    }
    if (config->invert_vertical) {
        output_vertical = -output_vertical;
    }

    bool velocity_active = data->velocity_horizontal_fp != 0 ||
                           data->velocity_vertical_fp != 0;
    bool release_pending = (now_ms - data->last_input_ms) < INPUT_RELEASE_MS;

    if (input_active || horizontal_active || vertical_active || velocity_active || release_pending) {
        k_work_reschedule(&data->work, K_MSEC(REPORT_INTERVAL_MS));
    } else {
        data->worker_running = false;
        data->last_sample_ms = 0;
        data->target_horizontal_fp = 0;
        data->target_vertical_fp = 0;
        data->last_horizontal_input_ms = 0;
        data->last_vertical_input_ms = 0;
        data->output_horizontal_fp = 0;
        data->output_vertical_fp = 0;
    }

    input_device = data->input_device;
    LOG_DBG("raw=(%ld,%ld) dt=%u speed=%ld gain=%ld velocity=(%ld,%ld) output=(%d,%d)",
            (long)frame_horizontal, (long)frame_vertical, dt_ms, (long)speed,
            (long)gain, (long)data->velocity_horizontal_fp,
            (long)data->velocity_vertical_fp, output_horizontal, output_vertical);

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
        data->pending_vertical += original_value;
    } else {
        data->pending_horizontal += original_value;
    }
    data->last_input_ms = k_uptime_get_32();

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
    LOG_INF("zmk-scroll-nurunuru Mac Mouse Fix style engine initialized");
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
        .input_filter_response = DT_INST_PROP(inst, input_filter_response),       \
        .response = DT_INST_PROP(inst, response),                                 \
        .drag_per_mille = DT_INST_PROP(inst, drag_per_mille),                     \
        .stop_threshold = DT_INST_PROP(inst, stop_threshold),                     \
        .input_gap_hold_ms = DT_INST_PROP(inst, input_gap_hold_ms),               \
        .invert_horizontal = DT_INST_PROP_OR(inst, invert_horizontal, false),     \
        .invert_vertical = DT_INST_PROP_OR(inst, invert_vertical, false),         \
    };                                                                            \
    DEVICE_DT_INST_DEFINE(inst, scroll_nurunuru_init, NULL,                       \
                          &scroll_nurunuru_data_##inst,                            \
                          &scroll_nurunuru_config_##inst, POST_KERNEL,             \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                     \
                          &scroll_nurunuru_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_NURUNURU_INST)
