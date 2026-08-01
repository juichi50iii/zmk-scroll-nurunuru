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
#define CURVE_SCALE 1000
#define REPORT_INTERVAL_MS 8
#define MAX_SAMPLE_MS 120

struct scroll_nurunuru_config {
    uint16_t base_gain;
    uint16_t max_gain;
    uint16_t acceleration_start;
    uint16_t acceleration_end;
    uint8_t target_response;
    uint8_t velocity_response;
    uint16_t distance_divisor;
    uint16_t max_velocity;
    uint16_t coast_ms;
    uint16_t release_ms;
    bool invert_horizontal;
    bool invert_vertical;
};

struct axis_animation {
    int32_t target_fp;
    int32_t velocity_fp;
    int32_t release_velocity_fp;
    uint32_t coast_started_ms;
    uint32_t last_input_ms;
    bool coasting;
};

struct scroll_nurunuru_data {
    const struct device *dev;
    const struct device *input_device;
    struct k_mutex lock;
    struct k_work_delayable work;
    bool worker_running;
    int32_t pending_horizontal;
    int32_t pending_vertical;
    struct axis_animation horizontal;
    struct axis_animation vertical;
    int32_t output_horizontal_fp;
    int32_t output_vertical_fp;
};

static int32_t abs_i32(int32_t value) {
    return value == INT32_MIN ? INT32_MAX : (value < 0 ? -value : value);
}

static int8_t sign_i32(int32_t value) {
    return (value > 0) - (value < 0);
}

static int32_t saturating_add(int32_t a, int32_t b) {
    return (int32_t)CLAMP((int64_t)a + b, (int64_t)INT32_MIN, (int64_t)INT32_MAX);
}

static int16_t clamp_i16(int32_t value) {
    return (int16_t)CLAMP(value, (int32_t)INT16_MIN, (int32_t)INT16_MAX);
}

static int32_t smoothstep(int32_t x) {
    x = CLAMP(x, 0, CURVE_SCALE);
    int64_t x2 = ((int64_t)x * x) / CURVE_SCALE;
    return (int32_t)((x2 * (3 * CURVE_SCALE - 2 * x)) / CURVE_SCALE);
}

/* x^3 * (x * (6x - 15) + 10): zero slope and acceleration at both ends. */
static int32_t smootherstep(int32_t x) {
    x = CLAMP(x, 0, CURVE_SCALE);
    int64_t x2 = ((int64_t)x * x) / CURVE_SCALE;
    int64_t x3 = (x2 * x) / CURVE_SCALE;
    int64_t inner = ((6LL * x * x) / CURVE_SCALE) - (15LL * x) +
                    (10LL * CURVE_SCALE);
    return (int32_t)((x3 * inner) / CURVE_SCALE);
}

static int32_t smooth_toward(int32_t current, int32_t target, uint8_t response) {
    response = CLAMP(response, 1, 100);
    return current + (int32_t)(((int64_t)(target - current) * response) / 100);
}

static int32_t calculate_gain(const struct scroll_nurunuru_config *config, int32_t speed) {
    if (speed <= config->acceleration_start) {
        return config->base_gain;
    }
    if (speed >= config->acceleration_end) {
        return config->max_gain;
    }

    int32_t progress = (int32_t)(((int64_t)(speed - config->acceleration_start) *
                                  CURVE_SCALE) /
                                 (config->acceleration_end - config->acceleration_start));
    int32_t curve = smootherstep(progress);
    return config->base_gain +
           (int32_t)(((int64_t)(config->max_gain - config->base_gain) * curve) /
                     CURVE_SCALE);
}

static int32_t calculate_target_fp(int32_t raw_delta, uint32_t dt_ms, int32_t gain,
                                   const struct scroll_nurunuru_config *config) {
    int64_t target = (int64_t)raw_delta * REPORT_INTERVAL_MS * FP_SCALE * gain;
    target /= (int64_t)MAX(dt_ms, 1U) * CURVE_SCALE *
              MAX(config->distance_divisor, 1);
    int32_t maximum = (int32_t)config->max_velocity * FP_SCALE;
    return (int32_t)CLAMP(target, -(int64_t)maximum, (int64_t)maximum);
}

static void accept_axis_input(struct axis_animation *axis, int32_t raw_delta, uint32_t now_ms,
                              const struct scroll_nurunuru_config *config) {
    if (raw_delta == 0) {
        return;
    }

    uint32_t dt_ms = axis->last_input_ms == 0 ? REPORT_INTERVAL_MS
                                               : now_ms - axis->last_input_ms;
    dt_ms = CLAMP(dt_ms, 1U, (uint32_t)MAX_SAMPLE_MS);
    int32_t speed = (int32_t)(((int64_t)abs_i32(raw_delta) * 1000) / dt_ms);
    int32_t target = calculate_target_fp(raw_delta, dt_ms, calculate_gain(config, speed), config);

    bool reversing = axis->velocity_fp != 0 &&
                     sign_i32(target) != sign_i32(axis->velocity_fp);
    bool starting = axis->last_input_ms == 0 || axis->coasting;

    if (reversing) {
        axis->velocity_fp = 0;
        axis->target_fp = 0;
        starting = true;
    }

    if (starting) {
        axis->coasting = false;
        axis->target_fp = target;
    } else {
        axis->target_fp = smooth_toward(axis->target_fp, target, config->target_response);
    }

    axis->last_input_ms = now_ms;
}

static int32_t animate_axis(struct axis_animation *axis, uint32_t now_ms,
                            const struct scroll_nurunuru_config *config) {
    bool input_held = axis->last_input_ms != 0 &&
                      (now_ms - axis->last_input_ms) < config->release_ms;

    if (input_held) {
        axis->velocity_fp = smooth_toward(axis->velocity_fp, axis->target_fp,
                                          config->velocity_response);
        return axis->velocity_fp;
    }

    if (!axis->coasting && axis->velocity_fp != 0) {
        axis->coasting = true;
        axis->coast_started_ms = now_ms;
        axis->release_velocity_fp = axis->velocity_fp;
    }

    if (axis->coasting) {
        uint32_t elapsed = now_ms - axis->coast_started_ms;
        if (config->coast_ms == 0 || elapsed >= config->coast_ms) {
            axis->velocity_fp = 0;
            axis->target_fp = 0;
            axis->last_input_ms = 0;
            axis->coasting = false;
            return 0;
        }

        int32_t progress = (int32_t)((elapsed * CURVE_SCALE) / config->coast_ms);
        axis->velocity_fp = (int32_t)(((int64_t)axis->release_velocity_fp *
                                       (CURVE_SCALE - smootherstep(progress))) /
                                      CURVE_SCALE);
        return axis->velocity_fp;
    }

    return 0;
}

/* Fixed-point error diffusion distributes unavoidable 1/16-notch reports evenly. */
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
        input_report_rel(input_device, INPUT_REL_HWHEEL, horizontal, vertical == 0, K_NO_WAIT);
    }
    if (vertical != 0) {
        input_report_rel(input_device, INPUT_REL_WHEEL, vertical, true, K_NO_WAIT);
    }
}

static void scroll_nurunuru_work_callback(struct k_work *work) {
    struct scroll_nurunuru_data *data =
        CONTAINER_OF(k_work_delayable_from_work(work), struct scroll_nurunuru_data, work);
    const struct scroll_nurunuru_config *config = data->dev->config;

    k_mutex_lock(&data->lock, K_FOREVER);
    uint32_t now_ms = k_uptime_get_32();
    int32_t frame_horizontal = data->pending_horizontal;
    int32_t frame_vertical = data->pending_vertical;
    data->pending_horizontal = 0;
    data->pending_vertical = 0;

    accept_axis_input(&data->horizontal, frame_horizontal, now_ms, config);
    accept_axis_input(&data->vertical, frame_vertical, now_ms, config);

    data->output_horizontal_fp += animate_axis(&data->horizontal, now_ms, config);
    data->output_vertical_fp += animate_axis(&data->vertical, now_ms, config);
    int16_t output_horizontal = extract_output(&data->output_horizontal_fp);
    int16_t output_vertical = extract_output(&data->output_vertical_fp);

    if (config->invert_horizontal) {
        output_horizontal = -output_horizontal;
    }
    if (config->invert_vertical) {
        output_vertical = -output_vertical;
    }

    bool active = data->horizontal.last_input_ms != 0 || data->vertical.last_input_ms != 0 ||
                  data->horizontal.coasting || data->vertical.coasting;
    if (active) {
        k_work_reschedule(&data->work, K_MSEC(REPORT_INTERVAL_MS));
    } else {
        data->worker_running = false;
    }

    const struct device *input_device = data->input_device;
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
    LOG_INF("zmk-scroll-nurunuru time-curve animator initialized");
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
        .target_response = DT_INST_PROP(inst, target_response),                   \
        .velocity_response = DT_INST_PROP(inst, velocity_response),               \
        .distance_divisor = DT_INST_PROP(inst, distance_divisor),                 \
        .max_velocity = DT_INST_PROP(inst, max_velocity),                         \
        .coast_ms = DT_INST_PROP(inst, coast_ms),                                 \
        .release_ms = DT_INST_PROP(inst, release_ms),                             \
        .invert_horizontal = DT_INST_PROP_OR(inst, invert_horizontal, false),     \
        .invert_vertical = DT_INST_PROP_OR(inst, invert_vertical, false),         \
    };                                                                            \
    DEVICE_DT_INST_DEFINE(inst, scroll_nurunuru_init, NULL,                       \
                          &scroll_nurunuru_data_##inst,                            \
                          &scroll_nurunuru_config_##inst, POST_KERNEL,             \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                     \
                          &scroll_nurunuru_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_NURUNURU_INST)
