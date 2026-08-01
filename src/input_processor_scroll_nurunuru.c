#define DT_DRV_COMPAT zmk_input_processor_scroll_nurunuru

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/input_processor.h>

LOG_MODULE_REGISTER(zmk_scroll_nurunuru, CONFIG_ZMK_SCROLL_NURUNURU_LOG_LEVEL);

#define MAX_EVENTS_PER_SAMPLE 10
#define SPEED_RESET_MS 100
#define REMAINDER_RESET_MS 100

struct scroll_nurunuru_config {
    uint16_t scroll_tick;
    uint8_t sensitivity;
    bool invert_horizontal;
    bool invert_vertical;
};

struct scroll_nurunuru_data {
    const struct device *input_device;
    struct k_mutex lock;
    int32_t pending_x;
    int32_t pending_y;
    int32_t remainder_x;
    int32_t remainder_y;
    int64_t last_sample_ms;
    int64_t last_remainder_ms;
};

static int32_t saturating_add(int32_t a, int32_t b) {
    return (int32_t)CLAMP((int64_t)a + b, (int64_t)INT32_MIN, (int64_t)INT32_MAX);
}

static int32_t accelerated_delta(int32_t delta, float acceleration) {
    if (abs(delta) <= 1) {
        return delta;
    }
    return (int32_t)((float)delta * acceleration);
}

static void emit_axis(const struct device *input_device, uint16_t code, int32_t *remainder,
                      uint16_t tick, bool invert, bool sync_last) {
    if (tick == 0 || abs(*remainder) <= tick) {
        return;
    }

    int direction = *remainder > 0 ? 1 : -1;
    int count = MIN(abs(*remainder) / tick, MAX_EVENTS_PER_SAMPLE);
    *remainder -= direction * count * tick;
    if (invert) {
        direction = -direction;
    }

    for (int i = 0; i < count; i++) {
        input_report_rel(input_device, code, direction,
                         sync_last && i == count - 1, K_MSEC(10));
    }
}

static void process_sample(struct scroll_nurunuru_data *data,
                           const struct scroll_nurunuru_config *config, int64_t now_ms) {
    int32_t x = data->pending_x;
    int32_t y = data->pending_y;
    data->pending_x = 0;
    data->pending_y = 0;

    if (data->last_remainder_ms > 0 &&
        now_ms - data->last_remainder_ms > REMAINDER_RESET_MS) {
        data->remainder_x = 0;
        data->remainder_y = 0;
        data->last_remainder_ms = 0;
    }

    int32_t movement = abs(x) + abs(y);
    int64_t dt_ms = data->last_sample_ms > 0 ? now_ms - data->last_sample_ms : 0;
    float acceleration = 1.0f;

    if (dt_ms > 0 && dt_ms < SPEED_RESET_MS) {
        float speed = (float)movement / (float)dt_ms;
        float base_sensitivity = (float)config->sensitivity / 2.5f;
        acceleration = 1.0f + 9.0f / (1.0f + expf(-0.5f * (speed - 8.0f)));
        acceleration *= base_sensitivity;
    }
    data->last_sample_ms = now_ms;

    data->remainder_x = saturating_add(data->remainder_x,
                                       accelerated_delta(x, acceleration));
    data->remainder_y = saturating_add(data->remainder_y,
                                       accelerated_delta(y, acceleration));

    bool emit_y = abs(data->remainder_y) > config->scroll_tick;
    bool emit_x = !emit_y && abs(data->remainder_x) > config->scroll_tick;
    if (emit_y) {
        emit_axis(data->input_device, INPUT_REL_WHEEL, &data->remainder_y,
                  config->scroll_tick, config->invert_vertical, true);
        data->remainder_x = 0;
    } else if (emit_x) {
        emit_axis(data->input_device, INPUT_REL_HWHEEL, &data->remainder_x,
                  config->scroll_tick, config->invert_horizontal, true);
        data->remainder_y = 0;
    }

    if (data->remainder_x != 0 || data->remainder_y != 0) {
        data->last_remainder_ms = now_ms;
    } else {
        data->last_remainder_ms = 0;
    }
}

static int scroll_nurunuru_handle_event(
    const struct device *dev, struct input_event *event, uint32_t param1, uint32_t param2,
    struct zmk_input_processor_state *processor_state) {
    struct scroll_nurunuru_data *data = dev->data;
    const struct scroll_nurunuru_config *config = dev->config;
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(processor_state);

    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    uint16_t original_code = event->code;
    int32_t original_value = event->value;
    bool sample_complete = event->sync;
    event->code = original_code == INPUT_REL_X ? INPUT_REL_WHEEL : INPUT_REL_HWHEEL;
    event->value = 0;

    k_mutex_lock(&data->lock, K_FOREVER);
    data->input_device = event->dev;
    if (original_code == INPUT_REL_X) {
        data->pending_y = saturating_add(data->pending_y, original_value);
    } else {
        data->pending_x = saturating_add(data->pending_x, original_value);
    }
    if (sample_complete && data->input_device != NULL) {
        process_sample(data, config, k_uptime_get());
    }
    k_mutex_unlock(&data->lock);

    return ZMK_INPUT_PROC_STOP;
}

static int scroll_nurunuru_init(const struct device *dev) {
    struct scroll_nurunuru_data *data = dev->data;
    k_mutex_init(&data->lock);
    LOG_INF("zmk-scroll-nurunuru PMW3610 sigmoid mode initialized");
    return 0;
}

static const struct zmk_input_processor_driver_api scroll_nurunuru_driver_api = {
    .handle_event = scroll_nurunuru_handle_event,
};

#define SCROLL_NURUNURU_INST(inst)                                               \
    static struct scroll_nurunuru_data scroll_nurunuru_data_##inst = {};         \
    static const struct scroll_nurunuru_config scroll_nurunuru_config_##inst = { \
        .scroll_tick = DT_INST_PROP(inst, scroll_tick),                          \
        .sensitivity = DT_INST_PROP(inst, sensitivity),                          \
        .invert_horizontal = DT_INST_PROP_OR(inst, invert_horizontal, false),    \
        .invert_vertical = DT_INST_PROP_OR(inst, invert_vertical, false),        \
    };                                                                           \
    DEVICE_DT_INST_DEFINE(inst, scroll_nurunuru_init, NULL,                      \
                          &scroll_nurunuru_data_##inst,                           \
                          &scroll_nurunuru_config_##inst, POST_KERNEL,            \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                    \
                          &scroll_nurunuru_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_NURUNURU_INST)
