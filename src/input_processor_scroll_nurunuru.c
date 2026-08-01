#define DT_DRV_COMPAT zmk_input_processor_scroll_nurunuru

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

#define UPDATE_MS 8
#define VELOCITY_Q 1024
#define MAX_EVENTS_PER_TICK 4
#define MAX_SAMPLE_MS 200

struct scroll_nurunuru_config {
    uint16_t scroll_tick;
    uint8_t sensitivity;
    uint16_t response_ms;
    uint16_t bridge_ms;
    uint16_t release_ms;
    bool invert_horizontal;
    bool invert_vertical;
};

struct scroll_nurunuru_data {
    const struct device *input_device;
    const struct device *dev;
    struct k_mutex lock;
    struct k_work_delayable update_work;
    int32_t pending_wheel;
    int32_t pending_hwheel;
    int32_t target_wheel_q;
    int32_t target_hwheel_q;
    int32_t velocity_wheel_q;
    int32_t velocity_hwheel_q;
    int64_t phase_wheel_mq;
    int64_t phase_hwheel_mq;
    int64_t last_sample_ms;
    int64_t last_input_ms;
    int64_t last_update_ms;
    bool running;
};

static int32_t saturating_add(int32_t a, int32_t b) {
    return (int32_t)CLAMP((int64_t)a + b, (int64_t)INT32_MIN, (int64_t)INT32_MAX);
}

static int32_t rate_from_sample(int32_t delta, int32_t dt_ms,
                                const struct scroll_nurunuru_config *config) {
    if (delta == 0 || config->scroll_tick == 0) {
        return 0;
    }

    int64_t numerator = (int64_t)delta * 1000 * VELOCITY_Q * config->sensitivity;
    int64_t denominator = (int64_t)dt_ms * config->scroll_tick * 2;
    return (int32_t)CLAMP(numerator / denominator, (int64_t)INT32_MIN,
                          (int64_t)INT32_MAX);
}

static int32_t approach(int32_t current, int32_t target, int32_t dt_ms, int32_t time_ms) {
    if (time_ms <= dt_ms) {
        return target;
    }
    return current + (int32_t)(((int64_t)(target - current) * dt_ms) / time_ms);
}

static int emit_axis(const struct device *input_device, uint16_t code, int64_t *phase_mq,
                     bool invert) {
    const int64_t one_event = (int64_t)1000 * VELOCITY_Q;
    int emitted = 0;

    while (llabs(*phase_mq) >= one_event && emitted < MAX_EVENTS_PER_TICK) {
        int direction = *phase_mq > 0 ? 1 : -1;
        *phase_mq -= direction * one_event;
        input_report_rel(input_device, code, invert ? -direction : direction, true,
                         K_MSEC(5));
        emitted++;
    }
    return emitted;
}

static void update_work_callback(struct k_work *work) {
    struct scroll_nurunuru_data *data =
        CONTAINER_OF(k_work_delayable_from_work(work), struct scroll_nurunuru_data,
                     update_work);
    const struct scroll_nurunuru_config *config = data->dev->config;

    k_mutex_lock(&data->lock, K_FOREVER);
    int64_t now_ms = k_uptime_get();
    int32_t dt_ms = data->last_update_ms > 0 ? (int32_t)(now_ms - data->last_update_ms)
                                             : UPDATE_MS;
    dt_ms = CLAMP(dt_ms, 1, 24);
    data->last_update_ms = now_ms;

    bool bridging = now_ms - data->last_input_ms <= config->bridge_ms;
    int32_t target_wheel_q = bridging ? data->target_wheel_q : 0;
    int32_t target_hwheel_q = bridging ? data->target_hwheel_q : 0;
    int32_t time_ms = bridging ? config->response_ms : config->release_ms;

    data->velocity_wheel_q =
        approach(data->velocity_wheel_q, target_wheel_q, dt_ms, time_ms);
    data->velocity_hwheel_q =
        approach(data->velocity_hwheel_q, target_hwheel_q, dt_ms, time_ms);
    data->phase_wheel_mq += (int64_t)data->velocity_wheel_q * dt_ms;
    data->phase_hwheel_mq += (int64_t)data->velocity_hwheel_q * dt_ms;

    if (data->input_device != NULL) {
        emit_axis(data->input_device, INPUT_REL_WHEEL, &data->phase_wheel_mq,
                  config->invert_vertical);
        emit_axis(data->input_device, INPUT_REL_HWHEEL, &data->phase_hwheel_mq,
                  config->invert_horizontal);
    }

    bool settled = !bridging && abs(data->velocity_wheel_q) < VELOCITY_Q / 32 &&
                   abs(data->velocity_hwheel_q) < VELOCITY_Q / 32 &&
                   llabs(data->phase_wheel_mq) < (int64_t)1000 * VELOCITY_Q &&
                   llabs(data->phase_hwheel_mq) < (int64_t)1000 * VELOCITY_Q;
    if (settled) {
        data->velocity_wheel_q = 0;
        data->velocity_hwheel_q = 0;
        data->phase_wheel_mq = 0;
        data->phase_hwheel_mq = 0;
        data->running = false;
        data->last_update_ms = 0;
    } else {
        k_work_reschedule(&data->update_work, K_MSEC(UPDATE_MS));
    }
    k_mutex_unlock(&data->lock);
}

static void process_sample(struct scroll_nurunuru_data *data,
                           const struct scroll_nurunuru_config *config, int64_t now_ms) {
    int32_t dt_ms = data->last_sample_ms > 0 ? (int32_t)(now_ms - data->last_sample_ms) : 16;
    dt_ms = CLAMP(dt_ms, 4, MAX_SAMPLE_MS);
    data->last_sample_ms = now_ms;
    data->last_input_ms = now_ms;

    data->target_wheel_q = rate_from_sample(data->pending_wheel, dt_ms, config);
    data->target_hwheel_q = rate_from_sample(data->pending_hwheel, dt_ms, config);
    data->pending_wheel = 0;
    data->pending_hwheel = 0;

    if (!data->running && (data->target_wheel_q != 0 || data->target_hwheel_q != 0)) {
        data->running = true;
        data->last_update_ms = now_ms;
        k_work_reschedule(&data->update_work, K_NO_WAIT);
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

    uint16_t code = event->code;
    int32_t value = event->value;
    bool sample_complete = event->sync;
    event->code = code == INPUT_REL_X ? INPUT_REL_WHEEL : INPUT_REL_HWHEEL;
    event->value = 0;

    k_mutex_lock(&data->lock, K_FOREVER);
    data->input_device = event->dev;
    if (code == INPUT_REL_X) {
        data->pending_wheel = saturating_add(data->pending_wheel, value);
    } else {
        data->pending_hwheel = saturating_add(data->pending_hwheel, value);
    }
    if (sample_complete) {
        process_sample(data, config, k_uptime_get());
    }
    k_mutex_unlock(&data->lock);
    return ZMK_INPUT_PROC_STOP;
}

static int scroll_nurunuru_init(const struct device *dev) {
    struct scroll_nurunuru_data *data = dev->data;
    data->dev = dev;
    k_mutex_init(&data->lock);
    k_work_init_delayable(&data->update_work, update_work_callback);
    LOG_INF("zmk-scroll-nurunuru 8 ms camera interpolation initialized");
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
        .response_ms = DT_INST_PROP(inst, response_ms),                          \
        .bridge_ms = DT_INST_PROP(inst, bridge_ms),                              \
        .release_ms = DT_INST_PROP(inst, release_ms),                            \
        .invert_horizontal = DT_INST_PROP_OR(inst, invert_horizontal, false),    \
        .invert_vertical = DT_INST_PROP_OR(inst, invert_vertical, false),        \
    };                                                                           \
    DEVICE_DT_INST_DEFINE(inst, scroll_nurunuru_init, NULL,                      \
                          &scroll_nurunuru_data_##inst,                           \
                          &scroll_nurunuru_config_##inst, POST_KERNEL,            \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                    \
                          &scroll_nurunuru_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_NURUNURU_INST)
