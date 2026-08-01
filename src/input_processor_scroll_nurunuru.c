#define DT_DRV_COMPAT zmk_input_processor_scroll_nurunuru

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/input_processor.h>
#include <zmk/endpoints.h>
#include <zmk/hid.h>

LOG_MODULE_REGISTER(zmk_scroll_nurunuru, CONFIG_ZMK_SCROLL_NURUNURU_LOG_LEVEL);

#define TOUCH_CENTER 16384
#define TOUCH_MIN 4096
#define TOUCH_MAX 28671
#define TOUCH_RELEASE_MS 56

struct scroll_nurunuru_config {
    uint8_t touch_scale;
    bool invert_horizontal;
    bool invert_vertical;
};

struct scroll_nurunuru_data {
    const struct device *dev;
    struct k_mutex lock;
    struct k_work_delayable release_work;
    int32_t pending_x;
    int32_t pending_y;
    int32_t touch_x;
    int32_t touch_y;
    bool touching;
};

static int32_t saturating_add(int32_t a, int32_t b) {
    return (int32_t)CLAMP((int64_t)a + b, (int64_t)INT32_MIN, (int64_t)INT32_MAX);
}

static void send_touch(struct scroll_nurunuru_data *data, bool touching) {
    zmk_hid_touchpad_set((uint16_t)data->touch_x, (uint16_t)data->touch_y, touching);
    zmk_endpoints_send_touchpad_report();
}

static void release_work_callback(struct k_work *work) {
    struct scroll_nurunuru_data *data =
        CONTAINER_OF(k_work_delayable_from_work(work), struct scroll_nurunuru_data,
                     release_work);

    k_mutex_lock(&data->lock, K_FOREVER);
    if (data->touching) {
        send_touch(data, false);
        data->touching = false;
        data->touch_x = TOUCH_CENTER;
        data->touch_y = TOUCH_CENTER;
    }
    k_mutex_unlock(&data->lock);
}

static void process_sample(struct scroll_nurunuru_data *data,
                           const struct scroll_nurunuru_config *config) {
    int32_t dx = data->pending_y * config->touch_scale;
    int32_t dy = data->pending_x * config->touch_scale;
    data->pending_x = 0;
    data->pending_y = 0;

    if (config->invert_horizontal) {
        dx = -dx;
    }
    if (config->invert_vertical) {
        dy = -dy;
    }
    if (dx == 0 && dy == 0) {
        return;
    }

    if (!data->touching) {
        data->touch_x = TOUCH_CENTER;
        data->touch_y = TOUCH_CENTER;
        data->touching = true;
        send_touch(data, true);
    }

    int32_t next_x = data->touch_x + dx;
    int32_t next_y = data->touch_y + dy;
    if (next_x < TOUCH_MIN || next_x > TOUCH_MAX ||
        next_y < TOUCH_MIN || next_y > TOUCH_MAX) {
        send_touch(data, false);
        data->touch_x = TOUCH_CENTER;
        data->touch_y = TOUCH_CENTER;
        send_touch(data, true);
        next_x = data->touch_x + dx;
        next_y = data->touch_y + dy;
    }

    data->touch_x = CLAMP(next_x, TOUCH_MIN, TOUCH_MAX);
    data->touch_y = CLAMP(next_y, TOUCH_MIN, TOUCH_MAX);
    send_touch(data, true);
    k_work_reschedule(&data->release_work, K_MSEC(TOUCH_RELEASE_MS));
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
    if (code == INPUT_REL_X) {
        data->pending_x = saturating_add(data->pending_x, value);
    } else {
        data->pending_y = saturating_add(data->pending_y, value);
    }
    if (sample_complete) {
        process_sample(data, config);
    }
    k_mutex_unlock(&data->lock);
    return ZMK_INPUT_PROC_STOP;
}

static int scroll_nurunuru_init(const struct device *dev) {
    struct scroll_nurunuru_data *data = dev->data;
    data->dev = dev;
    data->touch_x = TOUCH_CENTER;
    data->touch_y = TOUCH_CENTER;
    k_mutex_init(&data->lock);
    k_work_init_delayable(&data->release_work, release_work_callback);
    LOG_INF("zmk-scroll-nurunuru virtual touchpad initialized");
    return 0;
}

static const struct zmk_input_processor_driver_api scroll_nurunuru_driver_api = {
    .handle_event = scroll_nurunuru_handle_event,
};

#define SCROLL_NURUNURU_INST(inst)                                               \
    static struct scroll_nurunuru_data scroll_nurunuru_data_##inst = {};         \
    static const struct scroll_nurunuru_config scroll_nurunuru_config_##inst = { \
        .touch_scale = DT_INST_PROP(inst, touch_scale),                          \
        .invert_horizontal = DT_INST_PROP_OR(inst, invert_horizontal, false),    \
        .invert_vertical = DT_INST_PROP_OR(inst, invert_vertical, false),        \
    };                                                                           \
    DEVICE_DT_INST_DEFINE(inst, scroll_nurunuru_init, NULL,                      \
                          &scroll_nurunuru_data_##inst,                           \
                          &scroll_nurunuru_config_##inst, POST_KERNEL,            \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                    \
                          &scroll_nurunuru_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_NURUNURU_INST)
