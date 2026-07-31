/*
 * Copyright (c) 2026 Yuki Isogawa
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_scroll_nurunuru

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/input_processor.h>

#include <zmk/endpoints.h>
#include <zmk/hid.h>

LOG_MODULE_REGISTER(
    zmk_scroll_nurunuru,
    CONFIG_ZMK_SCROLL_NURUNURU_LOG_LEVEL
);

struct scroll_nurunuru_config {
    uint16_t report_interval_ms;

    int16_t horizontal_divisor;
    int16_t vertical_divisor;

    uint16_t idle_timeout_ms;

    bool invert_horizontal;
    bool invert_vertical;
};

struct scroll_nurunuru_data {
    const struct device *dev;

    /*
     * Raw physical movement waiting to be converted into scroll output.
     */
    int32_t pending_x;
    int32_t pending_y;

    /*
     * Remainders that were too small to form a complete scroll unit.
     */
    int32_t remainder_x;
    int32_t remainder_y;

    uint32_t last_input_ms;

    bool worker_running;

    struct k_mutex lock;
    struct k_work_delayable work;
};

static int16_t clamp_to_int16(int32_t value) {
    return (int16_t)CLAMP(
        value,
        (int32_t)INT16_MIN,
        (int32_t)INT16_MAX
    );
}

/*
 * Divide an accumulated raw movement value while retaining the remainder.
 *
 * Division in C truncates toward zero, which gives symmetrical behavior
 * for positive and negative scrolling.
 */
static int16_t extract_scroll_value(
    int32_t pending,
    int32_t *remainder,
    int16_t divisor
) {
    if (divisor <= 0) {
        divisor = 1;
    }

    int32_t accumulated =
        pending + *remainder;

    int32_t output =
        accumulated / divisor;

    *remainder =
        accumulated - (output * divisor);

    return clamp_to_int16(output);
}

static void send_scroll_report(
    int16_t horizontal,
    int16_t vertical
) {
    if (horizontal == 0 && vertical == 0) {
        return;
    }

    zmk_hid_mouse_scroll_set(
        horizontal,
        vertical
    );

    zmk_endpoints_send_mouse_report();

    /*
     * Clear the HID state immediately so this generated scroll value does
     * not leak into a later mouse-button or pointing-device report.
     */
    zmk_hid_mouse_scroll_set(0, 0);
}

static void scroll_nurunuru_work_callback(
    struct k_work *work
) {
    struct k_work_delayable *delayable =
        k_work_delayable_from_work(work);

    struct scroll_nurunuru_data *data =
        CONTAINER_OF(
            delayable,
            struct scroll_nurunuru_data,
            work
        );

    const struct scroll_nurunuru_config *config =
        data->dev->config;

    int16_t output_x = 0;
    int16_t output_y = 0;

    bool continue_running = false;

    k_mutex_lock(
        &data->lock,
        K_FOREVER
    );

    /*
     * Take all movement received since the previous frame.
     */
    int32_t frame_x =
        data->pending_x;

    int32_t frame_y =
        data->pending_y;

    data->pending_x = 0;
    data->pending_y = 0;

    output_x =
        extract_scroll_value(
            frame_x,
            &data->remainder_x,
            config->horizontal_divisor
        );

    output_y =
        extract_scroll_value(
            frame_y,
            &data->remainder_y,
            config->vertical_divisor
        );

    if (config->invert_horizontal) {
        output_x = -output_x;
    }

    if (config->invert_vertical) {
        output_y = -output_y;
    }

    uint32_t now_ms =
        k_uptime_get_32();

    uint32_t idle_ms =
        now_ms - data->last_input_ms;

    /*
     * Phase 1 has no inertia.
     *
     * Once physical input has stopped for idle_timeout_ms, the worker is
     * stopped. The accumulated fractional remainder is retained so tiny
     * movements are not lost between gestures.
     */
    if (idle_ms < config->idle_timeout_ms) {
        continue_running = true;

        k_work_reschedule(
            &data->work,
            K_MSEC(config->report_interval_ms)
        );
    } else {
        data->worker_running = false;
    }

    LOG_DBG(
        "frame=(%ld,%ld) output=(%d,%d) remainder=(%ld,%ld) idle=%u",
        (long)frame_x,
        (long)frame_y,
        output_x,
        output_y,
        (long)data->remainder_x,
        (long)data->remainder_y,
        idle_ms
    );

    k_mutex_unlock(
        &data->lock
    );

    ARG_UNUSED(continue_running);

    send_scroll_report(
        output_x,
        output_y
    );
}

static int scroll_nurunuru_handle_event(
    const struct device *dev,
    struct input_event *event,
    uint32_t param1,
    uint32_t param2,
    struct zmk_input_processor_state *processor_state
) {
    struct scroll_nurunuru_data *data =
        dev->data;

    const struct scroll_nurunuru_config *config =
        dev->config;

    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(processor_state);

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (
        event->code != INPUT_REL_X &&
        event->code != INPUT_REL_Y
    ) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->value == 0) {
        /*
         * The event belongs to the axes handled by this processor, so do
         * not allow it to become normal mouse movement later in the chain.
         */
        return ZMK_INPUT_PROC_STOP;
    }

    k_mutex_lock(
        &data->lock,
        K_FOREVER
    );

    if (event->code == INPUT_REL_X) {
        data->pending_y +=
            event->value;
    } else {
        data->pending_x +=
            event->value;
    }

    data->last_input_ms =
        k_uptime_get_32();

    if (!data->worker_running) {
        data->worker_running = true;

        /*
         * Wait until the next fixed frame rather than sending immediately.
         */
        k_work_reschedule(
            &data->work,
            K_MSEC(config->report_interval_ms)
        );
    }

    LOG_DBG(
        "input code=%u value=%d pending=(%ld,%ld)",
        event->code,
        event->value,
        (long)data->pending_x,
        (long)data->pending_y
    );

    k_mutex_unlock(
        &data->lock
    );

    /*
     * The physical X/Y event has been absorbed by nurunuru.
     *
     * It must not proceed to the ordinary mouse cursor output.
     */
    return ZMK_INPUT_PROC_STOP;
}

static int scroll_nurunuru_init(
    const struct device *dev
) {
    struct scroll_nurunuru_data *data =
        dev->data;

    data->dev = dev;

    data->pending_x = 0;
    data->pending_y = 0;

    data->remainder_x = 0;
    data->remainder_y = 0;

    data->last_input_ms = 0;
    data->worker_running = false;

    k_mutex_init(
        &data->lock
    );

    k_work_init_delayable(
        &data->work,
        scroll_nurunuru_work_callback
    );

    LOG_INF(
        "zmk-scroll-nurunuru initialized"
    );

    return 0;
}

static const struct zmk_input_processor_driver_api
    scroll_nurunuru_driver_api = {
        .handle_event =
            scroll_nurunuru_handle_event,
    };

#define SCROLL_NURUNURU_INST(inst)                                      \
    static struct scroll_nurunuru_data                                  \
        scroll_nurunuru_data_##inst = {};                              \
                                                                        \
    static const struct scroll_nurunuru_config                          \
        scroll_nurunuru_config_##inst = {                              \
            .report_interval_ms =                                      \
                DT_INST_PROP_OR(                                       \
                    inst,                                              \
                    report_interval_ms,                                \
                    8                                                  \
                ),                                                     \
                                                                        \
            .horizontal_divisor =                                      \
                DT_INST_PROP_OR(                                       \
                    inst,                                              \
                    horizontal_divisor,                                \
                    15                                                 \
                ),                                                     \
                                                                        \
            .vertical_divisor =                                        \
                DT_INST_PROP_OR(                                       \
                    inst,                                              \
                    vertical_divisor,                                  \
                    15                                                 \
                ),                                                     \
                                                                        \
            .idle_timeout_ms =                                         \
                DT_INST_PROP_OR(                                       \
                    inst,                                              \
                    idle_timeout_ms,                                   \
                    40                                                 \
                ),                                                     \
                                                                        \
            .invert_horizontal =                                       \
                DT_INST_PROP_OR(                                       \
                    inst,                                              \
                    invert_horizontal,                                 \
                    false                                              \
                ),                                                     \
                                                                        \
            .invert_vertical =                                         \
                DT_INST_PROP_OR(                                       \
                    inst,                                              \
                    invert_vertical,                                   \
                    false                                              \
                ),                                                     \
        };                                                             \
                                                                        \
    DEVICE_DT_INST_DEFINE(                                             \
        inst,                                                          \
        scroll_nurunuru_init,                                          \
        NULL,                                                          \
        &scroll_nurunuru_data_##inst,                                  \
        &scroll_nurunuru_config_##inst,                                \
        POST_KERNEL,                                                   \
        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                           \
        &scroll_nurunuru_driver_api                                    \
    );

DT_INST_FOREACH_STATUS_OKAY(
    SCROLL_NURUNURU_INST
)