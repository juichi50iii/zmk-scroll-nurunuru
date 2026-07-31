/*
 * Copyright (c) 2026 Yuki Isogawa
 *
 * SPDX-License-Identifier: MIT
 */

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

LOG_MODULE_REGISTER(
    zmk_scroll_nurunuru,
    CONFIG_ZMK_SCROLL_NURUNURU_LOG_LEVEL
);

/*
 * Internal fixed-point precision.
 *
 * 1 scroll unit = 1024 internal units.
 */
#define NURUNURU_FP_SCALE 1024

struct scroll_nurunuru_config {
    uint16_t report_interval_ms;

    int16_t horizontal_divisor;
    int16_t vertical_divisor;

    uint8_t velocity_smoothing;

    uint16_t idle_timeout_ms;

    bool invert_horizontal;
    bool invert_vertical;
};

struct scroll_nurunuru_data {
    const struct device *dev;

    /*
     * Raw movement received since the previous frame.
     *
     * The sensor axes are swapped intentionally:
     *
     * INPUT_REL_X -> vertical scroll
     * INPUT_REL_Y -> horizontal scroll
     */
    int32_t pending_horizontal;
    int32_t pending_vertical;

    /*
     * Smoothed velocity in fixed-point scroll units per frame.
     */
    int32_t velocity_horizontal_fp;
    int32_t velocity_vertical_fp;

    /*
     * Fractional scroll output retained between HID reports.
     */
    int32_t output_horizontal_fp;
    int32_t output_vertical_fp;

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

static uint8_t clamp_percentage(uint8_t value) {
    return CLAMP(value, 1, 100);
}

/*
 * Move current toward target by a percentage.
 *
 * smoothing = 100:
 *     current immediately becomes target.
 *
 * smoothing = 40:
 *     40% of the difference is applied each frame.
 */
static int32_t smooth_toward(
    int32_t current,
    int32_t target,
    uint8_t smoothing
) {
    smoothing = clamp_percentage(smoothing);

    return current +
           ((target - current) * smoothing) / 100;
}

static int32_t raw_to_velocity_fp(
    int32_t raw_value,
    int16_t divisor
) {
    if (divisor <= 0) {
        divisor = 1;
    }

    return (raw_value * NURUNURU_FP_SCALE) / divisor;
}

/*
 * Convert fixed-point output into integer HID scroll units while retaining
 * the fractional remainder.
 */
static int16_t extract_scroll_output(
    int32_t *accumulator_fp
) {
    int32_t output =
        *accumulator_fp / NURUNURU_FP_SCALE;

    *accumulator_fp -=
        output * NURUNURU_FP_SCALE;

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
     * Clear the stored scroll state so it is not included in a later mouse
     * report.
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

    int16_t output_horizontal = 0;
    int16_t output_vertical = 0;

    k_mutex_lock(
        &data->lock,
        K_FOREVER
    );

    /*
     * Capture everything received during this frame.
     */
    int32_t frame_horizontal =
        data->pending_horizontal;

    int32_t frame_vertical =
        data->pending_vertical;

    data->pending_horizontal = 0;
    data->pending_vertical = 0;

    uint32_t now_ms =
        k_uptime_get_32();

    uint32_t idle_ms =
        now_ms - data->last_input_ms;

    bool input_is_active =
        frame_horizontal != 0 ||
        frame_vertical != 0;

    if (input_is_active) {
        /*
         * Convert this frame's raw movement into a target velocity.
         */
        int32_t target_horizontal_fp =
            raw_to_velocity_fp(
                frame_horizontal,
                config->horizontal_divisor
            );

        int32_t target_vertical_fp =
            raw_to_velocity_fp(
                frame_vertical,
                config->vertical_divisor
            );

        /*
         * Smoothly follow the newest measured velocity.
         */
        data->velocity_horizontal_fp =
            smooth_toward(
                data->velocity_horizontal_fp,
                target_horizontal_fp,
                config->velocity_smoothing
            );

        data->velocity_vertical_fp =
            smooth_toward(
                data->velocity_vertical_fp,
                target_vertical_fp,
                config->velocity_smoothing
            );
    } else {
        /*
         * Phase 2 deliberately has no inertia.
         *
         * Once no new physical movement arrives, velocity stops.
         */
        data->velocity_horizontal_fp = 0;
        data->velocity_vertical_fp = 0;
    }

    data->output_horizontal_fp +=
        data->velocity_horizontal_fp;

    data->output_vertical_fp +=
        data->velocity_vertical_fp;

    output_horizontal =
        extract_scroll_output(
            &data->output_horizontal_fp
        );

    output_vertical =
        extract_scroll_output(
            &data->output_vertical_fp
        );

    if (config->invert_horizontal) {
        output_horizontal = -output_horizontal;
    }

    if (config->invert_vertical) {
        output_vertical = -output_vertical;
    }

    bool continue_running =
        idle_ms < config->idle_timeout_ms;

    if (continue_running) {
        k_work_reschedule(
            &data->work,
            K_MSEC(config->report_interval_ms)
        );
    } else {
        data->worker_running = false;

        /*
         * Phase 2 has no coasting, so clear the velocity when the gesture
         * has completely ended.
         */
        data->velocity_horizontal_fp = 0;
        data->velocity_vertical_fp = 0;
    }

    LOG_DBG(
        "frame=(%ld,%ld) velocity_fp=(%ld,%ld) output=(%d,%d) remainder_fp=(%ld,%ld) idle=%u",
        (long)frame_horizontal,
        (long)frame_vertical,
        (long)data->velocity_horizontal_fp,
        (long)data->velocity_vertical_fp,
        output_horizontal,
        output_vertical,
        (long)data->output_horizontal_fp,
        (long)data->output_vertical_fp,
        idle_ms
    );

    k_mutex_unlock(
        &data->lock
    );

    send_scroll_report(
        output_horizontal,
        output_vertical
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
        return ZMK_INPUT_PROC_STOP;
    }

    k_mutex_lock(
        &data->lock,
        K_FOREVER
    );

    /*
     * Swap the sensor axes for scroll use.
     *
     * Physical X movement becomes vertical scrolling.
     * Physical Y movement becomes horizontal scrolling.
     */
    if (event->code == INPUT_REL_X) {
        data->pending_vertical +=
            event->value;
    } else {
        data->pending_horizontal +=
            event->value;
    }

    data->last_input_ms =
        k_uptime_get_32();

    if (!data->worker_running) {
        data->worker_running = true;

        k_work_reschedule(
            &data->work,
            K_MSEC(config->report_interval_ms)
        );
    }

    LOG_DBG(
        "input code=%u value=%d pending=(%ld,%ld)",
        event->code,
        event->value,
        (long)data->pending_horizontal,
        (long)data->pending_vertical
    );

    k_mutex_unlock(
        &data->lock
    );

    return ZMK_INPUT_PROC_STOP;
}

static int scroll_nurunuru_init(
    const struct device *dev
) {
    struct scroll_nurunuru_data *data =
        dev->data;

    data->dev = dev;

    data->pending_horizontal = 0;
    data->pending_vertical = 0;

    data->velocity_horizontal_fp = 0;
    data->velocity_vertical_fp = 0;

    data->output_horizontal_fp = 0;
    data->output_vertical_fp = 0;

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

#define SCROLL_NURUNURU_INST(inst)                                     \
    static struct scroll_nurunuru_data                                 \
        scroll_nurunuru_data_##inst = {};                             \
                                                                       \
    static const struct scroll_nurunuru_config                         \
        scroll_nurunuru_config_##inst = {                             \
            .report_interval_ms =                                     \
                DT_INST_PROP_OR(                                      \
                    inst,                                             \
                    report_interval_ms,                               \
                    8                                                 \
                ),                                                    \
                                                                       \
            .horizontal_divisor =                                     \
                DT_INST_PROP_OR(                                      \
                    inst,                                             \
                    horizontal_divisor,                               \
                    15                                                \
                ),                                                    \
                                                                       \
            .vertical_divisor =                                       \
                DT_INST_PROP_OR(                                      \
                    inst,                                             \
                    vertical_divisor,                                 \
                    15                                                \
                ),                                                    \
                                                                       \
            .velocity_smoothing =                                     \
                DT_INST_PROP_OR(                                      \
                    inst,                                             \
                    velocity_smoothing,                               \
                    40                                                \
                ),                                                    \
                                                                       \
            .idle_timeout_ms =                                        \
                DT_INST_PROP_OR(                                      \
                    inst,                                             \
                    idle_timeout_ms,                                  \
                    40                                                \
                ),                                                    \
                                                                       \
            .invert_horizontal =                                      \
                DT_INST_PROP_OR(                                      \
                    inst,                                             \
                    invert_horizontal,                                \
                    false                                             \
                ),                                                    \
                                                                       \
            .invert_vertical =                                        \
                DT_INST_PROP_OR(                                      \
                    inst,                                             \
                    invert_vertical,                                  \
                    false                                             \
                ),                                                    \
        };                                                            \
                                                                       \
    DEVICE_DT_INST_DEFINE(                                            \
        inst,                                                         \
        scroll_nurunuru_init,                                         \
        NULL,                                                         \
        &scroll_nurunuru_data_##inst,                                 \
        &scroll_nurunuru_config_##inst,                               \
        POST_KERNEL,                                                  \
        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                          \
        &scroll_nurunuru_driver_api                                   \
    );

DT_INST_FOREACH_STATUS_OKAY(
    SCROLL_NURUNURU_INST
)