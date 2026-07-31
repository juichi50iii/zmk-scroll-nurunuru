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

LOG_MODULE_REGISTER(
    zmk_scroll_nurunuru,
    CONFIG_ZMK_SCROLL_NURUNURU_LOG_LEVEL
);

/*
 * Internal fixed-point scale.
 *
 * 1 scroll unit = 1024 internal units.
 */
#define NURUNURU_FP_SCALE 1024

/*
 * Gain calculation precision.
 *
 * 1000 = 1.000
 */
#define NURUNURU_GAIN_SCALE 1000

struct scroll_nurunuru_config {
    uint16_t report_interval_ms;
    uint16_t release_ms;

    int16_t horizontal_divisor;
    int16_t vertical_divisor;

    uint8_t velocity_smoothing;

    uint16_t acceleration_start;
    uint16_t acceleration_end;
    uint16_t max_gain_percent;

    uint8_t friction_percent;
    uint16_t inertia_start_speed;
    uint16_t inertia_timeout_ms;

    bool invert_horizontal;
    bool invert_vertical;
};

struct scroll_nurunuru_data {
    const struct device *dev;
    const struct device *input_device;

    /*
     * Raw movement received during the current frame.
     *
     * Sensor axes are swapped:
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
     * Fractional output retained between scroll reports.
     */
    int32_t output_horizontal_fp;
    int32_t output_vertical_fp;

    int32_t last_input_speed;
    bool input_was_active;

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

static int32_t abs_i32(int32_t value) {
    if (value == INT32_MIN) {
        return INT32_MAX;
    }

    return value < 0 ? -value : value;
}

static int32_t max_i32(int32_t a, int32_t b) {
    return a > b ? a : b;
}

/*
 * Smoothly move current toward target.
 *
 * smoothing = 100:
 *     immediate response
 *
 * smoothing = 40:
 *     apply 40 percent of the remaining difference each frame
 */
static int32_t smooth_toward(
    int32_t current,
    int32_t target,
    uint8_t smoothing
) {
    smoothing = clamp_percentage(smoothing);

    int64_t difference =
        (int64_t)target - current;

    return current +
           (int32_t)((difference * smoothing) / 100);
}

/*
 * Calculate acceleration gain using smoothstep.
 *
 * Below acceleration_start:
 *     gain = 1.0
 *
 * Above acceleration_end:
 *     gain = maximum
 *
 * Between them:
 *     gain changes continuously using:
 *
 *         smoothstep(t) = t² × (3 - 2t)
 */
static int32_t calculate_gain_scaled(
    int32_t speed,
    uint16_t acceleration_start,
    uint16_t acceleration_end,
    uint16_t max_gain_percent
) {
    int32_t maximum_gain =
        ((int32_t)max_gain_percent *
         NURUNURU_GAIN_SCALE) /
        100;

    if (maximum_gain < NURUNURU_GAIN_SCALE) {
        maximum_gain = NURUNURU_GAIN_SCALE;
    }

    if (acceleration_end <= acceleration_start) {
        return speed >= acceleration_start
                   ? maximum_gain
                   : NURUNURU_GAIN_SCALE;
    }

    if (speed <= acceleration_start) {
        return NURUNURU_GAIN_SCALE;
    }

    if (speed >= acceleration_end) {
        return maximum_gain;
    }

    int32_t range =
        acceleration_end - acceleration_start;

    int32_t position =
        speed - acceleration_start;

    /*
     * t uses NURUNURU_GAIN_SCALE precision.
     */
    int64_t t =
        ((int64_t)position *
         NURUNURU_GAIN_SCALE) /
        range;

    int64_t t_squared =
        (t * t) /
        NURUNURU_GAIN_SCALE;

    int64_t smoothstep =
        (t_squared *
         ((3 * NURUNURU_GAIN_SCALE) - (2 * t))) /
        NURUNURU_GAIN_SCALE;

    int32_t gain_range =
        maximum_gain - NURUNURU_GAIN_SCALE;

    return NURUNURU_GAIN_SCALE +
           (int32_t)(
               (smoothstep * gain_range) /
               NURUNURU_GAIN_SCALE
           );
}

static int32_t apply_gain(
    int32_t value,
    int32_t gain_scaled
) {
    int64_t result =
        ((int64_t)value * gain_scaled) /
        NURUNURU_GAIN_SCALE;

    return (int32_t)CLAMP(
        result,
        (int64_t)INT32_MIN,
        (int64_t)INT32_MAX
    );
}

static int32_t apply_friction(
    int32_t velocity_fp,
    uint8_t friction_percent
) {
    friction_percent =
        CLAMP(friction_percent, 0, 99);

    int64_t reduced =
        ((int64_t)velocity_fp * friction_percent) / 100;

    if (abs_i32((int32_t)reduced) < 8) {
        return 0;
    }

    return (int32_t)reduced;
}

static int32_t raw_to_velocity_fp(
    int32_t raw_value,
    int16_t divisor
) {
    if (divisor <= 0) {
        divisor = 1;
    }

    int64_t result =
        ((int64_t)raw_value *
         NURUNURU_FP_SCALE) /
        divisor;

    return (int32_t)CLAMP(
        result,
        (int64_t)INT32_MIN,
        (int64_t)INT32_MAX
    );
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

static void send_scroll_events(
    const struct device *input_device,
    int16_t horizontal,
    int16_t vertical
) {
    if (input_device == NULL) {
        return;
    }

    bool have_horizontal =
        horizontal != 0;

    bool have_vertical =
        vertical != 0;

    if (!have_horizontal && !have_vertical) {
        return;
    }

    int ret;

    if (have_horizontal) {
        ret = input_report_rel(
            input_device,
            INPUT_REL_HWHEEL,
            horizontal,
            !have_vertical,
            K_NO_WAIT
        );

        if (ret < 0) {
            LOG_WRN(
                "Failed to report horizontal scroll: %d",
                ret
            );
        }
    }

    if (have_vertical) {
        ret = input_report_rel(
            input_device,
            INPUT_REL_WHEEL,
            vertical,
            true,
            K_NO_WAIT
        );

        if (ret < 0) {
            LOG_WRN(
                "Failed to report vertical scroll: %d",
                ret
            );
        }
    }
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
    const struct device *input_device = NULL;

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

    int32_t speed = 0;
    int32_t gain_scaled =
        NURUNURU_GAIN_SCALE;

    if (input_is_active) {
        /*
         * Use the stronger axis as the gesture speed.
         *
         * This avoids diagonal movement receiving an unexpectedly larger
         * acceleration merely because both axes are active.
         */
        speed =
            max_i32(
                abs_i32(frame_horizontal),
                abs_i32(frame_vertical)
            );

        data->last_input_speed = speed;

        gain_scaled =
            calculate_gain_scaled(
                speed,
                config->acceleration_start,
                config->acceleration_end,
                config->max_gain_percent
            );

        int32_t accelerated_horizontal =
            apply_gain(
                frame_horizontal,
                gain_scaled
            );

        int32_t accelerated_vertical =
            apply_gain(
                frame_vertical,
                gain_scaled
            );

        int32_t target_horizontal_fp =
            raw_to_velocity_fp(
                accelerated_horizontal,
                config->horizontal_divisor
            );

        int32_t target_vertical_fp =
            raw_to_velocity_fp(
                accelerated_vertical,
                config->vertical_divisor
            );

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
         * Sensor reports can have gaps longer than the worker interval.
         * Do not interpret a single empty worker frame as release.
         *
         * During this grace period, retain the most recent velocity so
         * sparse low-speed sensor events are interpolated smoothly.
         */
        bool waiting_for_release =
            data->input_was_active &&
            idle_ms < config->release_ms;

        if (!waiting_for_release) {
            bool input_just_stopped =
                data->input_was_active;

            bool fast_enough_for_inertia =
                data->last_input_speed >=
                config->inertia_start_speed;

            if (
                input_just_stopped &&
                !fast_enough_for_inertia
            ) {
                /*
                 * Slow, deliberate movement stops only after the release
                 * grace period has elapsed.
                 */
                data->velocity_horizontal_fp = 0;
                data->velocity_vertical_fp = 0;
            } else {
                /*
                 * A sufficiently fast gesture continues with friction.
                 */
                data->velocity_horizontal_fp =
                    apply_friction(
                        data->velocity_horizontal_fp,
                        config->friction_percent
                    );

                data->velocity_vertical_fp =
                    apply_friction(
                        data->velocity_vertical_fp,
                        config->friction_percent
                    );
            }
        }
    }

    data->input_was_active =
        input_is_active ||
        (data->input_was_active && idle_ms < config->release_ms);

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

    bool velocity_is_active =
    data->velocity_horizontal_fp != 0 ||
    data->velocity_vertical_fp != 0;

    bool inertia_is_allowed =
        idle_ms < config->inertia_timeout_ms;

    bool continue_running =
        input_is_active ||
        (velocity_is_active && inertia_is_allowed);

    if (continue_running) {
        k_work_reschedule(
            &data->work,
            K_MSEC(config->report_interval_ms)
        );
    } else {
        data->worker_running = false;

        data->velocity_horizontal_fp = 0;
        data->velocity_vertical_fp = 0;
    }

    input_device =
        data->input_device;

    LOG_DBG(
        "frame=(%ld,%ld) speed=%ld gain=%ld velocity_fp=(%ld,%ld) output=(%d,%d) remainder_fp=(%ld,%ld) idle=%u",
        (long)frame_horizontal,
        (long)frame_vertical,
        (long)speed,
        (long)gain_scaled,
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

    send_scroll_events(
        input_device,
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

    uint16_t original_code =
        event->code;

    int32_t original_value =
        event->value;

    /*
     * Zephyr passes the same mutable input_event to every listener.
     *
     * ZMK_INPUT_PROC_STOP only stops this listener's processor chain.
     * Convert the original X/Y event into a zero-valued wheel event so
     * another listener cannot interpret it as cursor movement.
     */
    event->code =
        original_code == INPUT_REL_X
            ? INPUT_REL_WHEEL
            : INPUT_REL_HWHEEL;

    event->value = 0;

    if (original_value == 0) {
        return ZMK_INPUT_PROC_STOP;
    }

    k_mutex_lock(
        &data->lock,
        K_FOREVER
    );

    data->input_device =
        event->dev;

    /*
     * Swap sensor axes for scrolling.
     *
     * Physical X movement becomes vertical scrolling.
     * Physical Y movement becomes horizontal scrolling.
     */
    if (original_code == INPUT_REL_X) {
        data->pending_vertical +=
            original_value;
    } else {
        data->pending_horizontal +=
            original_value;
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
        original_code,
        original_value,
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
    data->input_device = NULL;

    data->pending_horizontal = 0;
    data->pending_vertical = 0;

    data->velocity_horizontal_fp = 0;
    data->velocity_vertical_fp = 0;

    data->output_horizontal_fp = 0;
    data->output_vertical_fp = 0;

    data->last_input_speed = 0;
    data->input_was_active = false;

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
            .release_ms =                                             \
                DT_INST_PROP_OR(                                      \
                    inst,                                             \
                    release_ms,                                       \
                    24                                                \
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
            .acceleration_start =                                     \
                DT_INST_PROP_OR(                                      \
                    inst,                                             \
                    acceleration_start,                               \
                    2                                                 \
                ),                                                    \
                                                                       \
            .acceleration_end =                                       \
                DT_INST_PROP_OR(                                      \
                    inst,                                             \
                    acceleration_end,                                 \
                    12                                                \
                ),                                                    \
                                                                       \
            .max_gain_percent =                                       \
                DT_INST_PROP_OR(                                       \
                    inst,                                              \
                    max_gain_percent,                                  \
                    300                                                \
                ),                                                     \
                                                                       \
            .friction_percent =                                        \
                DT_INST_PROP_OR(                                        \
                    inst,                                               \
                    friction_percent,                                   \
                    86                                                  \
                ),                                                      \
                                                                        \
            .inertia_start_speed =                                      \
                DT_INST_PROP_OR(                                        \
                    inst,                                               \
                    inertia_start_speed,                                \
                    6                                                   \
                ),                                                      \
                                                                        \
            .inertia_timeout_ms =                                       \
                DT_INST_PROP_OR(                                        \
                    inst,                                               \
                    inertia_timeout_ms,                                 \
                    300                                                 \
                ),                                                      \
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