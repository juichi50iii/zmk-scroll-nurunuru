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

#define NURUNURU_FP_SCALE 1024
#define NURUNURU_GAIN_SCALE 1000

#define NURUNURU_REPORT_INTERVAL_MS 8
#define NURUNURU_RELEASE_MS 40
#define NURUNURU_SCROLL_SCALE_STEP 24
#define NURUNURU_ACCELERATION_START 1
#define NURUNURU_ACCELERATION_END 10
#define NURUNURU_TRACKING_RESPONSE 70
#define NURUNURU_HOVER_FRAMES 6
#define NURUNURU_HOVER_STRENGTH_MULTIPLIER 3

/*
 * About 96 ms at an 8 ms report interval.
 */
#define NURUNURU_ROLLING_FULL_CHARGE_FRAMES 12

struct scroll_nurunuru_config {
    uint8_t scroll_scale;
    uint8_t acceleration;
    uint8_t hover;
    uint8_t inertia;
    uint8_t brake;

    bool invert_horizontal;
    bool invert_vertical;
};

struct scroll_nurunuru_data {
    const struct device *dev;
    const struct device *input_device;

    int32_t pending_horizontal;
    int32_t pending_vertical;

    int32_t velocity_horizontal_fp;
    int32_t velocity_vertical_fp;

    int32_t hover_horizontal_fp;
    int32_t hover_vertical_fp;
    uint8_t hover_frame;
    bool hover_active;

    /*
     * Continuous same-direction rolling builds additional momentum.
     */
    uint8_t rolling_frames;
    int8_t rolling_horizontal_direction;
    int8_t rolling_vertical_direction;

    int32_t output_horizontal_fp;
    int32_t output_vertical_fp;

    int32_t last_input_speed;
    bool input_was_active;

    uint32_t last_input_ms;
    bool worker_running;

    struct k_mutex lock;
    struct k_work_delayable work;
};

static uint8_t clamp_tuning(uint8_t value) {
    return CLAMP(value, 1, 10);
}

static int16_t clamp_to_int16(int32_t value) {
    return (int16_t)CLAMP(
        value,
        (int32_t)INT16_MIN,
        (int32_t)INT16_MAX
    );
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

static int8_t sign_i32(int32_t value) {
    if (value > 0) {
        return 1;
    }

    if (value < 0) {
        return -1;
    }

    return 0;
}

static int16_t get_scroll_divisor(
    const struct scroll_nurunuru_config *config
) {
    return (int16_t)(
        NURUNURU_SCROLL_SCALE_STEP *
        clamp_tuning(config->scroll_scale)
    );
}

static uint16_t get_max_gain_percent(
    const struct scroll_nurunuru_config *config
) {
    return (uint16_t)clamp_tuning(config->acceleration) * 100U;
}

static uint16_t get_inertia_start_speed(
    const struct scroll_nurunuru_config *config
) {
    return (uint16_t)(11 - clamp_tuning(config->inertia));
}

static uint16_t get_inertia_timeout_ms(
    const struct scroll_nurunuru_config *config
) {
    return (uint16_t)(500 + clamp_tuning(config->inertia) * 500);
}

static int32_t smooth_toward(
    int32_t current,
    int32_t target,
    uint8_t response
) {
    response = CLAMP(response, 1, 100);

    int64_t difference =
        (int64_t)target - current;

    return current +
           (int32_t)((difference * response) / 100);
}

static int32_t smoothstep_scaled(int32_t t_scaled) {
    t_scaled = CLAMP(
        t_scaled,
        0,
        NURUNURU_GAIN_SCALE
    );

    int64_t t_squared =
        ((int64_t)t_scaled * t_scaled) /
        NURUNURU_GAIN_SCALE;

    return (int32_t)(
        (t_squared *
         ((3 * NURUNURU_GAIN_SCALE) - (2 * t_scaled))) /
        NURUNURU_GAIN_SCALE
    );
}

static int32_t calculate_gain_scaled(
    int32_t speed,
    uint16_t max_gain_percent
) {
    int32_t maximum_gain =
        ((int32_t)max_gain_percent *
         NURUNURU_GAIN_SCALE) /
        100;

    maximum_gain =
        MAX(maximum_gain, NURUNURU_GAIN_SCALE);

    if (speed <= NURUNURU_ACCELERATION_START) {
        return NURUNURU_GAIN_SCALE;
    }

    if (speed >= NURUNURU_ACCELERATION_END) {
        return maximum_gain;
    }

    int32_t range =
        NURUNURU_ACCELERATION_END -
        NURUNURU_ACCELERATION_START;

    int32_t position =
        speed - NURUNURU_ACCELERATION_START;

    int32_t t =
        (int32_t)(
            ((int64_t)position * NURUNURU_GAIN_SCALE) /
            range
        );

    int32_t curve = smoothstep_scaled(t);
    int32_t gain_range = maximum_gain - NURUNURU_GAIN_SCALE;

    return NURUNURU_GAIN_SCALE +
           (int32_t)(
               ((int64_t)curve * gain_range) /
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

static int32_t raw_to_velocity_fp(
    int32_t raw_value,
    int16_t divisor
) {
    divisor = MAX(divisor, 1);

    int64_t result =
        ((int64_t)raw_value * NURUNURU_FP_SCALE) /
        divisor;

    return (int32_t)CLAMP(
        result,
        (int64_t)INT32_MIN,
        (int64_t)INT32_MAX
    );
}

/*
 * Hover is an initial push which overlaps normal input processing.
 *
 * It is strongest for small gestures and fades to zero over six frames.
 * Large gestures receive very little hover so acceleration remains in charge.
 */
static int32_t calculate_hover_force_fp(
    int32_t first_input_fp,
    int32_t input_speed,
    uint8_t hover,
    uint8_t hover_frame
) {
    if (hover_frame >= NURUNURU_HOVER_FRAMES) {
        return 0;
    }

    hover = clamp_tuning(hover);

    int32_t progress =
        (int32_t)(
            ((int64_t)hover_frame * NURUNURU_GAIN_SCALE) /
            NURUNURU_HOVER_FRAMES
        );

    int32_t time_factor =
        NURUNURU_GAIN_SCALE - smoothstep_scaled(progress);

    /*
     * Full hover at speed 1, fading to zero by speed 10.
     */
    int32_t speed_position = CLAMP(input_speed - 1, 0, 9);
    int32_t speed_t =
        (speed_position * NURUNURU_GAIN_SCALE) / 9;

    int32_t low_speed_factor =
        NURUNURU_GAIN_SCALE - smoothstep_scaled(speed_t);

    /*
     * hover 10 adds at most one extra copy of the first unaccelerated input.
     */
    int64_t force =
        (int64_t)first_input_fp *
        hover *
        time_factor *
        low_speed_factor;

    /*
     * Make hover three times stronger than the previous v5 behavior.
     */
    force *= NURUNURU_HOVER_STRENGTH_MULTIPLIER;

    force /=
        (int64_t)10 *
        NURUNURU_GAIN_SCALE *
        NURUNURU_GAIN_SCALE;

    return (int32_t)CLAMP(
        force,
        (int64_t)INT32_MIN,
        (int64_t)INT32_MAX
    );
}

/*
 * Same-direction continuous rolling gradually builds extra momentum.
 *
 * It is strongest at low speed and fades out toward speed 10, where the
 * ordinary acceleration curve already has control.
 *
 * inertia 1  -> up to about 1.1x
 * inertia 7  -> up to about 1.7x
 * inertia 10 -> up to about 2.0x
 */
static int32_t calculate_rolling_gain_scaled(
    uint8_t rolling_frames,
    int32_t speed,
    uint8_t inertia
) {
    uint8_t frames =
        MIN(
            rolling_frames,
            NURUNURU_ROLLING_FULL_CHARGE_FRAMES
        );

    int32_t progress =
        (int32_t)(
            ((int64_t)frames * NURUNURU_GAIN_SCALE) /
            NURUNURU_ROLLING_FULL_CHARGE_FRAMES
        );

    int32_t charge =
        smoothstep_scaled(progress);

    int32_t speed_position =
        CLAMP(speed - 1, 0, 9);

    int32_t speed_progress =
        (speed_position * NURUNURU_GAIN_SCALE) / 9;

    int32_t low_speed_factor =
        NURUNURU_GAIN_SCALE -
        smoothstep_scaled(speed_progress);

    int32_t extra_gain =
        ((int32_t)clamp_tuning(inertia) *
         NURUNURU_GAIN_SCALE) /
        10;

    int64_t added_gain =
        (int64_t)extra_gain *
        charge *
        low_speed_factor;

    added_gain /=
        (int64_t)NURUNURU_GAIN_SCALE *
        NURUNURU_GAIN_SCALE;

    return NURUNURU_GAIN_SCALE +
           (int32_t)added_gain;
}


/*
 * Velocity-dependent retention.
 *
 * inertia controls how much motion is preserved.
 * brake lowers retention near zero velocity, cleaning up the final tail.
 */
static uint8_t calculate_retention_percent(
    int32_t horizontal_velocity_fp,
    int32_t vertical_velocity_fp,
    const struct scroll_nurunuru_config *config
) {
    uint8_t inertia = clamp_tuning(config->inertia);
    uint8_t brake = clamp_tuning(config->brake);

    uint8_t fast_retention =
        (uint8_t)CLAMP(90 + inertia, 91, 99);

    uint8_t slow_retention =
        (uint8_t)CLAMP(
            (int32_t)fast_retention - ((int32_t)brake * 2),
            55,
            98
        );

    int32_t speed_fp =
        max_i32(
            abs_i32(horizontal_velocity_fp),
            abs_i32(vertical_velocity_fp)
        );

    const int32_t transition_end_fp =
        4 * NURUNURU_FP_SCALE;

    if (speed_fp >= transition_end_fp) {
        return fast_retention;
    }

    int32_t t =
        (int32_t)(
            ((int64_t)speed_fp * NURUNURU_GAIN_SCALE) /
            transition_end_fp
        );

    int32_t curve = smoothstep_scaled(t);
    int32_t range =
        (int32_t)fast_retention - slow_retention;

    return (uint8_t)CLAMP(
        (int32_t)slow_retention +
        (int32_t)(((int64_t)curve * range) /
                  NURUNURU_GAIN_SCALE),
        0,
        99
    );
}

static int32_t apply_retention(
    int32_t velocity_fp,
    uint8_t retention_percent
) {
    int64_t result =
        ((int64_t)velocity_fp * retention_percent) /
        100;

    if (abs_i32((int32_t)result) < 8) {
        return 0;
    }

    return (int32_t)result;
}

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

    bool have_horizontal = horizontal != 0;
    bool have_vertical = vertical != 0;

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

    const int16_t scroll_divisor =
        get_scroll_divisor(config);

    const uint16_t max_gain_percent =
        get_max_gain_percent(config);

    const uint16_t inertia_start_speed =
        get_inertia_start_speed(config);

    const uint16_t inertia_timeout_ms =
        get_inertia_timeout_ms(config);

    int16_t output_horizontal = 0;
    int16_t output_vertical = 0;
    const struct device *input_device = NULL;

    k_mutex_lock(&data->lock, K_FOREVER);

    int32_t frame_horizontal = data->pending_horizontal;
    int32_t frame_vertical = data->pending_vertical;

    data->pending_horizontal = 0;
    data->pending_vertical = 0;

    uint32_t now_ms = k_uptime_get_32();
    uint32_t idle_ms = now_ms - data->last_input_ms;

    bool input_is_active =
        frame_horizontal != 0 ||
        frame_vertical != 0;

    bool input_just_started =
        input_is_active && !data->input_was_active;

    int32_t speed = 0;
    int32_t gain_scaled = NURUNURU_GAIN_SCALE;
    uint8_t retention_percent = 0;

    if (input_is_active) {
        speed =
            max_i32(
                abs_i32(frame_horizontal),
                abs_i32(frame_vertical)
            );

        data->last_input_speed = speed;

        int8_t horizontal_direction =
            sign_i32(frame_horizontal);

        int8_t vertical_direction =
            sign_i32(frame_vertical);

        bool horizontal_reversed =
            horizontal_direction != 0 &&
            data->rolling_horizontal_direction != 0 &&
            horizontal_direction !=
                data->rolling_horizontal_direction;

        bool vertical_reversed =
            vertical_direction != 0 &&
            data->rolling_vertical_direction != 0 &&
            vertical_direction !=
                data->rolling_vertical_direction;

        if (
            input_just_started ||
            horizontal_reversed ||
            vertical_reversed
        ) {
            data->rolling_frames = 0;
        } else if (data->rolling_frames < UINT8_MAX) {
            data->rolling_frames++;
        }

        if (horizontal_direction != 0) {
            data->rolling_horizontal_direction =
                horizontal_direction;
        }

        if (vertical_direction != 0) {
            data->rolling_vertical_direction =
                vertical_direction;
        }

        int32_t raw_horizontal_fp =
            raw_to_velocity_fp(
                frame_horizontal,
                scroll_divisor
            );

        int32_t raw_vertical_fp =
            raw_to_velocity_fp(
                frame_vertical,
                scroll_divisor
            );

        if (input_just_started) {
            data->hover_horizontal_fp = raw_horizontal_fp;
            data->hover_vertical_fp = raw_vertical_fp;
            data->hover_frame = 0;
            data->hover_active = true;
        }

        gain_scaled =
            calculate_gain_scaled(
                speed,
                max_gain_percent
            );

        int32_t target_horizontal_fp =
            raw_to_velocity_fp(
                apply_gain(frame_horizontal, gain_scaled),
                scroll_divisor
            );

        int32_t target_vertical_fp =
            raw_to_velocity_fp(
                apply_gain(frame_vertical, gain_scaled),
                scroll_divisor
            );

        int32_t hover_horizontal_force_fp = 0;
        int32_t hover_vertical_force_fp = 0;

        if (data->hover_active) {
            hover_horizontal_force_fp =
                calculate_hover_force_fp(
                    data->hover_horizontal_fp,
                    speed,
                    config->hover,
                    data->hover_frame
                );

            hover_vertical_force_fp =
                calculate_hover_force_fp(
                    data->hover_vertical_fp,
                    speed,
                    config->hover,
                    data->hover_frame
                );

            data->hover_frame++;

            if (data->hover_frame >= NURUNURU_HOVER_FRAMES) {
                data->hover_active = false;
            }
        }

        /*
         * Current velocity carries the inertia from previous frames.
         * Input acceleration and hover overlap in the effective target.
         */
        target_horizontal_fp += hover_horizontal_force_fp;
        target_vertical_fp += hover_vertical_force_fp;

        int32_t rolling_gain_scaled =
            calculate_rolling_gain_scaled(
                data->rolling_frames,
                speed,
                config->inertia
            );

        target_horizontal_fp =
            apply_gain(
                target_horizontal_fp,
                rolling_gain_scaled
            );

        target_vertical_fp =
            apply_gain(
                target_vertical_fp,
                rolling_gain_scaled
            );

        data->velocity_horizontal_fp =
            smooth_toward(
                data->velocity_horizontal_fp,
                target_horizontal_fp,
                NURUNURU_TRACKING_RESPONSE
            );

        data->velocity_vertical_fp =
            smooth_toward(
                data->velocity_vertical_fp,
                target_vertical_fp,
                NURUNURU_TRACKING_RESPONSE
            );
    } else {
        bool waiting_for_release =
            data->input_was_active &&
            idle_ms < NURUNURU_RELEASE_MS;

        /*
         * Never hold velocity perfectly still during the release window.
         * A light, velocity-dependent retention prevents low-speed hovering.
         */
        retention_percent =
            calculate_retention_percent(
                data->velocity_horizontal_fp,
                data->velocity_vertical_fp,
                config
            );

        if (waiting_for_release) {
            retention_percent =
                (uint8_t)MIN(retention_percent + 1, 99);
        }

        bool fast_enough_for_inertia =
            data->last_input_speed >= inertia_start_speed;

        if (!waiting_for_release && !fast_enough_for_inertia) {
            data->velocity_horizontal_fp = 0;
            data->velocity_vertical_fp = 0;
        } else {
            data->velocity_horizontal_fp =
                apply_retention(
                    data->velocity_horizontal_fp,
                    retention_percent
                );

            data->velocity_vertical_fp =
                apply_retention(
                    data->velocity_vertical_fp,
                    retention_percent
                );
        }

        data->hover_active = false;

        /*
         * Keep rolling charge across short sensor gaps. Reset it only after
         * the release window has fully elapsed.
         */
        if (idle_ms >= NURUNURU_RELEASE_MS) {
            data->rolling_frames = 0;
            data->rolling_horizontal_direction = 0;
            data->rolling_vertical_direction = 0;
        }
    }

    data->input_was_active =
        input_is_active ||
        (data->input_was_active && idle_ms < NURUNURU_RELEASE_MS);

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
        idle_ms < inertia_timeout_ms;

    bool continue_running =
        input_is_active ||
        (velocity_is_active && inertia_is_allowed);

    if (continue_running) {
        k_work_reschedule(
            &data->work,
            K_MSEC(NURUNURU_REPORT_INTERVAL_MS)
        );
    } else {
        data->worker_running = false;
        data->velocity_horizontal_fp = 0;
        data->velocity_vertical_fp = 0;
        data->hover_active = false;
        data->rolling_frames = 0;
        data->rolling_horizontal_direction = 0;
        data->rolling_vertical_direction = 0;
    }

    input_device = data->input_device;

    LOG_DBG(
        "frame=(%ld,%ld) speed=%ld gain=%ld retention=%u hover=%u rolling=%u velocity=(%ld,%ld) output=(%d,%d) idle=%u",
        (long)frame_horizontal,
        (long)frame_vertical,
        (long)speed,
        (long)gain_scaled,
        retention_percent,
        data->hover_frame,
        data->rolling_frames,
        (long)data->velocity_horizontal_fp,
        (long)data->velocity_vertical_fp,
        output_horizontal,
        output_vertical,
        idle_ms
    );

    k_mutex_unlock(&data->lock);

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
    struct scroll_nurunuru_data *data = dev->data;

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

    uint16_t original_code = event->code;
    int32_t original_value = event->value;

    /* Prevent the original X/Y event from leaking into cursor movement. */
    event->code =
        original_code == INPUT_REL_X
            ? INPUT_REL_WHEEL
            : INPUT_REL_HWHEEL;

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

        k_work_reschedule(
            &data->work,
            K_MSEC(NURUNURU_REPORT_INTERVAL_MS)
        );
    }

    k_mutex_unlock(&data->lock);

    return ZMK_INPUT_PROC_STOP;
}

static int scroll_nurunuru_init(
    const struct device *dev
) {
    struct scroll_nurunuru_data *data = dev->data;

    data->dev = dev;
    data->input_device = NULL;

    data->pending_horizontal = 0;
    data->pending_vertical = 0;

    data->velocity_horizontal_fp = 0;
    data->velocity_vertical_fp = 0;

    data->hover_horizontal_fp = 0;
    data->hover_vertical_fp = 0;
    data->hover_frame = 0;
    data->hover_active = false;

    data->rolling_frames = 0;
    data->rolling_horizontal_direction = 0;
    data->rolling_vertical_direction = 0;

    data->output_horizontal_fp = 0;
    data->output_vertical_fp = 0;

    data->last_input_speed = 0;
    data->input_was_active = false;

    data->last_input_ms = 0;
    data->worker_running = false;

    k_mutex_init(&data->lock);

    k_work_init_delayable(
        &data->work,
        scroll_nurunuru_work_callback
    );

    LOG_INF("zmk-scroll-nurunuru force model initialized");

    return 0;
}

static const struct zmk_input_processor_driver_api
    scroll_nurunuru_driver_api = {
        .handle_event = scroll_nurunuru_handle_event,
    };

#define SCROLL_NURUNURU_INST(inst)                                     \
    static struct scroll_nurunuru_data                                 \
        scroll_nurunuru_data_##inst = {};                             \
                                                                       \
    static const struct scroll_nurunuru_config                         \
        scroll_nurunuru_config_##inst = {                             \
            .scroll_scale =                                           \
                DT_INST_PROP_OR(inst, scroll_scale, 7),               \
                                                                       \
            .acceleration =                                           \
                DT_INST_PROP_OR(inst, acceleration, 10),              \
                                                                       \
            .hover =                                                  \
                DT_INST_PROP_OR(inst, hover, 4),                      \
                                                                       \
            .inertia =                                                \
                DT_INST_PROP_OR(inst, inertia, 8),                    \
                                                                       \
            .brake =                                                  \
                DT_INST_PROP_OR(inst, brake, 3),                      \
                                                                       \
            .invert_horizontal =                                      \
                DT_INST_PROP_OR(inst, invert_horizontal, false),      \
                                                                       \
            .invert_vertical =                                        \
                DT_INST_PROP_OR(inst, invert_vertical, false),        \
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