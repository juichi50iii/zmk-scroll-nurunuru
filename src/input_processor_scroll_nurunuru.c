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
 * About 160 ms at an 8 ms report interval.
 */
#define NURUNURU_ROLLING_FULL_CHARGE_FRAMES 38
#define NURUNURU_ROLLING_LPF_RESPONSE 12
#define NURUNURU_ROLLING_CRUISE_RESPONSE 8
#define NURUNURU_ROLLING_RESPONSE 16
#define NURUNURU_ROLLING_VELOCITY_RESPONSE 10
#define NURUNURU_FLICK_VELOCITY_RESPONSE 70
#define NURUNURU_REVERSE_STOP_MS 150

/*
 * Limit frame-to-frame velocity change.
 *
 * ROLLING is deliberately softer.
 * FLICK remains responsive while avoiding sudden jumps.
 */
#define NURUNURU_ROLLING_MAX_VELOCITY_CHANGE_FP 48
#define NURUNURU_FLICK_MAX_VELOCITY_CHANGE_FP 256

/*
 * ROLLING-specific smoothing.
 *
 * Acceleration is allowed to follow relatively quickly.
 * Deceleration and temporary sensor gaps are deliberately much slower.
 */
#define NURUNURU_ROLLING_ACCEL_RESPONSE 18
#define NURUNURU_ROLLING_DECEL_RESPONSE 4

/*
 * Minimum virtual cruise velocity while ROLLING input is active.
 * This prevents low-speed sensor chatter from becoming visible stop/start.
 */
#define NURUNURU_ROLLING_MIN_CRUISE_FP 96

/*
 * After ROLLING input ends, preserve the last cruise velocity briefly before
 * normal inertia takes over.
 */
#define NURUNURU_ROLLING_COAST_BRIDGE_MS 120
#define NURUNURU_ROLLING_BRIDGE_RETENTION_PER_MILLE 995

#define NURUNURU_FIXED_SCROLL_DIVISOR 24
#define NURUNURU_FIXED_BRAKE 1

#define NURUNURU_ROLL_DETECT_FRAMES 6
#define NURUNURU_FLICK_SPEED_THRESHOLD 8
#define NURUNURU_ROLL_MAX_SPEED 5
#define NURUNURU_ROLL_MAX_PEAK_SPEED 7

enum scroll_nurunuru_gesture_mode {
    NURUNURU_GESTURE_UNDECIDED = 0,
    NURUNURU_GESTURE_FLICK,
    NURUNURU_GESTURE_ROLLING,
};

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
     * Continuous same-direction rolling builds momentum gradually.
     * The momentum is blended into velocity while input is active.
     */
    uint8_t rolling_frames;
    int8_t rolling_horizontal_direction;
    int8_t rolling_vertical_direction;
    int32_t rolling_horizontal_fp;
    int32_t rolling_vertical_fp;

    /* Rolling-only low-pass and cruise states. */
    int32_t rolling_lpf_horizontal_fp;
    int32_t rolling_lpf_vertical_fp;
    int32_t cruise_horizontal_fp;
    int32_t cruise_vertical_fp;

    enum scroll_nurunuru_gesture_mode gesture_mode;

    /*
     * Keep the selected output scale through the inertia tail.
     * ROLLING uses 1/1; FLICK and undecided/flick-like motion use 1/3.
     */
    enum scroll_nurunuru_gesture_mode output_mode;

    /*
     * Preserve ROLLING cruise briefly after release so the transition into
     * inertia is hidden instead of feeling like a mode switch.
     */
    bool rolling_bridge_active;
    uint32_t rolling_bridge_started_ms;
    int32_t rolling_bridge_horizontal_fp;
    int32_t rolling_bridge_vertical_fp;

    uint8_t gesture_frames;
    int32_t gesture_peak_speed;

    /*
     * Reverse input immediately stops the corresponding axis and suppresses
     * new output for a short period.
     */
    uint32_t horizontal_stop_until_ms;
    uint32_t vertical_stop_until_ms;

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
    ARG_UNUSED(config);
    return NURUNURU_FIXED_SCROLL_DIVISOR;
}

static uint16_t get_max_gain_percent(
    const struct scroll_nurunuru_config *config
) {
    return (uint16_t)clamp_tuning(config->acceleration) * 100U;
}

/*
 * Public inertia is intentionally front-loaded:
 *
 * public 1 -> internal 2
 * public 2 -> internal 4
 * public 3 -> internal 6
 * public 4 -> internal 8
 * public 5..10 -> internal 10
 *
 * Therefore inertia = 5 gives approximately the previous inertia = 10
 * behavior, leaving the upper half of the public range as "maximum glide".
 */
static uint8_t get_effective_inertia(
    const struct scroll_nurunuru_config *config
) {
    return (uint8_t)MIN(
        clamp_tuning(config->inertia) * 2,
        10
    );
}

static uint16_t get_inertia_start_speed(
    const struct scroll_nurunuru_config *config
) {
    return (uint16_t)(
        11 - get_effective_inertia(config)
    );
}

static uint16_t get_inertia_timeout_ms(
    const struct scroll_nurunuru_config *config
) {
    return (uint16_t)(
        500 + get_effective_inertia(config) * 500
    );
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

static int32_t limit_velocity_change(
    int32_t current,
    int32_t target,
    int32_t maximum_change_fp
) {
    if (maximum_change_fp <= 0) {
        return target;
    }

    int64_t difference =
        (int64_t)target - current;

    difference = CLAMP(
        difference,
        -(int64_t)maximum_change_fp,
        (int64_t)maximum_change_fp
    );

    return current + (int32_t)difference;
}

static int32_t smooth_toward_asymmetric(
    int32_t current,
    int32_t target,
    uint8_t acceleration_response,
    uint8_t deceleration_response
) {
    uint8_t response =
        abs_i32(target) > abs_i32(current)
            ? acceleration_response
            : deceleration_response;

    return smooth_toward(
        current,
        target,
        response
    );
}

static int32_t enforce_minimum_cruise(
    int32_t velocity_fp,
    int32_t input_value,
    int32_t minimum_cruise_fp
) {
    if (
        input_value == 0 ||
        minimum_cruise_fp <= 0
    ) {
        return velocity_fp;
    }

    int8_t direction =
        sign_i32(input_value);

    if (direction == 0) {
        return velocity_fp;
    }

    if (abs_i32(velocity_fp) >= minimum_cruise_fp) {
        return velocity_fp;
    }

    return (int32_t)direction *
           minimum_cruise_fp;
}

static int32_t apply_retention_per_mille(
    int32_t value,
    uint16_t retention_per_mille
) {
    retention_per_mille =
        CLAMP(retention_per_mille, 0, 999);

    int64_t result =
        ((int64_t)value *
         retention_per_mille) /
        1000;

    if (abs_i32((int32_t)result) < 8) {
        return 0;
    }

    return (int32_t)result;
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
static int32_t calculate_rolling_input_scale(
    uint8_t rolling_frames
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

    int32_t curve =
        smoothstep_scaled(progress);

    /*
     * Heavy beginning, then a calm low-speed cruise.
     * 0.60x -> 1.00x over roughly 300 ms.
     */
    const int32_t minimum_scale = 600;
    const int32_t maximum_scale = 1000;

    return minimum_scale +
           (int32_t)(
               ((int64_t)(maximum_scale - minimum_scale) * curve) /
               NURUNURU_GAIN_SCALE
           );
}

/*
 * Momentum is no longer an accelerator. It gently fills short gaps and keeps
 * the cruise velocity continuous.
 */
static int32_t calculate_rolling_momentum_target_fp(
    int32_t cruise_fp,
    uint8_t inertia
) {
    int32_t effective = clamp_tuning(inertia);

    /* inertia 10 -> at most 25% of cruise velocity. */
    int32_t strength_scaled =
        (effective * NURUNURU_GAIN_SCALE) / 40;

    return apply_gain(
        cruise_fp,
        strength_scaled
    );
}

static int32_t calculate_rolling_target_fp(
    int32_t raw_input_fp,
    uint8_t rolling_frames,
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

    int32_t strength_scaled =
        ((int32_t)clamp_tuning(inertia) *
         NURUNURU_GAIN_SCALE *
         9) /
        100;

    int64_t target =
        (int64_t)raw_input_fp *
        strength_scaled *
        charge;

    target /=
        (int64_t)NURUNURU_GAIN_SCALE *
        NURUNURU_GAIN_SCALE;

    return (int32_t)CLAMP(
        target,
        (int64_t)INT32_MIN,
        (int64_t)INT32_MAX
    );
}


static enum scroll_nurunuru_gesture_mode classify_gesture(
    enum scroll_nurunuru_gesture_mode current_mode,
    uint8_t gesture_frames,
    int32_t current_speed,
    int32_t peak_speed
) {
    if (current_mode != NURUNURU_GESTURE_UNDECIDED) {
        return current_mode;
    }

    if (current_speed >= NURUNURU_FLICK_SPEED_THRESHOLD) {
        return NURUNURU_GESTURE_FLICK;
    }

    if (
        gesture_frames >= NURUNURU_ROLL_DETECT_FRAMES &&
        current_speed <= NURUNURU_ROLL_MAX_SPEED &&
        peak_speed <= NURUNURU_ROLL_MAX_PEAK_SPEED
    ) {
        return NURUNURU_GESTURE_ROLLING;
    }

    if (gesture_frames >= 12) {
        return NURUNURU_GESTURE_FLICK;
    }

    return NURUNURU_GESTURE_UNDECIDED;
}

static bool timestamp_is_future(
    uint32_t until_ms,
    uint32_t now_ms
) {
    return (int32_t)(until_ms - now_ms) > 0;
}

static bool directions_are_opposite(
    int32_t input_value,
    int32_t velocity_fp
) {
    return input_value != 0 &&
           velocity_fp != 0 &&
           sign_i32(input_value) != sign_i32(velocity_fp);
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
    uint8_t inertia = get_effective_inertia(config);
    uint8_t brake = NURUNURU_FIXED_BRAKE;

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

    bool horizontal_stopped =
        timestamp_is_future(
            data->horizontal_stop_until_ms,
            now_ms
        );

    bool vertical_stopped =
        timestamp_is_future(
            data->vertical_stop_until_ms,
            now_ms
        );

    /*
     * Any opposite-direction input immediately kills motion on that axis.
     * The triggering input is consumed and the axis remains stopped for
     * NURUNURU_REVERSE_STOP_MS.
     */
    if (
        !horizontal_stopped &&
        directions_are_opposite(
            frame_horizontal,
            data->velocity_horizontal_fp
        )
    ) {
        data->velocity_horizontal_fp = 0;
        data->rolling_horizontal_fp = 0;
        data->rolling_lpf_horizontal_fp = 0;
        data->cruise_horizontal_fp = 0;
        data->output_horizontal_fp = 0;
        data->hover_horizontal_fp = 0;
        data->rolling_horizontal_direction = 0;
        data->gesture_mode = NURUNURU_GESTURE_UNDECIDED;
        data->output_mode = NURUNURU_GESTURE_FLICK;
        data->gesture_frames = 0;
        data->gesture_peak_speed = 0;

        data->horizontal_stop_until_ms =
            now_ms + NURUNURU_REVERSE_STOP_MS;

        frame_horizontal = 0;
        horizontal_stopped = true;
    }

    if (
        !vertical_stopped &&
        directions_are_opposite(
            frame_vertical,
            data->velocity_vertical_fp
        )
    ) {
        data->velocity_vertical_fp = 0;
        data->rolling_vertical_fp = 0;
        data->rolling_lpf_vertical_fp = 0;
        data->cruise_vertical_fp = 0;
        data->output_vertical_fp = 0;
        data->hover_vertical_fp = 0;
        data->rolling_vertical_direction = 0;
        data->gesture_mode = NURUNURU_GESTURE_UNDECIDED;
        data->output_mode = NURUNURU_GESTURE_FLICK;
        data->gesture_frames = 0;
        data->gesture_peak_speed = 0;

        data->vertical_stop_until_ms =
            now_ms + NURUNURU_REVERSE_STOP_MS;

        frame_vertical = 0;
        vertical_stopped = true;
    }

    if (horizontal_stopped) {
        frame_horizontal = 0;
        data->velocity_horizontal_fp = 0;
        data->rolling_horizontal_fp = 0;
        data->rolling_lpf_horizontal_fp = 0;
        data->cruise_horizontal_fp = 0;
        data->output_horizontal_fp = 0;
        data->hover_horizontal_fp = 0;
    }

    if (vertical_stopped) {
        frame_vertical = 0;
        data->velocity_vertical_fp = 0;
        data->rolling_vertical_fp = 0;
        data->rolling_lpf_vertical_fp = 0;
        data->cruise_vertical_fp = 0;
        data->output_vertical_fp = 0;
        data->hover_vertical_fp = 0;
    }

    bool input_is_active =
        frame_horizontal != 0 ||
        frame_vertical != 0;

    bool input_just_started =
        input_is_active &&
        !data->input_was_active;

    int32_t speed =
        max_i32(
            abs_i32(frame_horizontal),
            abs_i32(frame_vertical)
        );

    int32_t gain_scaled = NURUNURU_GAIN_SCALE;
    uint8_t retention_percent = 0;

    if (input_is_active) {
        /*
         * Any fresh input takes control immediately.
         */
        data->rolling_bridge_active = false;
        data->last_input_speed = speed;

        if (input_just_started) {
            data->gesture_mode =
                NURUNURU_GESTURE_UNDECIDED;
            data->gesture_frames = 1;
            data->gesture_peak_speed = speed;
        } else {
            if (data->gesture_frames < UINT8_MAX) {
                data->gesture_frames++;
            }

            data->gesture_peak_speed =
                max_i32(
                    data->gesture_peak_speed,
                    speed
                );
        }

        data->gesture_mode =
            classify_gesture(
                data->gesture_mode,
                data->gesture_frames,
                speed,
                data->gesture_peak_speed
            );

        /*
         * Persist the scale choice after release:
         * rolling keeps full output, everything else uses flick scale.
         */
        data->output_mode =
            data->gesture_mode == NURUNURU_GESTURE_ROLLING
                ? NURUNURU_GESTURE_ROLLING
                : NURUNURU_GESTURE_FLICK;

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

            if (horizontal_reversed || vertical_reversed) {
                data->gesture_mode =
                    NURUNURU_GESTURE_UNDECIDED;
                data->gesture_frames = 1;
                data->gesture_peak_speed = speed;
            }

            if (horizontal_reversed) {
                data->rolling_horizontal_fp = 0;
            }

            if (vertical_reversed) {
                data->rolling_vertical_fp = 0;
            }
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

        int32_t target_horizontal_fp = raw_horizontal_fp;
        int32_t target_vertical_fp = raw_vertical_fp;

        if (
            data->gesture_mode ==
            NURUNURU_GESTURE_ROLLING
        ) {
            data->hover_active = false;

            int32_t rolling_scale =
                calculate_rolling_input_scale(
                    data->rolling_frames
                );

            target_horizontal_fp =
                apply_gain(
                    raw_horizontal_fp,
                    rolling_scale
                );

            target_vertical_fp =
                apply_gain(
                    raw_vertical_fp,
                    rolling_scale
                );

            int32_t rolling_horizontal_target_fp =
                calculate_rolling_target_fp(
                    raw_horizontal_fp,
                    data->rolling_frames,
                    get_effective_inertia(config)
                );

            int32_t rolling_vertical_target_fp =
                calculate_rolling_target_fp(
                    raw_vertical_fp,
                    data->rolling_frames,
                    get_effective_inertia(config)
                );

            data->rolling_horizontal_fp =
                smooth_toward(
                    data->rolling_horizontal_fp,
                    rolling_horizontal_target_fp,
                    NURUNURU_ROLLING_RESPONSE
                );

            data->rolling_vertical_fp =
                smooth_toward(
                    data->rolling_vertical_fp,
                    rolling_vertical_target_fp,
                    NURUNURU_ROLLING_RESPONSE
                );

            target_horizontal_fp +=
                data->rolling_horizontal_fp;

            target_vertical_fp +=
                data->rolling_vertical_fp;
        } else {
            /*
             * Undecided and FLICK both use the proven flick engine from the
             * first frame. ROLLING alone takes over later.
             */
            gain_scaled =
                calculate_gain_scaled(
                    speed,
                    max_gain_percent
                );

            target_horizontal_fp =
                raw_to_velocity_fp(
                    apply_gain(
                        frame_horizontal,
                        gain_scaled
                    ),
                    scroll_divisor
                );

            target_vertical_fp =
                raw_to_velocity_fp(
                    apply_gain(
                        frame_vertical,
                        gain_scaled
                    ),
                    scroll_divisor
                );

            if (data->hover_active) {
                target_horizontal_fp +=
                    calculate_hover_force_fp(
                        data->hover_horizontal_fp,
                        speed,
                        config->hover,
                        data->hover_frame
                    );

                target_vertical_fp +=
                    calculate_hover_force_fp(
                        data->hover_vertical_fp,
                        speed,
                        config->hover,
                        data->hover_frame
                    );

                data->hover_frame++;

                if (
                    data->hover_frame >=
                    NURUNURU_HOVER_FRAMES
                ) {
                    data->hover_active = false;
                }
            }

            data->rolling_horizontal_fp =
                smooth_toward(
                    data->rolling_horizontal_fp,
                    0,
                    50
                );

            data->rolling_vertical_fp =
                smooth_toward(
                    data->rolling_vertical_fp,
                    0,
                    50
                );
        }

        /*
         * Current velocity follows the combined input, hover and rolling
         * momentum. Once input ends, this velocity is handed directly to the
         * inertia path below.
         */
        uint8_t velocity_response =
            data->gesture_mode == NURUNURU_GESTURE_ROLLING
                ? NURUNURU_ROLLING_VELOCITY_RESPONSE
                : NURUNURU_FLICK_VELOCITY_RESPONSE;

        int32_t maximum_velocity_change_fp =
            data->gesture_mode == NURUNURU_GESTURE_ROLLING
                ? NURUNURU_ROLLING_MAX_VELOCITY_CHANGE_FP
                : NURUNURU_FLICK_MAX_VELOCITY_CHANGE_FP;

        int32_t smoothed_horizontal_target_fp;
        int32_t smoothed_vertical_target_fp;

        if (
            data->gesture_mode ==
            NURUNURU_GESTURE_ROLLING
        ) {
            /*
             * ROLLING follows increases faster than decreases.
             * Sensor dips and tiny gaps are hidden rather than exposed.
             */
            smoothed_horizontal_target_fp =
                smooth_toward_asymmetric(
                    data->velocity_horizontal_fp,
                    target_horizontal_fp,
                    NURUNURU_ROLLING_ACCEL_RESPONSE,
                    NURUNURU_ROLLING_DECEL_RESPONSE
                );

            smoothed_vertical_target_fp =
                smooth_toward_asymmetric(
                    data->velocity_vertical_fp,
                    target_vertical_fp,
                    NURUNURU_ROLLING_ACCEL_RESPONSE,
                    NURUNURU_ROLLING_DECEL_RESPONSE
                );
        } else {
            smoothed_horizontal_target_fp =
                smooth_toward(
                    data->velocity_horizontal_fp,
                    target_horizontal_fp,
                    velocity_response
                );

            smoothed_vertical_target_fp =
                smooth_toward(
                    data->velocity_vertical_fp,
                    target_vertical_fp,
                    velocity_response
                );
        }

        data->velocity_horizontal_fp =
            limit_velocity_change(
                data->velocity_horizontal_fp,
                smoothed_horizontal_target_fp,
                maximum_velocity_change_fp
            );

        data->velocity_vertical_fp =
            limit_velocity_change(
                data->velocity_vertical_fp,
                smoothed_vertical_target_fp,
                maximum_velocity_change_fp
            );

        if (
            data->gesture_mode ==
            NURUNURU_GESTURE_ROLLING
        ) {
            data->velocity_horizontal_fp =
                enforce_minimum_cruise(
                    data->velocity_horizontal_fp,
                    frame_horizontal,
                    NURUNURU_ROLLING_MIN_CRUISE_FP
                );

            data->velocity_vertical_fp =
                enforce_minimum_cruise(
                    data->velocity_vertical_fp,
                    frame_vertical,
                    NURUNURU_ROLLING_MIN_CRUISE_FP
                );
        }
    } else {
        bool waiting_for_release =
            data->input_was_active &&
            idle_ms < NURUNURU_RELEASE_MS;

        /*
         * Start a short bridge when a ROLLING gesture releases.
         * The bridge holds the last cruise velocity almost unchanged before
         * handing it to the ordinary inertia path.
         */
        if (
            data->input_was_active &&
            data->output_mode ==
                NURUNURU_GESTURE_ROLLING &&
            !data->rolling_bridge_active
        ) {
            data->rolling_bridge_active = true;
            data->rolling_bridge_started_ms = now_ms;
            data->rolling_bridge_horizontal_fp =
                data->velocity_horizontal_fp;
            data->rolling_bridge_vertical_fp =
                data->velocity_vertical_fp;
        }

        bool rolling_bridge_running =
            data->rolling_bridge_active &&
            (
                now_ms -
                data->rolling_bridge_started_ms
            ) < NURUNURU_ROLLING_COAST_BRIDGE_MS;

        if (rolling_bridge_running) {
            data->rolling_bridge_horizontal_fp =
                apply_retention_per_mille(
                    data->rolling_bridge_horizontal_fp,
                    NURUNURU_ROLLING_BRIDGE_RETENTION_PER_MILLE
                );

            data->rolling_bridge_vertical_fp =
                apply_retention_per_mille(
                    data->rolling_bridge_vertical_fp,
                    NURUNURU_ROLLING_BRIDGE_RETENTION_PER_MILLE
                );

            data->velocity_horizontal_fp =
                data->rolling_bridge_horizontal_fp;

            data->velocity_vertical_fp =
                data->rolling_bridge_vertical_fp;
        } else {
            data->rolling_bridge_active = false;

            retention_percent =
                calculate_retention_percent(
                    data->velocity_horizontal_fp,
                    data->velocity_vertical_fp,
                    config
                );

            if (waiting_for_release) {
                retention_percent =
                    (uint8_t)MIN(
                        retention_percent + 2,
                        99
                    );
            }

            bool fast_enough_for_inertia =
                data->last_input_speed >=
                inertia_start_speed;

            if (
                !waiting_for_release &&
                !fast_enough_for_inertia
            ) {
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
        }

        data->rolling_horizontal_fp = 0;
        data->rolling_vertical_fp = 0;
        data->hover_active = false;

        if (idle_ms >= NURUNURU_RELEASE_MS) {
            data->gesture_mode =
                NURUNURU_GESTURE_UNDECIDED;
            data->gesture_frames = 0;
            data->gesture_peak_speed = 0;
            data->rolling_frames = 0;
            data->rolling_horizontal_direction = 0;
            data->rolling_vertical_direction = 0;
        }
    }

    data->input_was_active =
        input_is_active ||
        (
            data->input_was_active &&
            idle_ms < NURUNURU_RELEASE_MS
        );

    /*
     * Mode-specific final screen movement scale.
     *
     * ROLLING: 1 / 1
     * FLICK and its inertia tail: 1 / 3
     *
     * Scaling happens before integer HID extraction, so fractional movement
     * is retained instead of being rounded away.
     */
    int32_t scaled_horizontal_velocity_fp =
        data->velocity_horizontal_fp;

    int32_t scaled_vertical_velocity_fp =
        data->velocity_vertical_fp;

    if (
        data->output_mode !=
        NURUNURU_GESTURE_ROLLING
    ) {
        scaled_horizontal_velocity_fp =
            data->velocity_horizontal_fp / 3;

        scaled_vertical_velocity_fp =
            data->velocity_vertical_fp / 3;
    }

    data->output_horizontal_fp +=
        scaled_horizontal_velocity_fp;

    data->output_vertical_fp +=
        scaled_vertical_velocity_fp;

    output_horizontal =
        extract_scroll_output(
            &data->output_horizontal_fp
        );

    output_vertical =
        extract_scroll_output(
            &data->output_vertical_fp
        );

    if (horizontal_stopped) {
        output_horizontal = 0;
        data->output_horizontal_fp = 0;
    }

    if (vertical_stopped) {
        output_vertical = 0;
        data->output_vertical_fp = 0;
    }

    if (config->invert_horizontal) {
        output_horizontal = -output_horizontal;
    }

    if (config->invert_vertical) {
        output_vertical = -output_vertical;
    }

    bool velocity_is_active =
        data->velocity_horizontal_fp != 0 ||
        data->velocity_vertical_fp != 0;

    bool stop_window_is_active =
        horizontal_stopped ||
        vertical_stopped;

    bool rolling_bridge_is_active =
        data->rolling_bridge_active;

    bool inertia_is_allowed =
        idle_ms < inertia_timeout_ms;

    bool continue_running =
        input_is_active ||
        stop_window_is_active ||
        rolling_bridge_is_active ||
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

        data->rolling_horizontal_fp = 0;
        data->rolling_vertical_fp = 0;

        data->rolling_bridge_active = false;
        data->rolling_bridge_horizontal_fp = 0;
        data->rolling_bridge_vertical_fp = 0;

        data->hover_active = false;
        data->rolling_frames = 0;
        data->rolling_horizontal_direction = 0;
        data->rolling_vertical_direction = 0;
        data->gesture_mode =
            NURUNURU_GESTURE_UNDECIDED;
        data->output_mode =
            NURUNURU_GESTURE_FLICK;
        data->gesture_frames = 0;
        data->gesture_peak_speed = 0;
    }

    input_device = data->input_device;

    LOG_DBG(
        "frame=(%ld,%ld) speed=%ld mode=%u gain=%ld retention=%u hover=%u rolling=(%ld,%ld) stop=(%u,%u) velocity=(%ld,%ld) output=(%d,%d) idle=%u",
        (long)frame_horizontal,
        (long)frame_vertical,
        (long)speed,
        data->gesture_mode,
        (long)gain_scaled,
        retention_percent,
        data->hover_frame,
        (long)data->rolling_horizontal_fp,
        (long)data->rolling_vertical_fp,
        horizontal_stopped,
        vertical_stopped,
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
    data->rolling_horizontal_fp = 0;
    data->rolling_vertical_fp = 0;
    data->rolling_lpf_horizontal_fp = 0;
    data->rolling_lpf_vertical_fp = 0;
    data->cruise_horizontal_fp = 0;
    data->cruise_vertical_fp = 0;

    data->gesture_mode =
        NURUNURU_GESTURE_UNDECIDED;
    data->output_mode =
        NURUNURU_GESTURE_FLICK;

    data->rolling_bridge_active = false;
    data->rolling_bridge_started_ms = 0;
    data->rolling_bridge_horizontal_fp = 0;
    data->rolling_bridge_vertical_fp = 0;

    data->gesture_frames = 0;
    data->gesture_peak_speed = 0;

    data->horizontal_stop_until_ms = 0;
    data->vertical_stop_until_ms = 0;

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