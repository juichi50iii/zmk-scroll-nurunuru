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

/* -------------------------------------------------------------------------- */
/* Fixed-point and timing                                                     */
/* -------------------------------------------------------------------------- */

#define NURUNURU_FP_SCALE 1024
#define NURUNURU_GAIN_SCALE 1000

#define NURUNURU_REPORT_INTERVAL_MS 8
#define NURUNURU_RELEASE_MS 40
#define NURUNURU_REVERSE_STOP_MS 150

#define NURUNURU_FIXED_SCROLL_DIVISOR 24
#define NURUNURU_FIXED_BRAKE 1

/* -------------------------------------------------------------------------- */
/* FLICK engine                                                               */
/* -------------------------------------------------------------------------- */

#define NURUNURU_ACCELERATION_START 1
#define NURUNURU_ACCELERATION_END 10
#define NURUNURU_FLICK_RESPONSE 70
#define NURUNURU_FLICK_MAX_CHANGE_FP 256

#define NURUNURU_HOVER_FRAMES 6
#define NURUNURU_HOVER_STRENGTH_MULTIPLIER 3

/* Final FLICK screen movement = 1/3. */
#define NURUNURU_FLICK_OUTPUT_NUMERATOR 1
#define NURUNURU_FLICK_OUTPUT_DENOMINATOR 5

/* -------------------------------------------------------------------------- */
/* Gesture detection                                                         */
/* -------------------------------------------------------------------------- */

#define NURUNURU_FLICK_SPEED_THRESHOLD 8
#define NURUNURU_ROLL_MAX_SPEED 5
#define NURUNURU_ROLL_MAX_PEAK_SPEED 7
#define NURUNURU_UNDECIDED_TIMEOUT_FRAMES 12

/*
 * ROLLING session detection.
 *
 * Three same-direction low-speed pulses inside 200 ms are treated as one
 * continuous rolling gesture, even if zero-input frames occur between them.
 */
#define NURUNURU_ROLL_SESSION_TIMEOUT_MS 200
#define NURUNURU_ROLL_SESSION_REQUIRED_PULSES 3

static void update_rolling_session(
    struct scroll_nurunuru_data *data,
    int32_t frame_horizontal,
    int32_t frame_vertical,
    int32_t speed,
    uint32_t now_ms
) {
    if (speed <= 0 || speed > NURUNURU_ROLL_MAX_SPEED) {
        data->rolling_session_pulses = 0;
        data->rolling_session_horizontal_direction = 0;
        data->rolling_session_vertical_direction = 0;
        return;
    }

    int8_t horizontal_direction =
        sign_i32(frame_horizontal);

    int8_t vertical_direction =
        sign_i32(frame_vertical);

    bool session_expired =
        (
            now_ms -
            data->rolling_session_last_pulse_ms
        ) > NURUNURU_ROLL_SESSION_TIMEOUT_MS;

    bool horizontal_changed =
        horizontal_direction != 0 &&
        data->rolling_session_horizontal_direction != 0 &&
        horizontal_direction !=
            data->rolling_session_horizontal_direction;

    bool vertical_changed =
        vertical_direction != 0 &&
        data->rolling_session_vertical_direction != 0 &&
        vertical_direction !=
            data->rolling_session_vertical_direction;

    if (
        session_expired ||
        horizontal_changed ||
        vertical_changed
    ) {
        data->rolling_session_pulses = 0;
    }

    if (horizontal_direction != 0) {
        data->rolling_session_horizontal_direction =
            horizontal_direction;
    }

    if (vertical_direction != 0) {
        data->rolling_session_vertical_direction =
            vertical_direction;
    }

    data->rolling_session_last_pulse_ms = now_ms;

    if (data->rolling_session_pulses < UINT8_MAX) {
        data->rolling_session_pulses++;
    }
}

static bool rolling_session_is_ready(
    const struct scroll_nurunuru_data *data,
    int32_t peak_speed
) {
    return data->rolling_session_pulses >=
               NURUNURU_ROLL_SESSION_REQUIRED_PULSES &&
           peak_speed <= NURUNURU_ROLL_MAX_PEAK_SPEED;
}

/* -------------------------------------------------------------------------- */
/* ROLLING engine                                                             */
/* -------------------------------------------------------------------------- */

/*
 * One brief zero-input gap is common at low speed. Keep the last stable
 * ROLLING target for 48 ms instead of exposing that gap to the screen.
 */
#define NURUNURU_ROLLING_GAP_HOLD_MS 200

/*
 * Input increases should remain responsive; decreases are hidden more
 * strongly so the screen does not pulse with sensor chatter.
 */
#define NURUNURU_ROLLING_ACCEL_RESPONSE 18
#define NURUNURU_ROLLING_DECEL_RESPONSE 2

/*
 * Two smoothing stages:
 *   raw -> filtered input -> cruise target -> output velocity
 */
#define NURUNURU_ROLLING_INPUT_LPF_RESPONSE 16
#define NURUNURU_ROLLING_CRUISE_RESPONSE 10
#define NURUNURU_ROLLING_MAX_CHANGE_FP 48

/*
 * Only a tiny anti-zero safeguard. The real continuity comes from gap hold,
 * asymmetric smoothing and bridge handling.
 */
#define NURUNURU_ROLLING_MIN_CRUISE_FP 24

/*
 * ROLLING starts slightly heavy and gradually reaches normal speed.
 * 38 frames * 8 ms ~= 304 ms.
 */
#define NURUNURU_ROLLING_BUILD_FRAMES 38
#define NURUNURU_ROLLING_START_SCALE 600
#define NURUNURU_ROLLING_END_SCALE 1000

/* -------------------------------------------------------------------------- */
/* ROLLING release bridge                                                     */
/* -------------------------------------------------------------------------- */

#define NURUNURU_ROLLING_BRIDGE_MIN_MS 300
#define NURUNURU_ROLLING_BRIDGE_MAX_MS 1000
#define NURUNURU_ROLLING_BRIDGE_FULL_SPEED_FP \
    (4 * NURUNURU_FP_SCALE)

/*
 * 0.997 per 8 ms is a gentle decline. The ordinary inertia curve takes over
 * after the bridge duration.
 */
#define NURUNURU_ROLLING_BRIDGE_RETENTION_PER_MILLE 997

enum scroll_nurunuru_mode {
    NURUNURU_MODE_UNDECIDED = 0,
    NURUNURU_MODE_FLICK,
    NURUNURU_MODE_ROLLING,
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

    struct k_mutex lock;
    struct k_work_delayable work;
    bool worker_running;

    int32_t pending_horizontal;
    int32_t pending_vertical;

    int32_t velocity_horizontal_fp;
    int32_t velocity_vertical_fp;

    int32_t output_horizontal_fp;
    int32_t output_vertical_fp;

    /* Gesture classification. */
    enum scroll_nurunuru_mode mode;
    enum scroll_nurunuru_mode output_mode;
    uint8_t gesture_frames;
    int32_t gesture_peak_speed;

    /* FLICK hover. */
    bool hover_active;
    uint8_t hover_frame;
    int32_t hover_horizontal_fp;
    int32_t hover_vertical_fp;

    /* ROLLING continuity/filter state. */
    uint8_t rolling_frames;
    int8_t rolling_horizontal_direction;
    int8_t rolling_vertical_direction;

    int32_t rolling_filtered_horizontal_fp;
    int32_t rolling_filtered_vertical_fp;

    int32_t rolling_cruise_horizontal_fp;
    int32_t rolling_cruise_vertical_fp;

    int32_t rolling_hold_horizontal_fp;
    int32_t rolling_hold_vertical_fp;
    uint32_t rolling_last_nonzero_ms;

    /*
     * Same-direction pulse session detector for ultra-low-speed rolling.
     */
    uint8_t rolling_session_pulses;
    int8_t rolling_session_horizontal_direction;
    int8_t rolling_session_vertical_direction;
    uint32_t rolling_session_last_pulse_ms;

    /* ROLLING release bridge. */
    bool rolling_bridge_active;
    uint32_t rolling_bridge_started_ms;
    uint16_t rolling_bridge_duration_ms;
    int32_t rolling_bridge_horizontal_fp;
    int32_t rolling_bridge_vertical_fp;

    /* Reverse-input stop windows. */
    uint32_t horizontal_stop_until_ms;
    uint32_t vertical_stop_until_ms;

    int32_t last_input_speed;
    bool input_was_active;
    uint32_t last_input_ms;
};

/* -------------------------------------------------------------------------- */
/* Basic helpers                                                              */
/* -------------------------------------------------------------------------- */

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

static int32_t limit_change(
    int32_t current,
    int32_t target,
    int32_t maximum_change
) {
    if (maximum_change <= 0) {
        return target;
    }

    int64_t difference =
        (int64_t)target - current;

    difference = CLAMP(
        difference,
        -(int64_t)maximum_change,
        (int64_t)maximum_change
    );

    return current + (int32_t)difference;
}

static int32_t smoothstep_scaled(int32_t value_scaled) {
    value_scaled = CLAMP(
        value_scaled,
        0,
        NURUNURU_GAIN_SCALE
    );

    int64_t squared =
        ((int64_t)value_scaled * value_scaled) /
        NURUNURU_GAIN_SCALE;

    return (int32_t)(
        (squared *
         ((3 * NURUNURU_GAIN_SCALE) -
          (2 * value_scaled))) /
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

static int32_t raw_to_velocity_fp(int32_t raw_value) {
    int64_t result =
        ((int64_t)raw_value * NURUNURU_FP_SCALE) /
        NURUNURU_FIXED_SCROLL_DIVISOR;

    return (int32_t)CLAMP(
        result,
        (int64_t)INT32_MIN,
        (int64_t)INT32_MAX
    );
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

/* -------------------------------------------------------------------------- */
/* Public tuning mapping                                                      */
/* -------------------------------------------------------------------------- */

static uint16_t get_max_gain_percent(
    const struct scroll_nurunuru_config *config
) {
    return (uint16_t)clamp_tuning(config->acceleration) * 100U;
}

/*
 * inertia 5 is intentionally the old maximum:
 *
 * 1 -> 2
 * 2 -> 4
 * 3 -> 6
 * 4 -> 8
 * 5..10 -> 10
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

/* -------------------------------------------------------------------------- */
/* FLICK engine                                                               */
/* -------------------------------------------------------------------------- */

static int32_t calculate_acceleration_gain(
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

    int32_t position =
        speed - NURUNURU_ACCELERATION_START;

    int32_t range =
        NURUNURU_ACCELERATION_END -
        NURUNURU_ACCELERATION_START;

    int32_t progress =
        (int32_t)(
            ((int64_t)position *
             NURUNURU_GAIN_SCALE) /
            range
        );

    int32_t curve =
        smoothstep_scaled(progress);

    return NURUNURU_GAIN_SCALE +
           (int32_t)(
               ((int64_t)(
                    maximum_gain -
                    NURUNURU_GAIN_SCALE
                ) * curve) /
               NURUNURU_GAIN_SCALE
           );
}

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

    int32_t time_progress =
        (int32_t)(
            ((int64_t)hover_frame *
             NURUNURU_GAIN_SCALE) /
            NURUNURU_HOVER_FRAMES
        );

    int32_t time_factor =
        NURUNURU_GAIN_SCALE -
        smoothstep_scaled(time_progress);

    int32_t speed_position =
        CLAMP(input_speed - 1, 0, 9);

    int32_t speed_progress =
        (speed_position *
         NURUNURU_GAIN_SCALE) /
        9;

    int32_t low_speed_factor =
        NURUNURU_GAIN_SCALE -
        smoothstep_scaled(speed_progress);

    int64_t force =
        (int64_t)first_input_fp *
        hover *
        time_factor *
        low_speed_factor *
        NURUNURU_HOVER_STRENGTH_MULTIPLIER;

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

/* -------------------------------------------------------------------------- */
/* Gesture detector                                                           */
/* -------------------------------------------------------------------------- */

static enum scroll_nurunuru_mode classify_gesture(
    enum scroll_nurunuru_mode current_mode,
    uint8_t gesture_frames,
    int32_t current_speed,
    int32_t peak_speed
) {
    if (current_mode != NURUNURU_MODE_UNDECIDED) {
        return current_mode;
    }

    if (current_speed >= NURUNURU_FLICK_SPEED_THRESHOLD) {
        return NURUNURU_MODE_FLICK;
    }

    if (
        gesture_frames >=
        NURUNURU_UNDECIDED_TIMEOUT_FRAMES
    ) {
        return NURUNURU_MODE_FLICK;
    }

    ARG_UNUSED(peak_speed);

    return NURUNURU_MODE_UNDECIDED;
}

/* -------------------------------------------------------------------------- */
/* ROLLING engine                                                             */
/* -------------------------------------------------------------------------- */

static int32_t calculate_rolling_scale(
    uint8_t rolling_frames
) {
    uint8_t frames =
        MIN(
            rolling_frames,
            NURUNURU_ROLLING_BUILD_FRAMES
        );

    int32_t progress =
        (int32_t)(
            ((int64_t)frames *
             NURUNURU_GAIN_SCALE) /
            NURUNURU_ROLLING_BUILD_FRAMES
        );

    int32_t curve =
        smoothstep_scaled(progress);

    return NURUNURU_ROLLING_START_SCALE +
           (int32_t)(
               ((int64_t)(
                    NURUNURU_ROLLING_END_SCALE -
                    NURUNURU_ROLLING_START_SCALE
                ) * curve) /
               NURUNURU_GAIN_SCALE
           );
}

static int32_t enforce_minimum_cruise(
    int32_t velocity_fp,
    int8_t direction
) {
    if (direction == 0) {
        return velocity_fp;
    }

    if (
        abs_i32(velocity_fp) >=
        NURUNURU_ROLLING_MIN_CRUISE_FP
    ) {
        return velocity_fp;
    }

    return (int32_t)direction *
           NURUNURU_ROLLING_MIN_CRUISE_FP;
}

/* -------------------------------------------------------------------------- */
/* Inertia and bridge                                                         */
/* -------------------------------------------------------------------------- */

static uint8_t calculate_inertia_retention(
    int32_t horizontal_velocity_fp,
    int32_t vertical_velocity_fp,
    const struct scroll_nurunuru_config *config
) {
    uint8_t inertia =
        get_effective_inertia(config);

    uint8_t fast_retention =
        (uint8_t)CLAMP(
            90 + inertia,
            1,
            99
        );

    int32_t slow_value =
        (int32_t)fast_retention -
        (NURUNURU_FIXED_BRAKE * 2);

    uint8_t slow_retention =
        (uint8_t)CLAMP(
            slow_value,
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

    if (speed_fp <= 0) {
        return slow_retention;
    }

    if (speed_fp >= transition_end_fp) {
        return fast_retention;
    }

    int32_t progress =
        (int32_t)(
            ((int64_t)speed_fp *
             NURUNURU_GAIN_SCALE) /
            transition_end_fp
        );

    int32_t curve =
        smoothstep_scaled(progress);

    return (uint8_t)CLAMP(
        (int32_t)slow_retention +
        (int32_t)(
            ((int64_t)(
                fast_retention -
                slow_retention
            ) * curve) /
            NURUNURU_GAIN_SCALE
        ),
        0,
        99
    );
}

static int32_t apply_retention_percent(
    int32_t value,
    uint8_t retention_percent
) {
    retention_percent =
        CLAMP(retention_percent, 0, 99);

    int64_t result =
        ((int64_t)value *
         retention_percent) /
        100;

    if (abs_i32((int32_t)result) < 8) {
        return 0;
    }

    return (int32_t)result;
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

static uint16_t calculate_bridge_duration_ms(
    int32_t horizontal_velocity_fp,
    int32_t vertical_velocity_fp
) {
    int32_t speed_fp =
        max_i32(
            abs_i32(horizontal_velocity_fp),
            abs_i32(vertical_velocity_fp)
        );

    speed_fp =
        CLAMP(
            speed_fp,
            0,
            NURUNURU_ROLLING_BRIDGE_FULL_SPEED_FP
        );

    int32_t progress =
        (int32_t)(
            ((int64_t)speed_fp *
             NURUNURU_GAIN_SCALE) /
            NURUNURU_ROLLING_BRIDGE_FULL_SPEED_FP
        );

    int32_t curve =
        smoothstep_scaled(progress);

    return (uint16_t)(
        NURUNURU_ROLLING_BRIDGE_MIN_MS +
        (uint32_t)(
            ((int64_t)(
                NURUNURU_ROLLING_BRIDGE_MAX_MS -
                NURUNURU_ROLLING_BRIDGE_MIN_MS
            ) * curve) /
            NURUNURU_GAIN_SCALE
        )
    );
}

/* -------------------------------------------------------------------------- */
/* Output                                                                     */
/* -------------------------------------------------------------------------- */

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
                "Failed horizontal scroll report: %d",
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
                "Failed vertical scroll report: %d",
                ret
            );
        }
    }
}

/* -------------------------------------------------------------------------- */
/* State reset helpers                                                        */
/* -------------------------------------------------------------------------- */

static void reset_gesture_state(
    struct scroll_nurunuru_data *data
) {
    data->mode = NURUNURU_MODE_UNDECIDED;
    data->gesture_frames = 0;
    data->gesture_peak_speed = 0;

    data->hover_active = false;
    data->hover_frame = 0;

    data->rolling_frames = 0;
    data->rolling_horizontal_direction = 0;
    data->rolling_vertical_direction = 0;

    data->rolling_filtered_horizontal_fp = 0;
    data->rolling_filtered_vertical_fp = 0;

    data->rolling_cruise_horizontal_fp = 0;
    data->rolling_cruise_vertical_fp = 0;

    data->rolling_hold_horizontal_fp = 0;
    data->rolling_hold_vertical_fp = 0;
    data->rolling_last_nonzero_ms = 0;

    data->rolling_session_pulses = 0;
    data->rolling_session_horizontal_direction = 0;
    data->rolling_session_vertical_direction = 0;
    data->rolling_session_last_pulse_ms = 0;
}

static void reset_bridge_state(
    struct scroll_nurunuru_data *data
) {
    data->rolling_bridge_active = false;
    data->rolling_bridge_started_ms = 0;
    data->rolling_bridge_duration_ms = 0;
    data->rolling_bridge_horizontal_fp = 0;
    data->rolling_bridge_vertical_fp = 0;
}

/* -------------------------------------------------------------------------- */
/* Main worker                                                                */
/* -------------------------------------------------------------------------- */

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

    k_mutex_lock(&data->lock, K_FOREVER);

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
     * Opposite input immediately stops the corresponding axis. The triggering
     * input is consumed, and further input on that axis is ignored for 150 ms.
     */
    if (
        !horizontal_stopped &&
        directions_are_opposite(
            frame_horizontal,
            data->velocity_horizontal_fp
        )
    ) {
        data->velocity_horizontal_fp = 0;
        data->output_horizontal_fp = 0;
        data->rolling_hold_horizontal_fp = 0;
        data->rolling_filtered_horizontal_fp = 0;
        data->rolling_cruise_horizontal_fp = 0;
        data->rolling_bridge_horizontal_fp = 0;

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
        data->output_vertical_fp = 0;
        data->rolling_hold_vertical_fp = 0;
        data->rolling_filtered_vertical_fp = 0;
        data->rolling_cruise_vertical_fp = 0;
        data->rolling_bridge_vertical_fp = 0;

        data->vertical_stop_until_ms =
            now_ms + NURUNURU_REVERSE_STOP_MS;

        frame_vertical = 0;
        vertical_stopped = true;
    }

    if (horizontal_stopped) {
        frame_horizontal = 0;
        data->velocity_horizontal_fp = 0;
        data->output_horizontal_fp = 0;
    }

    if (vertical_stopped) {
        frame_vertical = 0;
        data->velocity_vertical_fp = 0;
        data->output_vertical_fp = 0;
    }

    bool raw_input_is_active =
        frame_horizontal != 0 ||
        frame_vertical != 0;

    /*
     * Brief zero frames remain part of a ROLLING gesture.
     */
    bool rolling_gap_hold_active =
        !raw_input_is_active &&
        data->output_mode == NURUNURU_MODE_ROLLING &&
        (
            now_ms -
            data->rolling_last_nonzero_ms
        ) < NURUNURU_ROLLING_GAP_HOLD_MS;

    bool input_is_active =
        raw_input_is_active ||
        rolling_gap_hold_active;

    bool input_just_started =
        input_is_active &&
        !data->input_was_active;

    int32_t speed =
        max_i32(
            abs_i32(frame_horizontal),
            abs_i32(frame_vertical)
        );

    int32_t gain_scaled =
        NURUNURU_GAIN_SCALE;

    uint8_t retention_percent = 0;

    if (input_is_active) {
        reset_bridge_state(data);

        if (raw_input_is_active) {
            data->last_input_speed = speed;
            data->rolling_last_nonzero_ms = now_ms;

            if (input_just_started) {
                data->mode =
                    NURUNURU_MODE_UNDECIDED;

                data->gesture_frames = 1;
                data->gesture_peak_speed = speed;

                data->hover_horizontal_fp =
                    raw_to_velocity_fp(frame_horizontal);

                data->hover_vertical_fp =
                    raw_to_velocity_fp(frame_vertical);

                data->hover_frame = 0;
                data->hover_active = true;
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

            update_rolling_session(
                data,
                frame_horizontal,
                frame_vertical,
                speed,
                now_ms
            );

            data->mode =
                classify_gesture(
                    data->mode,
                    data->gesture_frames,
                    speed,
                    data->gesture_peak_speed
                );

            if (
                data->mode ==
                    NURUNURU_MODE_UNDECIDED &&
                rolling_session_is_ready(
                    data,
                    data->gesture_peak_speed
                )
            ) {
                data->mode =
                    NURUNURU_MODE_ROLLING;
            }

            data->output_mode =
                data->mode == NURUNURU_MODE_ROLLING
                    ? NURUNURU_MODE_ROLLING
                    : NURUNURU_MODE_FLICK;
        }

        int32_t raw_horizontal_fp =
            raw_to_velocity_fp(frame_horizontal);

        int32_t raw_vertical_fp =
            raw_to_velocity_fp(frame_vertical);

        int32_t target_horizontal_fp = 0;
        int32_t target_vertical_fp = 0;

        if (data->mode == NURUNURU_MODE_ROLLING) {
            data->hover_active = false;

            if (raw_input_is_active) {
                int8_t horizontal_direction =
                    sign_i32(frame_horizontal);

                int8_t vertical_direction =
                    sign_i32(frame_vertical);

                if (
                    horizontal_direction != 0 &&
                    data->rolling_horizontal_direction != 0 &&
                    horizontal_direction !=
                        data->rolling_horizontal_direction
                ) {
                    data->rolling_frames = 0;
                    data->rolling_filtered_horizontal_fp = 0;
                    data->rolling_cruise_horizontal_fp = 0;
                }

                if (
                    vertical_direction != 0 &&
                    data->rolling_vertical_direction != 0 &&
                    vertical_direction !=
                        data->rolling_vertical_direction
                ) {
                    data->rolling_frames = 0;
                    data->rolling_filtered_vertical_fp = 0;
                    data->rolling_cruise_vertical_fp = 0;
                }

                if (horizontal_direction != 0) {
                    data->rolling_horizontal_direction =
                        horizontal_direction;
                }

                if (vertical_direction != 0) {
                    data->rolling_vertical_direction =
                        vertical_direction;
                }

                if (data->rolling_frames < UINT8_MAX) {
                    data->rolling_frames++;
                }

                data->rolling_filtered_horizontal_fp =
                    smooth_toward_asymmetric(
                        data->rolling_filtered_horizontal_fp,
                        raw_horizontal_fp,
                        NURUNURU_ROLLING_INPUT_LPF_RESPONSE,
                        NURUNURU_ROLLING_DECEL_RESPONSE
                    );

                data->rolling_filtered_vertical_fp =
                    smooth_toward_asymmetric(
                        data->rolling_filtered_vertical_fp,
                        raw_vertical_fp,
                        NURUNURU_ROLLING_INPUT_LPF_RESPONSE,
                        NURUNURU_ROLLING_DECEL_RESPONSE
                    );

                int32_t rolling_scale =
                    calculate_rolling_scale(
                        data->rolling_frames
                    );

                data->rolling_hold_horizontal_fp =
                    apply_gain(
                        data->rolling_filtered_horizontal_fp,
                        rolling_scale
                    );

                data->rolling_hold_vertical_fp =
                    apply_gain(
                        data->rolling_filtered_vertical_fp,
                        rolling_scale
                    );
            }

            /*
             * During a short sensor gap, these hold values remain unchanged.
             */
            data->rolling_cruise_horizontal_fp =
                smooth_toward_asymmetric(
                    data->rolling_cruise_horizontal_fp,
                    data->rolling_hold_horizontal_fp,
                    NURUNURU_ROLLING_ACCEL_RESPONSE,
                    NURUNURU_ROLLING_DECEL_RESPONSE
                );

            data->rolling_cruise_vertical_fp =
                smooth_toward_asymmetric(
                    data->rolling_cruise_vertical_fp,
                    data->rolling_hold_vertical_fp,
                    NURUNURU_ROLLING_ACCEL_RESPONSE,
                    NURUNURU_ROLLING_DECEL_RESPONSE
                );

            target_horizontal_fp =
                enforce_minimum_cruise(
                    data->rolling_cruise_horizontal_fp,
                    data->rolling_horizontal_direction
                );

            target_vertical_fp =
                enforce_minimum_cruise(
                    data->rolling_cruise_vertical_fp,
                    data->rolling_vertical_direction
                );

            int32_t smoothed_horizontal =
                smooth_toward_asymmetric(
                    data->velocity_horizontal_fp,
                    target_horizontal_fp,
                    NURUNURU_ROLLING_ACCEL_RESPONSE,
                    NURUNURU_ROLLING_DECEL_RESPONSE
                );

            int32_t smoothed_vertical =
                smooth_toward_asymmetric(
                    data->velocity_vertical_fp,
                    target_vertical_fp,
                    NURUNURU_ROLLING_ACCEL_RESPONSE,
                    NURUNURU_ROLLING_DECEL_RESPONSE
                );

            data->velocity_horizontal_fp =
                limit_change(
                    data->velocity_horizontal_fp,
                    smoothed_horizontal,
                    NURUNURU_ROLLING_MAX_CHANGE_FP
                );

            data->velocity_vertical_fp =
                limit_change(
                    data->velocity_vertical_fp,
                    smoothed_vertical,
                    NURUNURU_ROLLING_MAX_CHANGE_FP
                );
        } else {
            /*
             * UNDECIDED uses FLICK processing from frame one. If the gesture
             * later becomes ROLLING, the rolling engine takes over smoothly.
             */
            gain_scaled =
                calculate_acceleration_gain(
                    speed,
                    get_max_gain_percent(config)
                );

            target_horizontal_fp =
                raw_to_velocity_fp(
                    apply_gain(
                        frame_horizontal,
                        gain_scaled
                    )
                );

            target_vertical_fp =
                raw_to_velocity_fp(
                    apply_gain(
                        frame_vertical,
                        gain_scaled
                    )
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

            int32_t smoothed_horizontal =
                smooth_toward(
                    data->velocity_horizontal_fp,
                    target_horizontal_fp,
                    NURUNURU_FLICK_RESPONSE
                );

            int32_t smoothed_vertical =
                smooth_toward(
                    data->velocity_vertical_fp,
                    target_vertical_fp,
                    NURUNURU_FLICK_RESPONSE
                );

            data->velocity_horizontal_fp =
                limit_change(
                    data->velocity_horizontal_fp,
                    smoothed_horizontal,
                    NURUNURU_FLICK_MAX_CHANGE_FP
                );

            data->velocity_vertical_fp =
                limit_change(
                    data->velocity_vertical_fp,
                    smoothed_vertical,
                    NURUNURU_FLICK_MAX_CHANGE_FP
                );
        }
    } else {
        bool waiting_for_release =
            data->input_was_active &&
            idle_ms < NURUNURU_RELEASE_MS;

        /*
         * Start a ROLLING bridge once the gap-hold window has really expired.
         */
        if (
            data->input_was_active &&
            data->output_mode ==
                NURUNURU_MODE_ROLLING &&
            !data->rolling_bridge_active
        ) {
            data->rolling_bridge_active = true;
            data->rolling_bridge_started_ms = now_ms;

            data->rolling_bridge_duration_ms =
                calculate_bridge_duration_ms(
                    data->velocity_horizontal_fp,
                    data->velocity_vertical_fp
                );

            data->rolling_bridge_horizontal_fp =
                data->velocity_horizontal_fp;

            data->rolling_bridge_vertical_fp =
                data->velocity_vertical_fp;
        }

        bool bridge_running =
            data->rolling_bridge_active &&
            (
                now_ms -
                data->rolling_bridge_started_ms
            ) < data->rolling_bridge_duration_ms;

        if (bridge_running) {
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
                calculate_inertia_retention(
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
                get_inertia_start_speed(config);

            if (
                !waiting_for_release &&
                !fast_enough_for_inertia
            ) {
                data->velocity_horizontal_fp = 0;
                data->velocity_vertical_fp = 0;
            } else {
                data->velocity_horizontal_fp =
                    apply_retention_percent(
                        data->velocity_horizontal_fp,
                        retention_percent
                    );

                data->velocity_vertical_fp =
                    apply_retention_percent(
                        data->velocity_vertical_fp,
                        retention_percent
                    );
            }
        }

        data->hover_active = false;

        if (idle_ms >= NURUNURU_RELEASE_MS) {
            data->mode =
                NURUNURU_MODE_UNDECIDED;

            data->gesture_frames = 0;
            data->gesture_peak_speed = 0;
        }

        if (
            (
                now_ms -
                data->rolling_session_last_pulse_ms
            ) > NURUNURU_ROLL_SESSION_TIMEOUT_MS
        ) {
            data->rolling_session_pulses = 0;
            data->rolling_session_horizontal_direction = 0;
            data->rolling_session_vertical_direction = 0;
        }
    }

    data->input_was_active =
        input_is_active ||
        (
            data->input_was_active &&
            idle_ms < NURUNURU_RELEASE_MS
        );

    /*
     * FLICK is reduced to 1/3. ROLLING remains full scale.
     */
    int32_t scaled_horizontal_fp =
        data->velocity_horizontal_fp;

    int32_t scaled_vertical_fp =
        data->velocity_vertical_fp;

    if (data->output_mode != NURUNURU_MODE_ROLLING) {
        scaled_horizontal_fp =
            (int32_t)(
                ((int64_t)data->velocity_horizontal_fp *
                 NURUNURU_FLICK_OUTPUT_NUMERATOR) /
                NURUNURU_FLICK_OUTPUT_DENOMINATOR
            );

        scaled_vertical_fp =
            (int32_t)(
                ((int64_t)data->velocity_vertical_fp *
                 NURUNURU_FLICK_OUTPUT_NUMERATOR) /
                NURUNURU_FLICK_OUTPUT_DENOMINATOR
            );
    }

    data->output_horizontal_fp +=
        scaled_horizontal_fp;

    data->output_vertical_fp +=
        scaled_vertical_fp;

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

    bool inertia_is_allowed =
        idle_ms < get_inertia_timeout_ms(config);

    bool continue_running =
        input_is_active ||
        stop_window_is_active ||
        data->rolling_bridge_active ||
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

        reset_gesture_state(data);
        reset_bridge_state(data);

        data->output_mode = NURUNURU_MODE_FLICK;
    }

    input_device = data->input_device;

    LOG_DBG(
        "raw=(%ld,%ld) mode=%u session=%u gap=%u speed=%ld gain=%ld bridge=%u velocity=(%ld,%ld) output=(%d,%d)",
        (long)frame_horizontal,
        (long)frame_vertical,
        data->mode,
        data->rolling_session_pulses,
        rolling_gap_hold_active,
        (long)speed,
        (long)gain_scaled,
        data->rolling_bridge_duration_ms,
        (long)data->velocity_horizontal_fp,
        (long)data->velocity_vertical_fp,
        output_horizontal,
        output_vertical
    );

    k_mutex_unlock(&data->lock);

    send_scroll_events(
        input_device,
        output_horizontal,
        output_vertical
    );
}

/* -------------------------------------------------------------------------- */
/* Input processor API                                                        */
/* -------------------------------------------------------------------------- */

static int scroll_nurunuru_handle_event(
    const struct device *dev,
    struct input_event *event,
    uint32_t param1,
    uint32_t param2,
    struct zmk_input_processor_state *processor_state
) {
    struct scroll_nurunuru_data *data =
        dev->data;

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
     * Consume the original pointer event so cursor movement cannot leak
     * through while this processor emits wheel events asynchronously.
     */
    event->code =
        original_code == INPUT_REL_X
            ? INPUT_REL_WHEEL
            : INPUT_REL_HWHEEL;

    event->value = 0;

    if (original_value == 0) {
        return ZMK_INPUT_PROC_STOP;
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    data->input_device =
        event->dev;

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
            K_MSEC(NURUNURU_REPORT_INTERVAL_MS)
        );
    }

    k_mutex_unlock(&data->lock);

    return ZMK_INPUT_PROC_STOP;
}

static int scroll_nurunuru_init(
    const struct device *dev
) {
    struct scroll_nurunuru_data *data =
        dev->data;

    data->dev = dev;
    data->input_device = NULL;

    data->worker_running = false;

    data->pending_horizontal = 0;
    data->pending_vertical = 0;

    data->velocity_horizontal_fp = 0;
    data->velocity_vertical_fp = 0;

    data->output_horizontal_fp = 0;
    data->output_vertical_fp = 0;

    data->mode = NURUNURU_MODE_UNDECIDED;
    data->output_mode = NURUNURU_MODE_FLICK;

    data->gesture_frames = 0;
    data->gesture_peak_speed = 0;

    data->hover_active = false;
    data->hover_frame = 0;
    data->hover_horizontal_fp = 0;
    data->hover_vertical_fp = 0;

    data->rolling_frames = 0;
    data->rolling_horizontal_direction = 0;
    data->rolling_vertical_direction = 0;

    data->rolling_filtered_horizontal_fp = 0;
    data->rolling_filtered_vertical_fp = 0;

    data->rolling_cruise_horizontal_fp = 0;
    data->rolling_cruise_vertical_fp = 0;

    data->rolling_hold_horizontal_fp = 0;
    data->rolling_hold_vertical_fp = 0;
    data->rolling_last_nonzero_ms = 0;

    data->rolling_session_pulses = 0;
    data->rolling_session_horizontal_direction = 0;
    data->rolling_session_vertical_direction = 0;
    data->rolling_session_last_pulse_ms = 0;

    reset_bridge_state(data);

    data->horizontal_stop_until_ms = 0;
    data->vertical_stop_until_ms = 0;

    data->last_input_speed = 0;
    data->input_was_active = false;
    data->last_input_ms = 0;

    k_mutex_init(&data->lock);

    k_work_init_delayable(
        &data->work,
        scroll_nurunuru_work_callback
    );

    LOG_INF("zmk-scroll-nurunuru v2 initialized");

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
            .scroll_scale =                                           \
                DT_INST_PROP_OR(inst, scroll_scale, 1),               \
                                                                       \
            .acceleration =                                           \
                DT_INST_PROP_OR(inst, acceleration, 4),               \
                                                                       \
            .hover =                                                  \
                DT_INST_PROP_OR(inst, hover, 5),                      \
                                                                       \
            .inertia =                                                \
                DT_INST_PROP_OR(inst, inertia, 5),                    \
                                                                       \
            .brake =                                                  \
                DT_INST_PROP_OR(inst, brake, 1),                      \
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