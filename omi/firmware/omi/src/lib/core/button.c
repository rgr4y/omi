#include "button.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/l2cap.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/util.h>
#include <string.h>

#include "haptic.h"
#include "imu.h"
#include "led.h"
#include "mic.h"
#include "speaker.h"
#include "transport.h"
#include "wdog_facade.h"
#ifdef CONFIG_OMI_ENABLE_OFFLINE_STORAGE
#include "sd_card.h"
#endif

LOG_MODULE_REGISTER(button, CONFIG_LOG_DEFAULT_LEVEL);

extern bool is_off;

static void button_ccc_config_changed_handler(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t button_data_read_characteristic(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               void *buf,
                                               uint16_t len,
                                               uint16_t offset);

static struct bt_uuid_128 button_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x23BA7924, 0x0000, 0x1000, 0x7450, 0x346EAC492E92));
static struct bt_uuid_128 button_characteristic_data_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x23BA7925, 0x0000, 0x1000, 0x7450, 0x346EAC492E92));

static struct bt_gatt_attr button_service_attr[] = {
    BT_GATT_PRIMARY_SERVICE(&button_uuid),
    BT_GATT_CHARACTERISTIC(&button_characteristic_data_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           button_data_read_characteristic,
                           NULL,
                           NULL),
    BT_GATT_CCC(button_ccc_config_changed_handler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

static struct bt_gatt_service button_service = BT_GATT_SERVICE(button_service_attr);

static void button_ccc_config_changed_handler(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (value == BT_GATT_CCC_NOTIFY) {
        LOG_INF("Client subscribed for notifications");
    } else if (value == 0) {
        LOG_INF("Client unsubscribed from notifications");
    } else {
        LOG_ERR("Invalid CCC value: %u", value);
    }
}
static const struct device *const buttons = DEVICE_DT_GET(DT_ALIAS(buttons));
static const struct gpio_dt_spec usr_btn = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(usr_btn), gpios, {0});

static bool was_pressed = false;

// Using GPIO callback due to the lower priority of the input subsystem vs. storage.c's thread that prevents the
// callback from working properly.
#define BUTTON_CHECK_INTERVAL_MS 40U

#define DOUBLE_TAP 2
#define BUTTON_RELEASE 5
#define BOOKMARK_TAP 6
#define PAUSE_TOGGLE 7

// Tweakables for the custom interaction model.
#define TAP_THRESHOLD_MS 300U
#define DOUBLE_TAP_WINDOW_MS 600U
#define BOOKMARK_DEBOUNCE_WINDOW_MS (30U * 1000U)
#define PAUSE_HOLD_THRESHOLD_MS 1000U
#define POWEROFF_HEARTBEAT_START_MS 2500U
#define POWEROFF_HOLD_MS 5000U

#define HAPTIC_MARK_MS 100U
#define HAPTIC_NOPE_PULSE_MS 60U
#define HAPTIC_NOPE_GAP_MS 90U
#define HAPTIC_ARMED_MS 50U
#define HAPTIC_RESUMED_PULSE_MS 70U
#define HAPTIC_RESUMED_GAP_MS 60U
#define HAPTIC_PAUSED_LONG_MS 400U
#define HAPTIC_PAUSED_GAP_MS 300U
#define HAPTIC_PAUSED_SHORT_MS 150U
#define HAPTIC_HEARTBEAT_FINAL_MS 700U
#define HAPTIC_SHUTDOWN_DEFAULT_MS 100U
#define HAPTIC_SHUTDOWN_SETTLE_MS 200U

#define BUTTON_PRESSED 1
#define BUTTON_RELEASED 0

static const uint16_t heartbeat_start_ms[] = {2500U, 3300U, 3950U, 4500U};
static const uint16_t heartbeat_pulse_ms[] = {80U, 120U, 170U, 230U};

struct haptic_pattern {
    const uint16_t *pulse_ms;
    const uint16_t *gap_ms;
    uint8_t pulse_count;
};

static const uint16_t haptic_nope_pulses[] = {HAPTIC_NOPE_PULSE_MS, HAPTIC_NOPE_PULSE_MS};
static const uint16_t haptic_nope_gaps[] = {HAPTIC_NOPE_GAP_MS};
static const struct haptic_pattern haptic_pattern_nope = {
    .pulse_ms = haptic_nope_pulses,
    .gap_ms = haptic_nope_gaps,
    .pulse_count = ARRAY_SIZE(haptic_nope_pulses),
};

static const uint16_t haptic_resumed_pulses[] =
    {HAPTIC_RESUMED_PULSE_MS, HAPTIC_RESUMED_PULSE_MS, HAPTIC_RESUMED_PULSE_MS};
static const uint16_t haptic_resumed_gaps[] = {HAPTIC_RESUMED_GAP_MS, HAPTIC_RESUMED_GAP_MS};
static const struct haptic_pattern haptic_pattern_resumed = {
    .pulse_ms = haptic_resumed_pulses,
    .gap_ms = haptic_resumed_gaps,
    .pulse_count = ARRAY_SIZE(haptic_resumed_pulses),
};

static const uint16_t haptic_paused_pulses[] = {HAPTIC_PAUSED_LONG_MS, HAPTIC_PAUSED_SHORT_MS};
static const uint16_t haptic_paused_gaps[] = {HAPTIC_PAUSED_GAP_MS};
static const struct haptic_pattern haptic_pattern_paused = {
    .pulse_ms = haptic_paused_pulses,
    .gap_ms = haptic_paused_gaps,
    .pulse_count = ARRAY_SIZE(haptic_paused_pulses),
};

static FSM_STATE_T current_button_state = IDLE;
static int final_button_state[2] = {0, 0};

static bool btn_is_pressed;
static uint32_t btn_press_start_ms;
static bool pending_short_tap;
static uint32_t pending_short_tap_release_ms;
static uint32_t last_bookmark_accepted_ms;
static bool hold_armed_feedback_sent;
static bool heartbeat_sent[ARRAY_SIZE(heartbeat_start_ms)];
static bool poweroff_triggered;
static bool recording_paused;

static const struct haptic_pattern *active_haptic_pattern;
static uint8_t active_haptic_pattern_index;

static void check_button_level(struct k_work *work_item);
static void button_haptic_pattern_work_handler(struct k_work *work_item);
static void turnoff_all_with_haptic(uint32_t haptic_duration_ms);

K_WORK_DELAYABLE_DEFINE(button_work, check_button_level);
K_WORK_DELAYABLE_DEFINE(button_haptic_pattern_work, button_haptic_pattern_work_handler);

static inline void notify_button_event(uint8_t code, uint8_t value, const char *event_name)
{
    final_button_state[0] = code;
    final_button_state[1] = value;
    LOG_INF("%s", event_name);
    struct bt_conn *conn = get_current_connection();
    if (conn != NULL) {
        bt_gatt_notify(conn, &button_service.attrs[1], &final_button_state, sizeof(final_button_state));
    }
}

static inline void notify_release(void)
{
    notify_button_event(BUTTON_RELEASE, 0, "Button released");
}

static inline void notify_double_tap(void)
{
    notify_button_event(DOUBLE_TAP, 0, "Button double tap");
}

static inline void notify_bookmark_tap(void)
{
    notify_button_event(BOOKMARK_TAP, 0, "Button bookmark tap");
}

static inline void notify_pause_toggle(void)
{
    notify_button_event(PAUSE_TOGGLE, recording_paused ? 1 : 0, "Button pause/resume toggle");
}

static void stop_haptic_pattern(bool stop_motor)
{
    active_haptic_pattern = NULL;
    active_haptic_pattern_index = 0;
    k_work_cancel_delayable(&button_haptic_pattern_work);
    if (stop_motor) {
        play_haptic_milli(0);
    }
}

static void start_haptic_pattern(const struct haptic_pattern *pattern)
{
    if (pattern == NULL || pattern->pulse_count == 0 || pattern->pulse_ms == NULL) {
        return;
    }

    stop_haptic_pattern(false);
    active_haptic_pattern = pattern;
    active_haptic_pattern_index = 0;
    play_haptic_milli(pattern->pulse_ms[0]);

    if (pattern->pulse_count > 1) {
        uint32_t next_delay_ms = pattern->pulse_ms[0] + pattern->gap_ms[0];
        k_work_reschedule(&button_haptic_pattern_work, K_MSEC(next_delay_ms));
    } else {
        active_haptic_pattern = NULL;
    }
}

static void button_haptic_pattern_work_handler(struct k_work *work_item)
{
    ARG_UNUSED(work_item);
    if (active_haptic_pattern == NULL) {
        return;
    }

    active_haptic_pattern_index++;
    if (active_haptic_pattern_index >= active_haptic_pattern->pulse_count) {
        active_haptic_pattern = NULL;
        return;
    }

    play_haptic_milli(active_haptic_pattern->pulse_ms[active_haptic_pattern_index]);

    if ((active_haptic_pattern_index + 1U) < active_haptic_pattern->pulse_count) {
        uint32_t next_delay_ms = active_haptic_pattern->pulse_ms[active_haptic_pattern_index] +
                                 active_haptic_pattern->gap_ms[active_haptic_pattern_index];
        k_work_reschedule(&button_haptic_pattern_work, K_MSEC(next_delay_ms));
    } else {
        active_haptic_pattern = NULL;
    }
}

static void reset_press_state(void)
{
    btn_press_start_ms = 0;
    hold_armed_feedback_sent = false;
    poweroff_triggered = false;
    memset(heartbeat_sent, 0, sizeof(heartbeat_sent));
}

static void handle_bookmark_tap(uint32_t now_ms)
{
    struct bt_conn *conn = get_current_connection();
    if (conn == NULL) {
        LOG_INF("Bookmark ignored (not connected)");
        start_haptic_pattern(&haptic_pattern_nope);
        return;
    }

    bool accepted =
        (last_bookmark_accepted_ms == 0U) || ((now_ms - last_bookmark_accepted_ms) >= BOOKMARK_DEBOUNCE_WINDOW_MS);

    if (accepted) {
        last_bookmark_accepted_ms = now_ms;
        notify_bookmark_tap();
        stop_haptic_pattern(false);
        play_haptic_milli(HAPTIC_MARK_MS);
        return;
    }

    uint32_t cooldown_ms = BOOKMARK_DEBOUNCE_WINDOW_MS - (now_ms - last_bookmark_accepted_ms);
    LOG_INF("Bookmark ignored (cooldown active: %u ms left)", cooldown_ms);
    start_haptic_pattern(&haptic_pattern_nope);
}

static void process_hold_feedback(uint32_t hold_ms)
{
    if (!hold_armed_feedback_sent && hold_ms >= PAUSE_HOLD_THRESHOLD_MS) {
        hold_armed_feedback_sent = true;
        stop_haptic_pattern(false);
        play_haptic_milli(HAPTIC_ARMED_MS);
    }

    if (hold_ms >= POWEROFF_HEARTBEAT_START_MS && hold_ms < POWEROFF_HOLD_MS) {
        for (size_t i = 0; i < ARRAY_SIZE(heartbeat_start_ms); i++) {
            if (!heartbeat_sent[i] && hold_ms >= heartbeat_start_ms[i]) {
                heartbeat_sent[i] = true;
                stop_haptic_pattern(false);
                play_haptic_milli(heartbeat_pulse_ms[i]);
            }
        }
    }

    if (!poweroff_triggered && hold_ms >= POWEROFF_HOLD_MS) {
        poweroff_triggered = true;
        turnoff_all_with_haptic(HAPTIC_HEARTBEAT_FINAL_MS);
    }
}

static void check_button_level(struct k_work *work_item)
{
    ARG_UNUSED(work_item);
    uint32_t now_ms = k_uptime_get_32();
    uint8_t btn_state = was_pressed ? BUTTON_PRESSED : BUTTON_RELEASED;

    if (btn_state == BUTTON_PRESSED && !btn_is_pressed) {
        btn_is_pressed = true;
        btn_press_start_ms = now_ms;
        hold_armed_feedback_sent = false;
        poweroff_triggered = false;
        memset(heartbeat_sent, 0, sizeof(heartbeat_sent));
    } else if (btn_state == BUTTON_RELEASED && btn_is_pressed && !poweroff_triggered) {
        btn_is_pressed = false;
        uint32_t press_duration_ms = now_ms - btn_press_start_ms;

        if (press_duration_ms < TAP_THRESHOLD_MS) {
            if (pending_short_tap && (now_ms - pending_short_tap_release_ms) <= DOUBLE_TAP_WINDOW_MS) {
                pending_short_tap = false;
                notify_double_tap();
            } else {
                pending_short_tap = true;
                pending_short_tap_release_ms = now_ms;
            }
        } else if (press_duration_ms >= PAUSE_HOLD_THRESHOLD_MS && press_duration_ms < POWEROFF_HEARTBEAT_START_MS) {
            pending_short_tap = false;
            recording_paused = !recording_paused;
            notify_pause_toggle();
            start_haptic_pattern(recording_paused ? &haptic_pattern_paused : &haptic_pattern_resumed);
        } else if (press_duration_ms >= POWEROFF_HEARTBEAT_START_MS && press_duration_ms < POWEROFF_HOLD_MS) {
            pending_short_tap = false;
            stop_haptic_pattern(true);
        }

        current_button_state = GRACE;
        notify_release();
        reset_press_state();
    } else if (btn_state == BUTTON_RELEASED && !btn_is_pressed && pending_short_tap &&
               (now_ms - pending_short_tap_release_ms) > DOUBLE_TAP_WINDOW_MS) {
        pending_short_tap = false;
        handle_bookmark_tap(now_ms);
    } else if (btn_state == BUTTON_PRESSED && btn_is_pressed && !poweroff_triggered) {
        uint32_t hold_ms = now_ms - btn_press_start_ms;
        if (pending_short_tap && hold_ms >= TAP_THRESHOLD_MS) {
            pending_short_tap = false;
        }
        process_hold_feedback(hold_ms);
    }

    k_work_reschedule(&button_work, K_MSEC(BUTTON_CHECK_INTERVAL_MS));
}

static ssize_t button_data_read_characteristic(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               void *buf,
                                               uint16_t len,
                                               uint16_t offset)
{
    LOG_INF("button_data_read_characteristic");
    LOG_PRINTK("was_pressed: %d\n", final_button_state[0]);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &final_button_state, sizeof(final_button_state));
}

static struct gpio_callback button_cb_data;

static void button_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    was_pressed = (gpio_pin_get_dt(&usr_btn) == 1);
    LOG_INF("Button %s (GPIO callback)", was_pressed ? "pressed" : "released");
}

int button_regist_callback()
{
    int ret;

    // Configure GPIO as input with pull-up
    ret = gpio_pin_configure_dt(&usr_btn, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure button GPIO (%d)", ret);
        return ret;
    }

    // Setup interrupt on both edges
    ret = gpio_pin_interrupt_configure_dt(&usr_btn, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure button interrupt (%d)", ret);
        return ret;
    }

    // Register callback
    gpio_init_callback(&button_cb_data, button_gpio_callback, BIT(usr_btn.pin));
    gpio_add_callback(usr_btn.port, &button_cb_data);

    LOG_INF("Button initialized with GPIO interrupt");

    return 0;
}

int button_init()
{
    int ret;

    // Initialize the buttons device from evt
    if (!device_is_ready(buttons)) {
        LOG_ERR("Buttons device not ready");
        return -ENODEV;
    }

    // Enable runtime power management for the buttons device
    ret = pm_device_runtime_get(buttons);
    if (ret < 0) {
        LOG_ERR("Failed to enable buttons device (%d)", ret);
        return ret;
    }

    // Regist callback
    ret = button_regist_callback();
    if (ret < 0) {
        LOG_ERR("Failed to register buttons callback (%d)", ret);
        return ret;
    }

    return 0;
}

void activate_button_work()
{
    k_work_schedule(&button_work, K_MSEC(BUTTON_CHECK_INTERVAL_MS));
}

void register_button_service()
{
    bt_gatt_service_register(&button_service);
}

FSM_STATE_T get_current_button_state()
{
    return current_button_state;
}

void turnoff_all()
{
    turnoff_all_with_haptic(HAPTIC_SHUTDOWN_DEFAULT_MS);
}

static void turnoff_all_with_haptic(uint32_t haptic_duration_ms)
{
    int rc;

    // Immediate feedback: LED off and haptic
    led_off();
    // Set is_off immediately so set_led_state() keeps LEDs off
    is_off = true;

#ifdef CONFIG_OMI_ENABLE_HAPTIC
    if (haptic_duration_ms > 0U) {
        play_haptic_milli(haptic_duration_ms);
        k_msleep(haptic_duration_ms + HAPTIC_SHUTDOWN_SETTLE_MS);
        haptic_off();
    }
#endif

    // Delays for stability
    k_msleep(1000);

    // // Enter the low power mode
    transport_off();
    k_msleep(300);

    // Always turn off microphone
    mic_off();
    k_msleep(100);

    // Turn off speaker if enabled
#ifdef CONFIG_OMI_ENABLE_SPEAKER
    speaker_off();
    k_msleep(100);
#endif

    // Turn off accelerometer if enabled
#ifdef CONFIG_OMI_ENABLE_ACCELEROMETER
    accel_off();
    k_msleep(100);
#endif

    if (is_sd_on()) {
        app_sd_off();
    }
    k_msleep(300);

    // Put the buttons device to sleep if button is enabled
#ifdef CONFIG_OMI_ENABLE_BUTTON
    pm_device_runtime_put(buttons);
    k_msleep(100);
#endif

    // Disable USB if enabled
#ifdef CONFIG_OMI_ENABLE_USB
    NRF_USBD->INTENCLR = 0xFFFFFFFF;
#endif

    // Log system power off
    LOG_INF("System powering off");

    // Configure usr_btn as input with interrupt to allow wake-up
    rc = gpio_pin_configure_dt(&usr_btn, GPIO_INPUT);
    if (rc < 0) {
        LOG_ERR("Could not configure usr_btn GPIO (%d)", rc);
        return;
    }

    rc = gpio_pin_interrupt_configure_dt(&usr_btn, GPIO_INT_LEVEL_LOW);
    if (rc < 0) {
        LOG_ERR("Could not configure usr_btn GPIO interrupt (%d)", rc);
        return;
    }
    rc = watchdog_deinit();
    if (rc < 0) {
        LOG_ERR("Failed to deinitialize watchdog (%d)", rc);
        return;
    }

    /* Persist an IMU timestamp base so we can estimate time across system_off. */
    lsm6dsl_time_prepare_for_system_off();
    k_msleep(1000);
    LOG_INF("Entering system off; press usr_btn to restart");

    // Power off the system using sys_poweroff
    sys_poweroff();
}

void force_button_state(FSM_STATE_T state)
{
    current_button_state = state;
}

bool button_is_recording_paused(void)
{
    return recording_paused;
}
