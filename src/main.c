#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mfd/npm13xx.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#include "advertising.h"
#include "pwr_service.h"
#include "fuel_gauge.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define MEASUREMENT_INTERVAL	 K_MSEC(2000)
#define ADVERTISING_WINDOW_MS	 60000
#define IDLE_WINDOW_MS		 60000
/* nPM1300 timer maximum is 0xFFFFFF * 16 ms (~74.5 hours). */
#define HIBERNATE_WAKE_MS	 268000000U
#define ADV_RESTART_DELAY	 K_MSEC(100)

#define NOTIFICATION_INTERVAL	 CONFIG_BT_POWER_PROFILING_NOTIFICATION_INTERVAL
#define NOTIFICATION_TIMEOUT	 CONFIG_BT_POWER_PROFILING_NOTIFICATION_TIMEOUT
#define NOTIFICATION_PIPELINE_SIZE CONFIG_BT_CONN_TX_MAX

#define PAYLOAD_LEN		  CONFIG_BT_POWER_PROFILING_DATA_LENGTH
#define PAYLOAD_VERSION		  1
#define PAYLOAD_FLAG_VBUS	  BIT(0)
#define PAYLOAD_FLAG_CHARGING	  BIT(1)
#define PAYLOAD_NOT_AVAILABLE	  0xFFFF
#define CHARGING_ACTIVE_MASK	  (BIT(2) | BIT(3) | BIT(4))

#if PAYLOAD_LEN < 14
#error "CONFIG_BT_POWER_PROFILING_DATA_LENGTH must be at least 14 bytes"
#endif

static const struct device *const pmic = DEVICE_DT_GET(DT_NODELABEL(npm1300_pmic));
static const struct device *const charger = DEVICE_DT_GET(DT_NODELABEL(npm1300_charger));

static struct bt_conn *device_conn;
static atomic_t notify_pipeline = ATOMIC_INIT(NOTIFICATION_PIPELINE_SIZE);

static void notify_work_handler(struct k_work *work);
static void notify_timeout_handler(struct k_work *work);
static void adv_restart_handler(struct k_work *work);
static void idle_handler(struct k_work *work);
static void power_off_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(notify_work, notify_work_handler);
static K_WORK_DELAYABLE_DEFINE(notify_timeout, notify_timeout_handler);
static K_WORK_DELAYABLE_DEFINE(adv_restart_work, adv_restart_handler);
static K_WORK_DELAYABLE_DEFINE(idle_work, idle_handler);
static K_WORK_DELAYABLE_DEFINE(power_off_work, power_off_handler);

static bool idle_mode;

static uint16_t clamp_u16(float value)
{
	if (!isfinite(value) || value <= 0.f) {
		return 0;
	}

	if (value >= (float)UINT16_MAX) {
		return UINT16_MAX;
	}

	return (uint16_t)lroundf(value);
}

static int16_t clamp_s16(float value)
{
	if (!isfinite(value)) {
		return 0;
	}

	if (value >= (float)INT16_MAX) {
		return INT16_MAX;
	}

	if (value <= (float)INT16_MIN) {
		return INT16_MIN;
	}

	return (int16_t)lroundf(value);
}

static bool vbus_is_connected(void)
{
	struct sensor_value val;
	int ret;

	ret = sensor_attr_get(charger, SENSOR_CHAN_CURRENT, SENSOR_ATTR_UPPER_THRESH, &val);
	if (ret < 0) {
		return false;
	}

	return (val.val1 != 0) || (val.val2 != 0);
}

/*
 * nPM1300 register map, see zephyr/drivers/regulator/regulator_npm13xx.c.
 *
 * LDO1/LDO2 and LDSW1/LDSW2 are the same output pins: each block works either
 * as an LDO or as a load switch, so clearing the enable bit turns both off.
 */
#define NPM13XX_BUCK_BASE	   0x04U
#define NPM13XX_BUCK_OFFSET_EN_CLR  0x01U
#define NPM13XX_BUCK_OFFSET_PWM_CLR 0x05U
#define NPM13XX_BUCK_OFFSET_CTRL0   0x15U
#define NPM13XX_BUCK_OFFSET_STATUS  0x34U
#define NPM13XX_BUCK1_ON_MASK	   0x04U
#define NPM13XX_BUCK1_PFM_MASK	   0x01U
#define NPM13XX_BUCK2_ON_MASK	   0x40U

#define NPM13XX_LDSW_BASE	    0x08U
#define NPM13XX_LDSW_OFFSET_EN_CLR  0x01U
#define NPM13XX_LDSW_OFFSET_STATUS  0x04U
#define NPM13XX_LDSW1_ON_MASK	    0x03U
#define NPM13XX_LDSW2_ON_MASK	    0x0CU

static int pmic_disable_unused_rails(void)
{
	int err;

	/* BUCK2 is not used on this design. */
	err = mfd_npm13xx_reg_write(pmic, NPM13XX_BUCK_BASE,
				    NPM13XX_BUCK_OFFSET_EN_CLR + 2U, 1U);
	if (err) {
		LOG_ERR("Cannot disable BUCK2 (%d)", err);
		return err;
	}

	/* Disable LDO1 / load switch 1. */
	err = mfd_npm13xx_reg_write(pmic, NPM13XX_LDSW_BASE, NPM13XX_LDSW_OFFSET_EN_CLR, 1U);
	if (err) {
		LOG_ERR("Cannot disable LDO1/LS1 (%d)", err);
		return err;
	}

	/* Disable LDO2 / load switch 2. */
	err = mfd_npm13xx_reg_write(pmic, NPM13XX_LDSW_BASE,
				    NPM13XX_LDSW_OFFSET_EN_CLR + 2U, 1U);
	if (err) {
		LOG_ERR("Cannot disable LDO2/LS2 (%d)", err);
		return err;
	}

	return 0;
}

/*
 * Force BUCK1 into PFM (pulse frequency modulation) mode: set the PFM bit in
 * BUCK_CTRL0 and clear the PWM bit. This is a fixed mode, not auto.
 */
static int pmic_set_buck1_pfm(void)
{
	uint8_t ctrl = 0;
	int err;

	err = mfd_npm13xx_reg_update(pmic, NPM13XX_BUCK_BASE, NPM13XX_BUCK_OFFSET_CTRL0,
				     NPM13XX_BUCK1_PFM_MASK, NPM13XX_BUCK1_PFM_MASK);
	if (err) {
		LOG_ERR("Cannot set BUCK1 PFM bit (%d)", err);
		return err;
	}

	err = mfd_npm13xx_reg_write(pmic, NPM13XX_BUCK_BASE, NPM13XX_BUCK_OFFSET_PWM_CLR, 1U);
	if (err) {
		LOG_ERR("Cannot clear BUCK1 PWM bit (%d)", err);
		return err;
	}

	if (mfd_npm13xx_reg_read(pmic, NPM13XX_BUCK_BASE, NPM13XX_BUCK_OFFSET_CTRL0, &ctrl) == 0) {
		LOG_INF("BUCK1 fixed PFM (BUCK_CTRL0=0x%02x)", ctrl);
	}

	return 0;
}

static void pmic_log_rail_status(void)
{
	uint8_t buck = 0;
	uint8_t ldsw = 0;
	int err;

	err = mfd_npm13xx_reg_read(pmic, NPM13XX_BUCK_BASE, NPM13XX_BUCK_OFFSET_STATUS, &buck);
	if (err) {
		LOG_WRN("Cannot read BUCK status (%d)", err);
		return;
	}

	err = mfd_npm13xx_reg_read(pmic, NPM13XX_LDSW_BASE, NPM13XX_LDSW_OFFSET_STATUS, &ldsw);
	if (err) {
		LOG_WRN("Cannot read LDO/LS status (%d)", err);
		return;
	}

	LOG_INF("PMIC rails: BUCK1=%d BUCK2=%d LDO1/LS1=%d LDO2/LS2=%d (0x%02x/0x%02x)",
		!!(buck & NPM13XX_BUCK1_ON_MASK), !!(buck & NPM13XX_BUCK2_ON_MASK),
		!!(ldsw & NPM13XX_LDSW1_ON_MASK), !!(ldsw & NPM13XX_LDSW2_ON_MASK),
		buck, ldsw);
}

/*
 * 20-byte payload (little endian):
 *  0     version
 *  1     flags: bit0 VBUS, bit1 charging
 *  2..3  SoC x100 [%]
 *  4..5  voltage [mV]
 *  6..7  current [mA x10], positive = discharging
 *  8..9  temperature [degC x100]
 * 10..11 state of health x100 [%]
 * 12..13 time to empty/full [min], 0xFFFF = not available
 * 14..   reserved, zero
 */
static void log_measurement(const struct fuel_gauge_snapshot *snap, uint8_t flags)
{
	int soc = (int)lroundf(snap->soc * 100.f);
	int voltage = (int)lroundf(snap->voltage * 1000.f);
	int current = (int)lroundf(snap->current * 10000.f);
	int temp = (int)lroundf(snap->temp * 100.f);
	int soh = (int)lroundf(snap->soh * 100.f);

	LOG_INF("battery SoC=%d.%02d%% V=%d mV I=%d.%d mA T=%d.%02d C SoH=%d.%02d%% VBUS=%d CHG=%d",
		soc / 100, abs(soc % 100), voltage, current / 10, abs(current % 10),
		temp / 100, abs(temp % 100), soh / 100, abs(soh % 100),
		(flags & PAYLOAD_FLAG_VBUS) ? 1 : 0,
		(flags & PAYLOAD_FLAG_CHARGING) ? 1 : 0);
}

static void publish_measurement(bool vbus_connected)
{
	struct fuel_gauge_snapshot snap;
	uint8_t payload[PAYLOAD_LEN] = {0};
	uint16_t time_to = PAYLOAD_NOT_AVAILABLE;
	uint8_t flags = 0;
	float minutes;

	if (fuel_gauge_get_snapshot(&snap) < 0 || !snap.valid) {
		LOG_WRN("Fuel gauge snapshot not valid");
		return;
	}

	if (vbus_connected) {
		flags |= PAYLOAD_FLAG_VBUS;
	}

	if (snap.chg_status & CHARGING_ACTIVE_MASK) {
		flags |= PAYLOAD_FLAG_CHARGING;
	}

	if (isfinite(snap.tte)) {
		minutes = snap.tte / 60.f;
		time_to = (uint16_t)MIN(clamp_u16(minutes), (uint16_t)(PAYLOAD_NOT_AVAILABLE - 1));
	} else if (isfinite(snap.ttf)) {
		minutes = snap.ttf / 60.f;
		time_to = (uint16_t)MIN(clamp_u16(minutes), (uint16_t)(PAYLOAD_NOT_AVAILABLE - 1));
	}

	payload[0] = PAYLOAD_VERSION;
	payload[1] = flags;
	sys_put_le16(clamp_u16(snap.soc * 100.f), &payload[2]);
	sys_put_le16(clamp_u16(snap.voltage * 1000.f), &payload[4]);
	sys_put_le16((uint16_t)clamp_s16(snap.current * 10000.f), &payload[6]);
	sys_put_le16((uint16_t)clamp_s16(snap.temp * 100.f), &payload[8]);
	sys_put_le16(clamp_u16(snap.soh * 100.f), &payload[10]);
	sys_put_le16(time_to, &payload[12]);

	log_measurement(&snap, flags);

	(void)pwr_service_set_data(payload, sizeof(payload));
}

static void notify_work_handler(struct k_work *work)
{
	int err;
	atomic_val_t pipeline = atomic_get(&notify_pipeline);

	if (pipeline == 0) {
		/* No room in the Bluetooth stack for another notification. */
		k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(NOTIFICATION_INTERVAL));
		return;
	}

	err = pwr_service_notify(NULL);
	if (err == 0) {
		atomic_dec(&notify_pipeline);
	}

	k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(NOTIFICATION_INTERVAL));
}

static void notify_timeout_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (device_conn) {
		k_work_cancel_delayable(&notify_work);

		(void)bt_conn_disconnect(device_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
}

static void pwr_notify_sent_cb(struct bt_conn *conn, void *user_data)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(user_data);

	atomic_inc(&notify_pipeline);
}

static void pwr_notify_status_cb(enum pwr_service_status status, void *user_data)
{
	ARG_UNUSED(user_data);

	switch (status) {
	case PWR_SERVICE_STATUS_NOTIFY_ENABLED:
		k_work_schedule(&notify_timeout, K_MSEC(NOTIFICATION_TIMEOUT));
		k_work_schedule(&notify_work, K_NO_WAIT);
		break;

	case PWR_SERVICE_STATUS_NOTIFY_DISABLED:
		k_work_cancel_delayable(&notify_work);
		k_work_cancel_delayable(&notify_timeout);
		break;

	default:
		break;
	}
}

static const struct pwr_service_cb pwr_service_callback = {
	.sent_cb = pwr_notify_sent_cb,
	.notify_status_cb = pwr_notify_status_cb,
};

static void start_advertising_window(void)
{
	int err;

	if (idle_mode) {
		return;
	}

	err = advertising_start_connectable();
	if (err) {
		LOG_ERR("Advertising start failed (%d)", err);
	} else {
		LOG_INF("Advertising as \"%s\", window %u s", CONFIG_BT_DEVICE_NAME,
			ADVERTISING_WINDOW_MS / 1000U);
	}

	/* Only the start of a window arms the 60 s timer; internal advertising
	 * restarts must not extend it.
	 */
	if (!k_work_delayable_is_pending(&idle_work)) {
		k_work_reschedule(&idle_work, K_MSEC(ADVERTISING_WINDOW_MS));
	}
}

static void adv_restart_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	start_advertising_window();
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	if (conn_err) {
		return;
	}

	device_conn = conn;

	/* A connection cancels the idle / power-off countdown. */
	k_work_cancel_delayable(&idle_work);

	LOG_INF("Central connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);

	device_conn = NULL;

	atomic_set(&notify_pipeline, NOTIFICATION_PIPELINE_SIZE);

	k_work_cancel_delayable(&notify_work);
	k_work_cancel_delayable(&notify_timeout);

	LOG_INF("Central disconnected (0x%02x)", reason);

	/* Restart the advertising window from scratch. */
	k_work_schedule(&adv_restart_work, ADV_RESTART_DELAY);
}

static void idle_handler(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);

	if (device_conn) {
		/* A connection arrived just before the timer: stay active. */
		return;
	}

	idle_mode = true;

	k_work_cancel_delayable(&adv_restart_work);
	(void)advertising_stop();

	err = bt_disable();

	LOG_INF("Idle mode: advertising stopped, Bluetooth %s",
		err ? "disable failed" : "disabled");

	k_work_schedule(&power_off_work, K_MSEC(IDLE_WINDOW_MS));
}

static void power_off_handler(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);

	LOG_INF("Power off: nPM1300 hibernate (wake in %u ms)", HIBERNATE_WAKE_MS);

	/* Flush the line to the RTT backend before hibernate cuts the supply. */
	log_flush();
	k_msleep(10);

	err = mfd_npm13xx_hibernate(pmic, HIBERNATE_WAKE_MS);
	if (err) {
		LOG_ERR("Hibernate failed (%d)", err);
	}
}

BT_CONN_CB_DEFINE(connection_cb) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void on_adv_terminated(void)
{
	if (!device_conn && !idle_mode) {
		k_work_schedule(&adv_restart_work, ADV_RESTART_DELAY);
	}
}

static const struct advertising_cb adv_module_cb = {
	.terminated = on_adv_terminated,
};

int main(void)
{
	int err;

	LOG_INF("Rapoo PMIC monitor starting");

	if (!device_is_ready(pmic) || !device_is_ready(charger)) {
		LOG_ERR("nPM1300 not ready (pmic=%d charger=%d)", device_is_ready(pmic),
			device_is_ready(charger));
		return 0;
	}

	/* LDO1/LDO2 and the load switches share outputs, turning them off here
	 * disables both functions. BUCK2 is intentionally left enabled: on this
	 * setup it may be the rail that powers the DK SoC.
	 */
	pmic_log_rail_status();

	err = pmic_disable_unused_rails();
	if (err) {
		LOG_WRN("Unused rail disable failed (%d)", err);
	}

	pmic_log_rail_status();

	err = pmic_set_buck1_pfm();
	if (err) {
		LOG_WRN("BUCK1 PFM configuration failed (%d)", err);
	}

	err = pwr_service_cb_register(&pwr_service_callback, NULL);
	if (err) {
		LOG_ERR("pwr_service_cb_register failed (%d)", err);
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed (%d)", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		err = settings_subsys_init();
		if (err) {
			LOG_ERR("settings_subsys_init failed (%d)", err);
			return 0;
		}

		err = settings_load();
		if (err) {
			LOG_ERR("settings_load failed (%d)", err);
			return 0;
		}
	}

	err = advertising_init(&adv_module_cb);
	if (err) {
		LOG_ERR("advertising_init failed (%d)", err);
		return 0;
	}

	if (fuel_gauge_init(charger) < 0) {
		LOG_ERR("fuel_gauge_init failed");
		return 0;
	}

	/* Start the first 60 s advertising window. */
	start_advertising_window();

	for (;;) {
		bool vbus_connected = vbus_is_connected();

		err = fuel_gauge_update(charger, vbus_connected);
		if (err) {
			LOG_ERR("fuel_gauge_update failed (%d)", err);
		}

		publish_measurement(vbus_connected);
		fuel_gauge_store_state();

		k_sleep(MEASUREMENT_INTERVAL);
	}
}
