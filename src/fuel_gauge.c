#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#include <nrf_fuel_gauge.h>

#include "fuel_gauge.h"

#define SETTINGS_KEY "fuel_gauge_state"

/* Store the state at most every 10 minutes, or right after a VBUS change. */
#define STATE_STORE_INTERVAL_MS (10 * 60 * 1000)

/* nPM13xx CHARGER.BCHGCHARGESTATUS register bitmasks */
#define NPM13XX_CHG_STATUS_COMPLETE_MASK BIT(1)
#define NPM13XX_CHG_STATUS_TRICKLE_MASK	 BIT(2)
#define NPM13XX_CHG_STATUS_CC_MASK	 BIT(3)
#define NPM13XX_CHG_STATUS_CV_MASK	 BIT(4)

static int64_t ref_time;
static int64_t last_store_time;
static bool store_pending;
static struct fuel_gauge_snapshot snapshot;

static const struct battery_model battery_model = {
#include "battery_model.inc"
};

static uint8_t fuel_gauge_state_buf[1024];

static int read_sensors(const struct device *charger, float *voltage, float *current, float *temp,
			int32_t *chg_status)
{
	struct sensor_value value;
	int ret;

	ret = sensor_sample_fetch(charger);
	if (ret < 0) {
		return ret;
	}

	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &value);
	*voltage = (float)value.val1 + ((float)value.val2 / 1000000);

	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_TEMP, &value);
	*temp = (float)value.val1 + ((float)value.val2 / 1000000);

	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &value);
	*current = (float)value.val1 + ((float)value.val2 / 1000000);

	sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_STATUS, &value);
	*chg_status = value.val1;

	return 0;
}

static int charge_status_inform(int32_t chg_status)
{
	union nrf_fuel_gauge_ext_state_info_data state_info;

	if (chg_status & NPM13XX_CHG_STATUS_COMPLETE_MASK) {
		state_info.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_COMPLETE;
	} else if (chg_status & NPM13XX_CHG_STATUS_TRICKLE_MASK) {
		state_info.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_TRICKLE;
	} else if (chg_status & NPM13XX_CHG_STATUS_CC_MASK) {
		/* The charge current may be limited by VBUS, hence CC_LIMITED. */
		state_info.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_CC_LIMITED;
	} else if (chg_status & NPM13XX_CHG_STATUS_CV_MASK) {
		state_info.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_CV;
	} else {
		state_info.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_IDLE;
	}

	return nrf_fuel_gauge_ext_state_update(NRF_FUEL_GAUGE_EXT_STATE_INFO_CHARGE_STATE_CHANGE,
					       &state_info);
}

int fuel_gauge_init(const struct device *charger)
{
	struct sensor_value value;
	struct nrf_fuel_gauge_init_parameters parameters =
		NRF_FUEL_GAUGE_DEFAULT_INIT_PARAMETERS_SECONDARY(0.0f, 0.0f, 0.0f, &battery_model);
	float max_charge_current;
	float term_charge_current;
	int32_t chg_status;
	int ret;

	ret = read_sensors(charger, &parameters.v0, &parameters.i0, &parameters.t0, &chg_status);
	if (ret < 0) {
		return ret;
	}

	/* Zephyr sensor API convention is negative=discharging, while nrf_fuel_gauge
	 * expects the opposite (negative=charging).
	 */
	parameters.i0 = -parameters.i0;

	/* Store charge nominal and termination current, needed for TTF calculation. */
	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT, &value);
	max_charge_current = (float)value.val1 + ((float)value.val2 / 1000000);
	term_charge_current = max_charge_current / 10.f;

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		ssize_t len;

		len = settings_load_one(SETTINGS_KEY, fuel_gauge_state_buf,
					sizeof(fuel_gauge_state_buf));
		if (len == nrf_fuel_gauge_state_size) {
			parameters.state = fuel_gauge_state_buf;
			parameters.state_size = len;
		}
	}

	ret = nrf_fuel_gauge_init(&parameters, NULL);
	if (ret < 0) {
		return ret;
	}

	ret = nrf_fuel_gauge_ext_state_update(NRF_FUEL_GAUGE_EXT_STATE_INFO_CHARGE_CURRENT_LIMIT,
					      &(union nrf_fuel_gauge_ext_state_info_data){
						      .charge_current_limit = max_charge_current});
	if (ret < 0) {
		return ret;
	}

	ret = nrf_fuel_gauge_ext_state_update(NRF_FUEL_GAUGE_EXT_STATE_INFO_TERM_CURRENT,
					      &(union nrf_fuel_gauge_ext_state_info_data){
						      .charge_term_current = term_charge_current});
	if (ret < 0) {
		return ret;
	}

	ret = charge_status_inform(chg_status);
	if (ret < 0) {
		return ret;
	}

	ref_time = k_uptime_get();
	last_store_time = ref_time;

	return 0;
}

int fuel_gauge_update(const struct device *charger, bool vbus_connected)
{
	static int32_t chg_status_prev;
	static bool vbus_prev;

	struct nrf_fuel_gauge_state_info state_info;
	float voltage;
	float current;
	float temp;
	float soc;
	float soh;
	float tte;
	float ttf;
	float delta;
	int32_t chg_status;
	int ret;

	ret = read_sensors(charger, &voltage, &current, &temp, &chg_status);
	if (ret < 0) {
		return ret;
	}

	ret = nrf_fuel_gauge_ext_state_update(
		vbus_connected ? NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_CONNECTED
			       : NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_DISCONNECTED,
		NULL);
	if (ret < 0) {
		return ret;
	}

	if (chg_status != chg_status_prev) {
		chg_status_prev = chg_status;

		ret = charge_status_inform(chg_status);
		if (ret < 0) {
			return ret;
		}
	}

	if (vbus_connected != vbus_prev) {
		vbus_prev = vbus_connected;
		store_pending = true;
	}

	delta = (float)k_uptime_delta(&ref_time) / 1000.f;

	/* Negative=charging for the fuel gauge library. */
	current = -current;
	ret = nrf_fuel_gauge_process(voltage, current, temp, delta, &soc, &state_info);
	if (ret < 0) {
		return ret;
	}

	ret = nrf_fuel_gauge_soh_get(&soh);
	if (ret < 0) {
		soh = NAN;
	}

	if (current >= 0.0f) {
		/* Discharging */
		ret = nrf_fuel_gauge_tte_get(&tte);
		if (ret < 0) {
			tte = NAN;
		}
		ttf = NAN;
	} else {
		/* Charging */
		ret = nrf_fuel_gauge_ttf_get(&ttf);
		if (ret < 0) {
			ttf = NAN;
		}
		tte = NAN;
	}

	snapshot.valid = true;
	snapshot.vbus_connected = vbus_connected;
	snapshot.chg_status = chg_status;
	snapshot.voltage = voltage;
	snapshot.current = current;
	snapshot.temp = temp;
	snapshot.soc = soc;
	snapshot.soh = soh;
	snapshot.tte = tte;
	snapshot.ttf = ttf;

	return 0;
}

int fuel_gauge_get_snapshot(struct fuel_gauge_snapshot *out)
{
	if (!out) {
		return -EINVAL;
	}

	*out = snapshot;

	return 0;
}

void fuel_gauge_store_state(void)
{
	int64_t now;
	int ret;

	if (!IS_ENABLED(CONFIG_SETTINGS)) {
		return;
	}

	now = k_uptime_get();

	if (!store_pending && (now - last_store_time) < STATE_STORE_INTERVAL_MS) {
		return;
	}

	ret = nrf_fuel_gauge_state_get(fuel_gauge_state_buf, sizeof(fuel_gauge_state_buf));
	if (ret < 0) {
		return;
	}

	ret = settings_save_one(SETTINGS_KEY, fuel_gauge_state_buf, nrf_fuel_gauge_state_size);
	if (ret < 0) {
		return;
	}

	store_pending = false;
	last_store_time = now;
}
