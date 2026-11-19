#ifndef __FUEL_GAUGE_H__
#define __FUEL_GAUGE_H__

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

/** @brief Latest fuel gauge measurement. */
struct fuel_gauge_snapshot {
	/** True once the fuel gauge algorithm produced a first valid result. */
	bool valid;
	/** True when VBUS is present. */
	bool vbus_connected;
	/** nPM13xx charger status register value. */
	int32_t chg_status;
	/** Battery voltage [V]. */
	float voltage;
	/** Battery current [A], positive = discharging (Zephyr sensor convention). */
	float current;
	/** Battery temperature [degC]. */
	float temp;
	/** State of charge [%]. */
	float soc;
	/** State of health [%]. */
	float soh;
	/** Time to empty [s]. May be NAN. */
	float tte;
	/** Time to full [s]. May be NAN. */
	float ttf;
};

int fuel_gauge_init(const struct device *charger);

int fuel_gauge_update(const struct device *charger, bool vbus_connected);

int fuel_gauge_get_snapshot(struct fuel_gauge_snapshot *out);

/** @brief Persist the fuel gauge state if it is due (periodic or after VBUS change). */
void fuel_gauge_store_state(void);

#endif /* __FUEL_GAUGE_H__ */
