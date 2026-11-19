# rapoo_npm1300

Combined application based on two NCS v3.4.1 samples:

- `nrf/samples/bluetooth/peripheral_power_profiling` (BLE power profiling peripheral)
- `nrf/samples/pmic/native/npm13xx_fuel_gauge` (nPM1300 fuel gauge)

Target: `nrf54lm20dk/nrf54lm20a/cpuapp`

## What was changed

- Power profiling sample merged with the nPM1300 fuel gauge.
- Buttons, LEDs, NFC, UART console, shell and all logging removed.
- Only one I2C bus is kept: **SDA = P1.09, SCL = P1.18** (400 kHz).
- All other board peripheral nodes are disabled/deleted (UART20/30, SPI00/22,
  PWM20/21, ADC, NFCT, IEEE 802.15.4, USB, temp sensor, GPIO0-3, GPIOTE20/30).
- nPM1300 has no host interrupt GPIO: VBUS is polled every 500 ms.
- 32 kHz LFCLK uses the internal RC oscillator (`CONFIG_CLOCK_CONTROL_NRF_K32SRC_RC`),
  RC calibration stays enabled (uses HFXO, required by the radio anyway).
- No boot button: advertising starts automatically at boot and is restarted after
  a disconnect or advertising timeout.

## BLE data format

PWR service characteristic (UUID `...1630`, char `...1631` read/notify), 20 bytes,
little endian:

| offset | size | content |
|---|---|---|
| 0 | 1 | protocol version (1) |
| 1 | 1 | flags: bit0 VBUS present, bit1 charging |
| 2 | 2 | SoC x100 [%] |
| 4 | 2 | battery voltage [mV] |
| 6 | 2 | current [mA x10], positive = discharging |
| 8 | 2 | temperature [degC x100] |
| 10 | 2 | state of health x100 [%] |
| 12 | 2 | time to empty/full [min], 0xFFFF = not available |
| 14 | 6 | reserved, zero |

Notifications are sent every 500 ms while notifications are enabled, for up to
10 s after enabling (power profiling behaviour). Values are refreshed every
500 ms from the nPM1300.

## Build

```bash
source ~/ncs-env.sh
cd ~/projects/rapoo_npm1300
west build -p -b nrf54lm20dk/nrf54lm20a/cpuapp -d build .
```

Flash (only after checking the connected device):

```bash
west flash -d build
```

## Notes

- On the nRF54LM20 DK, P1.09 is wired to Button 1 and P1.18 to UART20 RTS. Both
  nodes are removed in the overlay, but the physical wiring on the DK can still
  disturb the I2C bus. Use the nPM1300 EK wiring (SDA P1.09, SCL P1.18, plus
  VBUS/BAT) or a custom board.
- The fuel gauge uses the sample's default `battery_model.inc` ("Example" battery).
  Replace it with the real battery model and adjust `term-microvolt`,
  `current-microamp` in the overlay for accurate SoC.
- Fuel gauge state and BLE bonding data are stored in RRAM using ZMS
  (`storage_partition`).
