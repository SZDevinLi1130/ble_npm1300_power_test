# nPM1300 GPOIRQ 导致静态电流增加 ~5 mA —— 问题结案报告

| 项目 | 内容 |
|---|---|
| 日期 | 2026-11-19 |
| 项目 | rapoo_npm1300 |
| SDK | nRF Connect SDK v3.4.1（Zephyr 4.4.2） |
| 硬件 | 自研板（nRF54LM20A + nPM1300） |
| 相关外设 | nPM1300 GPIO3（GPOIRQ）→ SoC P0.09 |
| 现象 | 使能 PMIC 中断后，VBAT 静态电流增加约 **4–5 mA** |
| 结论 | NCS MFD 驱动未清除 ADC/BCHARGER0 事件，GPOIRQ 被锁存事件拉高，驱动工作队列自旋，导致 CPU/I2C 持续繁忙 |
| 处理 | 移除 PMIC 中断，改为每 2 s 轮询 VBUS/充电状态 |

---

## 1. 现象

固件中启用 nPM1300 中断（devicetree 中配置 `host-int-gpios = <&gpio0 9 0>`、`pmic-int-pin = <3>`，并启用 `gpio0` / `gpiote30`）后：

- 整机静态电流比关闭中断时高 **约 4–5 mA**；
- 功能本身正常（电量、电压、BLE 广播/通知均正常）；
- 关闭中断后电流回到约 **0.8 mA** 基线。

---

## 2. 测试条件

| 项目 | 说明 |
|---|---|
| 供电 | PPK2 SMU 模式，4200 mV 加在 nPM1300 VBAT |
| 电流测量 | PPK2（VBAT 侧） |
| 日志 | J-Link RTT |
| 中断配置 | PMIC GPIO3 = GPOIRQ（模式 5），SoC P0.09 输入，上升沿中断 |
| DT 配置 | `host-int-gpios = <&gpio0 9 0>`（ACTIVE_HIGH） |

---

## 3. 排查过程与关键证据

由于现场没有示波器，采用“软件可观测”的方式排查：在固件中周期读取

- P0.09 **原始电平**（`gpio_pin_get_raw()`）；
- 2 秒内的 **中断边沿计数**；
- nPM1300 **8 组事件寄存器**（`EVENTS*SET`）；
- 对应的 **8 组中断使能寄存器**（`INTENEVENTS*SET`）；
- PMIC GPIO3 的模式寄存器（应等于 `0x05` = GPOIRQ）。

### 3.1 第一轮观测

```
diag irq=0 lvl=1 CHG=00 BAT=00 SHP=00 VBUS=14 GPIO=00 MODE3=05
```

- `irq=0`：2 秒内没有边沿 → 不是信号抖动；
- `lvl=1`：P0.09 一直为高；
- `VBUS=0x14`：`EVENTSVBUSIN0SET(0x16)` 的 bit2（过压检测）和 bit4（欠压检测）被锁存；
- `MODE3=05`：PMIC GPIO3 已正确配置为 GPOIRQ。

### 3.2 清除 VBUS 事件后

对 `EVENTSVBUSIN0CLR(0x17)` 写入 `0x3C` 清除过压/欠压事件后：

```
diag irq=0 lvl=1 EV 77 2f 00 00 00 00 00 00
diag EN 00 00 00 00 00 00 00 00 MODE3=05
```

事件寄存器顺序：`ADC, BCHG0, BCHG1, BCHG2, SHPHLD, VBUS0, VBUS1, GPIO`

- `ADC = 0x77`：`EVENTSADCSET(0x02)`，bit0/1/2/4/5/6 = VBAT/NTC/芯片温度/VSET1/VSET2/IBAT 的“ADC 转换完成”事件；
- `BCHG0 = 0x2F`：`EVENTSBCHARGER0SET(0x06)`，电池温度区域/芯片温度事件（本板未接 NTC，温度读数 `-88.12 °C`）；
- `EN = 00 ...`：所有中断使能寄存器读出均为 0；
- `lvl=1`：**即使所有中断使能都为 0，GPOIRQ 仍然为高**。

### 3.3 关键结论

1. nPM1300 的 GPOIRQ 是**事件锁存位的直接“或”**，不受 `INTENEVENTS*` 使能位控制；
2. ADC 事件在**每次 ADC 转换完成时重新锁存**，而应用每 2 s 读一次传感器（`sensor_sample_fetch()`），因此 ADC 事件会被周期性重新置位；
3. NCS 的 MFD 驱动只清除了部分事件组，**ADC(0x02) 与 BCHARGER0(0x06) 永远不会被清除**。

---

## 4. 根因分析

### 4.1 NCS MFD 驱动的事件覆盖不完整

`zephyr/drivers/mfd/mfd_npm13xx.c` 中的事件表：

```c
static const struct event_reg_t event_reg[NPM13XX_EVENT_MAX] = {
    [NPM13XX_EVENT_CHG_COMPLETED]  = {0x0AU, 0x10U},
    [NPM13XX_EVENT_CHG_ERROR]      = {0x0AU, 0x20U},
    [NPM13XX_EVENT_BATTERY_DETECTED] = {0x0EU, 0x01U},
    [NPM13XX_EVENT_BATTERY_REMOVED]  = {0x0EU, 0x02U},
    [NPM13XX_EVENT_SHIPHOLD_PRESS]   = {0x12U, 0x01U},
    [NPM13XX_EVENT_SHIPHOLD_RELEASE] = {0x12U, 0x02U},
    [NPM13XX_EVENT_WATCHDOG_WARN]    = {0x12U, 0x08U},
    [NPM13XX_EVENT_VBUS_DETECTED]    = {0x16U, 0x01U},
    [NPM13XX_EVENT_VBUS_REMOVED]     = {0x16U, 0x02U},
    [NPM13XX_EVENT_GPIO0_EDGE]       = {0x22U, 0x01U},
    ...
};
```

覆盖的寄存器：`0x0A / 0x0E / 0x12 / 0x16（仅 bit0/1）/ 0x22`。

**未覆盖**：

| 寄存器 | 内容 | 本问题中是否锁存 |
|---|---|---|
| `0x02` EVENTSADCSET | ADC 转换完成事件 | ✅ `0x77` |
| `0x06` EVENTSBCHARGER0SET | 电池温度区域/芯片温度事件 | ✅ `0x2F` |
| `0x1A` EVENTSVBUSIN1SET | 热/充电器检测事件 | 本次为 0 |
| `0x16` bit2..5 | VBUS 过压/欠压事件 | ✅ `0x14` |

### 4.2 驱动的工作队列自旋

`mfd_npm13xx.c` 的 `work_callback()` 末尾：

```c
/* Resubmit handler to queue if interrupt is still active */
if (gpio_pin_get_dt(&config->host_int_gpios) != 0) {
        k_work_submit(&data->work);
}
```

- 只要 P0.09 仍为高，就会立即重新排队；
- 每轮都会通过 I2C 读取 0x26 字节的 MAIN 寄存器并清事件；
- 由于 ADC/BCHARGER0 事件从未被清除，P0.09 永远保持高；
- 结果是 **CPU + I2C 持续繁忙**，形成稳定的 ~5 mA 功耗。

### 4.3 事件为什么反复出现

- 应用主循环每 2 s 调用 `fuel_gauge_update()` → `sensor_sample_fetch()`；
- 充电器驱动触发 nPM1300 ADC 转换；
- 每次转换完成都会置位 `EVENTSADCSET`；
- 该事件无人清除 → GPOIRQ 保持高。

---

## 5. 为什么在 nRF54LM20 DK 上测不出来

DK 上的 P0.09 **不是空闲 GPIO**。官方 pin map：

| GPIO | Function | 扩展排针 | 默认连到 P1（IMCU） |
|---|---|---|---|
| P0.06 | UART0_TXD | – | Yes |
| P0.07 | UART0_RXD | – | Yes |
| P0.08 | UART0_RST | – | Yes |
| **P0.09** | **UART0_CST (CTS)** | **–** | **Yes** |

因此：

- DK 上 P0.09 与板载 IMCU（nRF5340）的 UART RTS 共用同一网络；
- SoC 读到的电平主要由 IMCU 的 RTS 决定（空闲通常为低）；
- MFD 驱动执行 `gpio_pin_get_dt()` 得到 0，不会自旋 → 电流正常；
- **该测试条件掩盖了问题**，不能作为“中断可用”的有效对照。

补充：Nordic 官方 nPM1300 EK 接线表中，nRF54L 系列 DK 的 GPIO3 推荐接 **P0.04**（扩展排针上的空闲 GPIO），而不是 P0.09。

---

## 6. 修复方案（已实施）

### 6.1 移除 PMIC 中断

产品每 2 s 已轮询 VBUS/充电状态，PMIC 中断没有实际收益，因此直接移除：

**`boards/nrf54lm20dk_nrf54lm20a_cpuapp.overlay`**

```dts
/* No GPIO consumer in this application (PMIC interrupt is not used ...) */
&gpio0 {
    status = "disabled";
};

&gpiote30 {
    status = "disabled";
};

npm1300_pmic: pmic@6b {
    compatible = "nordic,npm1300";
    reg = <0x6b>;

    /* PMIC interrupt GPIO not used: GPOIRQ is a raw event OR and the MFD
     * driver leaves ADC/BCHARGER0 events uncleared ... VBUS/charge status
     * is polled.
     */
    npm1300_charger: charger {
        ...
    };
};
```

**`src/main.c`**

- 删除全部临时诊断代码（IRQ 计数、事件寄存器打印、极性测试、OV/UV mask）；
- 保留每 2 s 的 `vbus_is_connected()` + `fuel_gauge_update()` 轮询；
- 其他电源配置保持不变：
  - `pmic_disable_unused_rails()`：关闭 BUCK2 / LDO1 / LDO2（含负载开关）；
  - `pmic_set_buck1_pfm()`：BUCK1 固定 PFM。

### 6.2 编译结果

```
BOARD: nrf54lm20dk/nrf54lm20a/cpuapp
FLASH: 236104 B (11.32%)
RAM:    35324 B (6.75%)
```

### 6.3 验证

- 关闭中断的版本多次实测：VBAT 平均电流回到 **约 0.78–0.82 mA**；
- 打开中断的版本：实测比基线高 **约 4–5 mA**；
- 差分与根因分析一致；
- 最终清理版（无诊断代码）已编译（FLASH 236104 B / RAM 35324 B），建议烧录后复测一次基线电流。

---

## 7. 如果后续仍需要 PMIC 中断

当前 NCS 版本的 Zephyr MFD 驱动不能直接用于该场景，可选：

1. **给 MFD 驱动打补丁**（推荐程度中）：
   - 在 `work_callback()` 中清除全部 8 组事件寄存器（至少包含 `0x02` ADC 与 `0x06` BCHARGER0）；
   - 或按需仅清 `0x02` / `0x06`。
2. **改用 Nordic 的 npmx 驱动**（推荐程度高）：
   - 使用 `npmx_core_event_interrupt_enable()` + 逐事件回调；
   - 能正确处理 ADC/温度/VBUS 等事件，避免“未知事件卡死 IRQ”。
3. **保持轮询**（当前方案，推荐用于本产品）：
   - 结构最简单、功耗最低、无驱动耦合；
   - 2 s 的 VBUS/充电状态响应对当前产品足够。

---

## 8. 相关文件

| 文件 | 说明 |
|---|---|
| `boards/nrf54lm20dk_nrf54lm20a_cpuapp.overlay` | 中断已移除，gpio0/gpiote30 disabled |
| `src/main.c` | 2 s 轮询、电源轨配置、广播/Idle/hibernate 状态机 |
| `zephyr/drivers/mfd/mfd_npm13xx.c`（NCS） | 事件表与 `work_callback()` 自旋逻辑 |
| `zephyr/drivers/sensor/nordic/npm13xx_charger/npm13xx_charger.c`（NCS） | 触发 ADC 采样，产生 ADC 事件 |

---

## 附录 A：nPM1300 MAIN 事件寄存器

| 偏移 | 名称 | 清除寄存器 | MFD 驱动是否处理 |
|---|---|---|---|
| 0x02 | EVENTSADCSET | 0x03 | ❌ |
| 0x06 | EVENTSBCHARGER0SET | 0x07 | ❌ |
| 0x0A | EVENTSBCHARGER1SET | 0x0B | ✅（bit4/5） |
| 0x0E | EVENTSBCHARGER2SET | 0x0F | ✅（bit0/1） |
| 0x12 | EVENTSSHPHLDSET | 0x13 | ✅（bit0/1/3） |
| 0x16 | EVENTSVBUSIN0SET | 0x17 | ⚠️ 仅 bit0/1 |
| 0x1A | EVENTSVBUSIN1SET | 0x1B | ❌ |
| 0x22 | EVENTSGPIOSET | 0x23 | ✅ |

对应中断使能清除寄存器：`0x05, 0x09, 0x0D, 0x11, 0x15, 0x19, 0x1D, 0x25`。

> 实测表明 `INTENEVENTS*` 不能阻断 GPOIRQ，GPOIRQ 反映的是事件锁存位的直接“或”。

## 附录 B：诊断方法（无需示波器）

临时在应用中加入：

1. 注册一个 P0.09 的 GPIO 回调，统计 2 s 内的边沿数；
2. 每 2 s 通过 I2C 读取 8 组 `EVENTS*SET` 与 `INTENEVENTS*SET`；
3. 用 `gpio_pin_get_raw()` 读取 P0.09 原始电平；
4. 通过 RTT 打印，例如：

```
diag irq=0 lvl=1 EV 77 2f 00 00 00 00 00 00
diag EN 00 00 00 00 00 00 00 00 MODE3=05
```

判读：

| 现象 | 结论 |
|---|---|
| `lvl=1` 且 `irq=0` | 电平卡高（不是抖动） |
| `EV` 某组非 0 且长期不变 | 该组事件未被清除，是 GPOIRQ 的源 |
| `EN=0` 但 `lvl=1` | GPOIRQ 不受中断使能控制 |
| `MODE3 != 05` | PMIC GPIO3 未正确配置为 GPOIRQ |
