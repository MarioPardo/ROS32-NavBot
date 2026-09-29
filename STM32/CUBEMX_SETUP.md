# ROS32Bot — STM32F103C8T6 CubeMX Setup Guide

Target: **STM32F103C8T6 (Blue Pill)**, LQFP48, STM32Cube FW_F1 V1.8.7.
Purpose: generate the firmware scaffold for `ros32bot_stm32` that drives 2× L298N
motor channels, reads 2× quadrature encoders, talks to a gyro over I2C2, reports
battery voltage on ADC1, and exposes USART1 for micro-ROS to the Raspberry Pi.

> Verified on this machine against the local CMSIS/HAL package:
> `~/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.7`

---

## 0. Before you start

Confirmed present on this machine:

| Tool | Path / status |
|---|---|
| STM32CubeMX | `~/STM32CubeMX` |
| FW_F1 V1.8.7 | `~/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.7` (already cached) |
| arm-none-eabi-gcc | `~/st/stm32cubeclt_1.20.0/...` (13.3.1) |
| openocd | `/usr/bin/openocd` |

Your previous F103 projects (`BluePillHelloWorld`, `FreeRTOS_Test`) already use
FW_F1 V1.8.7, so use the same package — no download, no migration prompts.

**Two things to fix/decide now (see §9 for details):**

1. **FreeRTOS: decided — you can use it.** It consumes *no* timer peripheral and
   does not disturb PWM. **But** your `FreeRTOS_Test.ioc` used **TIM4 as the HAL
   timebase** (`VP_SYS_VS_tim4`), and that trick cannot be reused — TIM4 is now
   Motor 2's PWM, and this chip has **no TIM6/TIM7** to fall back on. The timebase
   must instead be derived from the FreeRTOS tick. This is a 10-line file, already
   written out in **§9.1**.
2. PB3 and PB4 are **JTAG pins by default**. Motor 2's direction lines will not
   work until you switch the debug interface to SWD. This is the single most
   common way this exact pinout fails.

---

## Scope — two phases

**Phase 1 (this guide's target): FreeRTOS + basic robot I/O. No micro-ROS.**
Drive both motors, read both encoders, read the gyro. Everything on one board,
debugged over a local link.

**Phase 2 (later): add micro-ROS** on top of a working robot.

Battery sensing on ADC1 (§5.6) is not in your Phase 1 list — configure the pin now
(it costs nothing and the pin map should be built in one pass) but treat its
bring-up as optional until the drive train works.

This split is worth respecting, because it dissolves most of the risk in this
design:

| Concern | Phase 1 | Phase 2 |
|---|---|---|
| RAM (20 KB total) | Comfortable — no micro-ROS allocator or transport buffers | Tight; §9.2 applies |
| Peripheral interrupts | **None needed** — no `...FromISR()` calls, so the §9.1 priority rule is dormant | USART1 + DMA1 IRQs required, priority ≥ 5 |
| USART1 | Plain blocking TX for debug, or none at all | micro-ROS transport with DMA |
| Task set | Motor + sensor + debug | Motor + sensor + micro-ROS |

**What to configure now vs. defer.** Build the full pin/peripheral set from §4 in
one pass — the pins do not change between phases, and regenerating later is
cheap. But **defer these two**, because they only matter for micro-ROS and add
moving parts you would have to debug alongside your motor loop:

- **USART1 DMA** (§5.5) — add in Phase 2.
- **USART1/DMA1 NVIC interrupts** (§5.7) — add in Phase 2.

For Phase 1 debugging, prefer **SEGGER RTT** (you already have `SEGGER_RTT.c` in
the FreeRTOS_Test project). RTT uses the SWD probe you are already flashing with,
costs no pins, needs no UART, and — most importantly here — **keeps USART1 free**
so Phase 2 does not have to evict your debug channel. If you would rather not use
RTT, plain blocking USART1 TX at 115200 is fine; just know you will be sharing that
wire with micro-ROS later.

**Phase 1 is done when:** both wheels drive forward/reverse under command, both
encoders report counts whose sign and magnitude track actual wheel motion, and the
gyro's rate readings change plausibly when you rotate the board by hand.

---

## 1. Create the project

1. CubeMX → **File → New Project**.
2. **MCU/MPU Selector** tab → part number `STM32F103C8` → select
   **STM32F103C8Tx** (LQFP48). Do *not* pick the board "Blue Pill" unless it is
   listed with the right revision; the bare MCU is cleaner.
3. **Start Project**.
4. Answer **No** if asked to initialize peripherals with default mode — you want a
   clean slate. (If it defaults things in, you can reset individual pins later.)

**Naming (do this before generating, it is baked into every include guard):**

| Field | Value |
|---|---|
| Project Name | `ros32bot_stm32` |
| Project Location | `/home/mariop/Documents/Programming/ROS32Bot/STM32` |
| Toolchain | `Makefile` (see §6 for why) |

The project will land in `STM32/ros32bot_stm32/`. Keep this guide **outside** that
folder — CubeMX warns when the destination is not empty.

---

## 2. System Core — the two settings that break everything if missed

### 2.1 SYS → Debug: **Serial Wire** ⚠️ critical

**System Core → SYS → Debug: `Serial Wire`**

Why this matters more than usual for you:

| Pin | Default function | Needed for |
|---|---|---|
| PB3 | JTDO | Motor 2 Direction A |
| PB4 | NJTRST | Motor 2 Direction B |
| PA15 | JTDI | (free, unused) |

With the default *JTAG* setting, PB3/PB4 are owned by the debug peripheral and your
direction pins silently do nothing. Selecting **Serial Wire** keeps SWD on
PA13/PA14 (your ST-Link still works) and releases PB3, PB4, PA15 as GPIO.

**Also set:** Timebase Source = `SysTick`. Counter-intuitively this is the correct
choice *because* you are using FreeRTOS: selecting SysTick makes CubeMX generate
no timebase file at all, and §9.1 supplies HAL's tick from the FreeRTOS tick.
**Do not** let CubeMX assign the timebase to TIM1–TIM4 — it would reconfigure one
of your PWM/encoder timers out from under you. If it already did, TIM1 is the
likely victim and **PA8 will refuse to be PWM** — see §9.7.

> **Do this first.** Set SYS → Debug and SYS → Timebase Source *before* enabling
> FreeRTOS and before assigning any timer. CubeMX picks a timebase timer from
> whatever is free at the instant FreeRTOS is enabled, so claiming TIM1–TIM4
> beforehand makes the §9.7 conflict far more likely.

### 2.2 RCC → HSE: **Crystal/Ceramic Resonator**

**System Core → RCC → High Speed Clock (HSE): `Crystal/Ceramic Resonator`**

The Blue Pill has an 8 MHz crystal populated. Leave **LSE disabled** — the Blue
Pill footprint for the 32.768 kHz crystal is normally *not* populated, and enabling
it will hang your clock init waiting for a clock that never starts.

---

## 3. Clock tree → 72 MHz

Go to the **Clock Configuration** tab and set:

| Node | Value |
|---|---|
| Input frequency (HSE) | `8` MHz |
| PLL Source Mux | `HSE` |
| PLL Mul | `x9` |
| SYSCLK Mux | `PLLCLK` |
| **SYSCLK** | **72 MHz** |
| AHB Prescaler | `/1` → HCLK `72 MHz` |
| APB1 Prescaler | `/2` → PCLK1 `36 MHz` |
| APB2 Prescaler | `/1` → PCLK2 `72 MHz` |
| ADC Prescaler | `/6` → ADC clock `12 MHz` |
| USB Prescaler | `/1.5` (unused) |
| Flash Latency | `2` (auto-filled; required above 48 MHz) |

Two facts this produces, both important later:

- **APB1 timer clock = 72 MHz** and **APB2 timer clock = 72 MHz.** APB1 is divided
  to 36 MHz but the timer clock is doubled back to 72 MHz. So TIM1 (APB2) and
  TIM4 (APB1) run at the *same* 72 MHz — your two PWM timers can share identical
  prescaler/ARR values.
- **ADC clock = 12 MHz**, which satisfies the F1 limit of ≤ 14 MHz.

If the ADC Prescaler box is red, reduce it to `/8` (9 MHz) — still fine.

> **Why APB1 refuses `/1`, and why that is correct.** CubeMX highlights the APB1
> prescaler (and PCLK1) in red because on STM32F103 **APB1 is a low-speed bus with a
> hard 36 MHz ceiling**, while APB2 is allowed the full 72 MHz. With SYSCLK at
> 72 MHz, setting APB1 to `/1` would demand PCLK1 = 72 MHz — out of spec, hence the
> red. **`/2` is not a workaround, it is the required value.** Nothing is lost by
> it, because of the timer-clock rule below.

### 3.1 What each clock is, and why these numbers

**The oscillator sources** (CubeMX lists these under RCC; only two matter here):

| Source | What it is | Role in this design |
|---|---|---|
| **HSI** | Internal 8 MHz **RC** oscillator | Runs at reset with no external parts. **Inaccurate** — roughly ±1% at 25 °C and several percent over temperature. Used as the boot clock and safety fallback. Its error is why you must never run a UART off it |
| **HSE** | **External crystal = 8 MHz on the Blue Pill** | Accurate to tens of ppm. **This is the one we use**, and why §2.2 sets Crystal/Resonator. Baud accuracy is the concrete reason — your micro-ROS link in Phase 2 will not hold at 115200+ on HSI |
| **LSI** | Internal ~40 kHz RC | Watchdog (IWDG) clock. Leave alone |
| **LSE** | External 32.768 kHz crystal | RTC. Blue Pill normally does not populate it → **disabled** (§2.2, §9.6) |

**The chain, and what each node feeds:**

1. **PLL** — not an oscillator but a multiplier. It takes **HSE 8 MHz × 9 = 72 MHz**.
   `×9` is chosen because 8 MHz × 9 hits exactly the F103's maximum, 72 MHz.
2. **SYSCLK = 72 MHz** — the root. 72 MHz is the part's ceiling and the best choice
   here: maximum control-loop headroom, and clean timer arithmetic
   (`72 MHz / (PSC+1) / (ARR+1)` = PWM frequency, so `PSC=0, ARR=3599` is exactly
   20 kHz).
3. **AHB prescaler `/1` → HCLK = 72 MHz.** This is the **core, SRAM, flash
   interface, DMA, and the SysTick reference** — i.e. the speed the CPU actually
   executes at. Max is 72 MHz.
4. **APB1 prescaler `/2` → PCLK1 = 36 MHz.** APB1 is the **low-speed** peripheral
   bus (36 MHz max) and hosts **TIM2, TIM3, TIM4**, I2C2, USART2/3, SPI2. Your
   encoders (TIM2/TIM3), Motor 2's PWM (TIM4), and the gyro bus (I2C2) all live
   here — which is exactly why the `36 MHz` figure looks alarming and isn't.
5. **APB2 prescaler `/1` → PCLK2 = 72 MHz.** APB2 is the **high-speed** bus
   (72 MHz max) and hosts **TIM1**, USART1, SPI1, the ADC, and all GPIO ports.
   Motor 1's PWM (TIM1) and the micro-ROS UART (USART1) live here.
6. **The timer-clock doubling rule — the part that makes APB1 `/2` harmless.**
   Peripheral timers do not run at PCLK directly. If the APB prescaler is **`/1`**,
   the timer clock equals PCLK; if it is **`/2` or greater**, the timer clock is
   **PCLK × 2**:
   - APB1 timers (TIM2/TIM3/TIM4): 36 MHz × 2 = **72 MHz**
   - APB2 timers (TIM1): 72 MHz × 1 = **72 MHz** (no doubling, since `/1`)

   **Both land on 72 MHz.** That is the entire reason APB1 being "only" 36 MHz does
   not handicap your motor PWM, and why §5.2 gives TIM1 and TIM4 *identical*
   prescaler/ARR values. Also note the encoder timers get 72 MHz too, which is the
   resolution behind their count rate.
7. **ADC prescaler `/6` → 12 MHz**, derived from PCLK2. The F1 ADC's maximum is
   14 MHz, so 12 MHz is correct. (This divider exists because the ADC's
   sample-and-hold needs a bounded clock, not because of bus bandwidth.)
8. **Flash latency = 2 wait states.** Required above 48 MHz at 3.3 V. CubeMX sets
   it automatically — if it ever shows a value that disagrees, trust CubeMX's
   `SystemClock_Config()`, not a hand-edit.
9. **USB prescaler `/1.5` → 48 MHz.** Irrelevant: this design has no USB. It is
   just a divider tap; leave it as CubeMX shows it.

**In one line:** HSE 8 MHz → PLL ×9 → SYSCLK 72 MHz → HCLK 72 MHz (core) → APB2 72 MHz
fast peripherals / APB1 36 MHz slow peripherals → but *all timers see 72 MHz* → ADC 12 MHz.

---

## 4. Master pin map

Configure these in **Pinout & Configuration**. Set the **User Label** for every
pin (right-click the pin → *Enter User Label*). CubeMX generates
`<label>_Pin` and `<label>_GPIO_Port` defines used by all your firmware code.

| Pin | CubeMX signal | User Label | Notes |
|---|---|---|---|
| PA8 | `TIM1_CH1` | `M1_PWM` | Motor 1 speed |
| PB14 | `GPIO_Output` | `M1_IN1` | Motor 1 direction |
| PB15 | `GPIO_Output` | `M1_IN2` | Motor 1 direction |
| PB6 | `TIM4_CH1` | `M2_PWM` | Motor 2 speed |
| PB3 | `GPIO_Output` | `M2_IN1` | **needs SWD** (§2.1) |
| PB4 | `GPIO_Output` | `M2_IN2` | **needs SWD** (§2.1) |
| PA0 | `TIM2_CH1` | `ENC1_A` | Encoder 1 |
| PA1 | `TIM2_CH2` | `ENC1_B` | Encoder 1 |
| PA6 | `TIM3_CH1` | `ENC2_A` | Encoder 2 |
| PA7 | `TIM3_CH2` | `ENC2_B` | Encoder 2 |
| PB10 | `I2C2_SCL` | `I2C2_SCL` | Gyro bus |
| PB11 | `I2C2_SDA` | `I2C2_SDA` | Gyro bus |
| PA9 | `USART1_TX` | `USART1_TX` | → Pi RX |
| PA10 | `USART1_RX` | `USART1_RX` | ← Pi TX |
| PA4 | `ADC1_IN4` | `VBAT_ADC` | Battery divider |
| PA13 | `SYS_JTMS-SWDIO` | — | auto (keep) |
| PA14 | `SYS_JTCK-SWCLK` | — | auto (keep) |
| PC13 | `GPIO_Output` | `LED` | *optional but recommended* — free, active-LOW, invaluable for bring-up |

> **Deliberately avoided pin conflicts.** Three remaps on this chip would silently
> collide with your motor pins. Do not enable any of them:
> - **USART1 remap** moves TX/RX to **PB6/PB7** → collides with `M2_PWM`.
> - **TIM3 partial remap** moves CH1/CH2 to **PB4/PB5** → collides with `M2_IN2`.
> - **TIM4 remap** overlaps PWM pins.
>
> The default (no remap) mapping is exactly what you want everywhere. CubeMX only
> enables a remap if you tick it; just never tick it.

---

## 5. Per-peripheral configuration

### 5.1 Motor direction GPIOs (PB14, PB15, PB3, PB4)

**System Core → GPIO** — select each pin and set:

| Parameter | Value | Reason |
|---|---|---|
| GPIO output level | `Low` | Motors must not lurch at boot |
| GPIO mode | `Output Push Pull` | L298N logic input |
| GPIO Pull-up/Pull-down | `No pull-up and no pull-down` | L298N inputs are high-impedance |
| Maximum output speed | `Low` | Plain DC logic, not a fast edge |
| User Label | as §4 | — |

**L298N truth table** (IN1/IN2), so the logic is unambiguous:

| IN1 | IN2 | Result |
|---|---|---|
| 0 | 0 | Coast / stop |
| 1 | 0 | Forward |
| 0 | 1 | Reverse |
| 1 | 1 | Brake (both low-side on) — avoid holding this |

The direction pins are **state**, the PWM pin is **magnitude**. Convention that
keeps the math simple: drive `IN1 = (duty > 0)`, `IN2 = (duty < 0)`, and pass
`|duty|` to the PWM compare. When you cross zero, set PWM compare to 0 *before*
flipping the direction pins — flipping direction while PWM is high dumps the full
supply across a reversing motor and browns out your rail.

### 5.2 PWM — TIM1 (CH1) and TIM4 (CH1)

**Timers → TIM1 → Channel1: `PWM Generation CH1`**
**Timers → TIM4 → Channel1: `PWM Generation CH1`**

Identical parameter block for both (both timers are clocked at 72 MHz):

| Parameter | Value |
|---|---|
| Prescaler | `0` |
| Counter Mode | `Up` |
| Counter Period (ARR) | `3599` |
| Auto-reload preload | `Disable` |
| PWM Mode | `PWM mode 1` |
| Pulse (CCR1) | `0` |
| Output compare preload | `Enable` |
| Fast Mode | `Disable` |
| CH Polarity | `High` |

**Resulting PWM frequency = 72 MHz / (0+1) / (3599+1) = 20 kHz.**

Why 20 kHz: it is above the audible band, so the motors do not whine; it is well
below the L298N's switching limit; and 3600 counts of resolution is far more than
you need for a motor. Your old F446 project used ARR=15999 (4.5 kHz) on GPIO/timer
pins that no longer apply — expect to hear a difference.

Two notes specific to TIM1:
- TIM1 is an **advanced-control** timer. It has a Main Output Enable (MOE) gate;
  `HAL_TIM_PWM_Start()` handles MOE for you, so no extra work — just be aware that
  twiddling CR2/BDTR manually can silently kill your output.
- Ignore TIM1's complementary/break pins (PB13 = CH1N, PB12 = BKIN). You are using
  a single-ended L298N, so they stay unassigned.

> ⚠️ **If PA8 offers no PWM option, TIM1 is taken by the HAL timebase.** This is the
> single most likely thing to go wrong while configuring this pin, and it is not
> obvious because the culprit is a *different* panel. Enabling FreeRTOS makes
> CubeMX claim a hardware timer for `SYS → Timebase Source`; if it picked **TIM1**,
> then TIM1 is reserved for the 1 ms tick and `PA8 / TIM1_CH1` disappears from the
> pin's signal list — while PB6/TIM4, TIM2 and TIM3 still happily offer PWM. Fix:
> **System Core → SYS → Timebase Source = `SysTick`**, which releases TIM1. See
> §9.7.

### 5.3 Encoders — TIM2 and TIM3

**Timers → TIM2 → Combined Channels: `Encoder Mode`**
**Timers → TIM3 → Combined Channels: `Encoder Mode`**

Parameter block for both:

| Parameter | Value |
|---|---|
| Encoder Mode | `Encoder Mode TI1 and TI2` |
| Prescaler | `0` |
| Counter Period (ARR) | `65535` |
| Counter Mode | `Up` |
| Auto-reload preload | `Disable` |
| IC1/IC2 Polarity | `Rising Edge` |
| IC1/IC2 Filter | `10` |
| IC1/IC2 Prescaler | `No division` |

Notes:

- **ARR = 65535** (full 16-bit range) so the counter free-runs and you read
  position directly. On TI12 mode the hardware counts **4× per encoder line**
  (both edges of both channels), so one motor-shaft revolution yields
  `4 × lines × gear_ratio` counts. That 4× is why the filter matters.
- **Filter = 10** debounces the inputs in hardware. Brush noise from the motors
  *will* corrupt counts without it. If counts still look noisy, raise toward 15.
- TIM2 and TIM3 are **16-bit** counters on this chip. They wrap at 65535 and
  count downward for reverse. Handle the wrap in software by taking a **delta
  between samples** as a signed 16-bit difference rather than reading absolute
  position — that is what your F446 `encoder_position` / `encoder_velocity` pair
  already did, and the approach ports directly.
- **Pull-ups:** in the **GPIO** view, set PA0/PA1/PA6/PA7 to
  `Pull-up` if your encoders have **open-collector** outputs (common on optical
  and Hall modules). For push-pull outputs leave them at `No pull-up`. Encoder
  channels are the one place where a wrong pull setting shows up as flaky counts.

### 5.4 I2C2 — gyro bus (PB10/PB11)

**Connectivity → I2C2 → I2C**

| Parameter | Value |
|---|---|
| I2C Speed Mode | `Fast Mode` |
| I2C Clock Speed | `400000` Hz |
| Clock No Stretch Mode | `Disabled` |
| Primary Address Length | `7-bit` |
| Dual Address Acknowledged | `Disabled` |
| Analog Filter | `Enabled` |
| Digital Filter | `0` |

- I2C2 lives on APB1 (36 MHz) — CubeMX computes `CCR`/`TRISE` for you. Do not
  hand-edit those registers.
- **Pull-ups are external.** The internal ~40 kΩ pull-ups are far too weak for
  400 kHz. Most MPU-6050/MPU-9250 breakouts include 4.7 kΩ pull-ups on board; if
  yours does not, add them to SDA and SCL.
- With PB6 taken by `M2_PWM`, **I2C2 is your only usable I2C bus** — so put every
  I2C device (gyro and any future device) on PB10/PB11 and keep them at distinct
  addresses.

### 5.5 USART1 — micro-ROS link (PA9/PA10)

**Connectivity → USART1 → Mode: `Asynchronous`**

| Parameter | Value |
|---|---|
| Baud Rate | `115200` (raise to `921600` once the link is proven) |
| Word Length | `8 Bits` |
| Parity | `None` |
| Stop Bits | `1` |
| Data Direction | `Receive and Transmit` |
| Hardware Flow Control | `None` |
| Oversampling | `16 Samples` |

> **Phase 1 note — defer this.** Leave USART1 as a plain asynchronous port with no
> DMA and no interrupts. Configure it now only if you intend to use it as your
> debug channel; otherwise leave it for Phase 2. Adding DMA here is harmless but
> pointless until micro-ROS exists, and every extra moving part is something you
> would be debugging alongside your motor loop instead of after it.

**Phase 2 only** — in **USART1 → DMA Settings**, add both directions, because
micro-ROS needs them:

| DMA Request | Channel | Direction | Mode | Data width |
|---|---|---|---|---|
| `USART1_TX` | `DMA1 Channel 4` | Memory → Peripheral | Normal | Byte / Byte |
| `USART1_RX` | `DMA1 Channel 5` | Peripheral → Memory | Circular | Byte / Byte |

Set baud to 115200 to start. Higher baud is desirable for micro-ROS throughput,
but only after you have proven byte-exact communication — a misconfigured clock
tree produces garbage at high baud while appearing to work at low baud.

### 5.6 ADC1 — battery voltage (PA4)

> **VBAT is a different thing — do not confuse the two.** The LQFP48 **VBAT pin**
> (pin 1) is a *power supply* pin for the backup domain: it keeps the RTC and backup
> registers alive when VDD is absent. On the Blue Pill it is tied to VDD. **Leaving
> it as-is is exactly right**, and there is nothing to configure for it.
>
> It is **not** where your motor battery is measured. Two separate reasons:
> - **On STM32F1 the ADC has no VBAT channel.** Verified in the local HAL header:
>   `stm32f1xx_hal_adc.h` defines internal channels only for `ADC_CHANNEL_16`
>   (temperature sensor) and `ADC_CHANNEL_17` (VREFINT). A search for
>   `ADC_CHANNEL_VBAT` returns **zero** matches. (That channel exists on F3/F4/L4
>   parts — not here.)
> - The divider on **PA4 / ADC1_IN4** is therefore the *only* way to read pack
>   voltage on this chip.

**Analog → ADC1**

Enable `IN4` (that is PA4), then:

| Parameter | Value |
|---|---|
| Mode | `Independent` |
| Data Alignment | `Right` |
| Scan Conversion Mode | `Enabled` (only if you also add VREFINT, see below) |
| Continuous Conversion Mode | `Enabled` |
| DMA Continuous Requests | `Enabled` |
| Number of Conversions | `1` (or 2 with VREFINT) |
| Rank 1 Channel | `ADC_CHANNEL_4` |
| Rank 1 Sampling Time | `239.5 Cycles` |

Add **DMA**: `ADC1` → `DMA1 Channel 1`, Circular, Peripheral → Memory,
Half-Word / Half-Word.

Long sampling time matters here: your divider is high-impedance, and the F1 ADC
sample-and-hold needs time to charge. `239.5 cycles` is the maximum and is the
right choice for a battery divider.

**Recommended: add VREFINT as a second rank.** The F1 has an internal reference
channel (`ADC_CHANNEL_VREFINT`, channel 17) whose factory calibration value is
stored at a fixed address. Reading it lets you compute the *actual* VDDA instead
of assuming 3.3 V, which makes your battery reading accurate even when running
from USB. Worth the second channel given you are using ADC for a safety-relevant
measurement.

**Divider design.** With `R1` from VBAT to the pin and `R2` pin to GND:

```
V_pin = VBAT × R2 / (R1 + R2)
VBAT  = V_pin × (R1 + R2) / R2
```

For a 12 V max pack: `R1 = 10 kΩ`, `R2 = 3.3 kΩ` → ratio ≈ 4.03, full scale
≈ 13.3 V, and V_pin = 2.97 V at 12 V — inside the 3.3 V limit with margin.
Put a **100 nF capacitor across R2** to stiffen the node against motor noise.

**For a 1S Li-Po (3.0–4.2 V) charged by an MCP73871:** `R1 = 10 kΩ`,
`R2 = 27 kΩ` → ratio 3.7, V_pin = 3.06 V at 4.2 V, ~114 µA quiescent drain.
Note the smaller ratio than the 12 V case — a 1S cell's *upper* end is close to
3.3 V, so the divider is doing much less work and the margin is thinner.

> ⚠️ **Never connect a 1S Li-Po node straight to PA4.** A charged cell sits at
> **4.2 V**, well above VDDA (3.3 V). The pin's protection diode conducts, the
> reading clips, and the ADC input can be damaged. The divider above is not
> optional.

**MCP73871 specifics — which node to measure.** The MCP73871 is a 1S Li-Po charger
with **power-path / load-sharing**: with USB present the system runs from USB while
the cell charges; without USB it runs from the cell. That gives you two different
"+" nodes, and they are not interchangeable:

| Node | What it is | Use for battery monitoring? |
|---|---|---|
| `VBAT` / `BAT` | The actual battery terminal | ✅ **Yes — measure this** |
| `OUT` / `SYS` / `LOAD` | System rail, held up by USB when present | ❌ Misleading — reads the input supply while charging |

Measuring `OUT` tells you what the system rail is doing, **not** how full the
battery is: with USB plugged in it can read well above the cell voltage. Measure
`BAT` through the divider to get true state of charge.

Two further checks on that board:

- **Common ground.** The charger's GND, the Blue Pill's GND, and the divider's
  bottom must be one node. A power-path board with a floating reference is the
  classic cause of a nonsense ADC reading.
- **Check whether `OUT` can power the Blue Pill at all.** `OUT` follows the cell
  (3.0–4.2 V), **and 4.2 V exceeds the STM32F103's 3.6 V maximum VDD.** If your
  board has no 3.3 V regulator between `OUT` and the MCU, you need to add one —
  feeding a charged cell straight into VDD is out of spec. Verify the board's
  actual output voltage with a meter at full charge before connecting it.

*Optional:* the MCP73871's `STAT1`/`STAT2` pins are open-drain charge-status
outputs. If you later want charge state in ROS, wire them to spare GPIOs with
pull-ups — they are not needed for the PA4 measurement.

> ⚠️ **PA4 must never exceed 3.3 V.** The encoder pins may tolerate 5 V logic, but
> PA4 is your analog input: anything above VDDA forward-biases the pin's
> protection diode and can damage the ADC. Size the divider for your *maximum*
> pack voltage (including charger voltage if you charge in-place), not nominal.

### 5.7 NVIC

> **Phase 1 note — defer this.** You need no peripheral interrupts in Phase 1 at
> all: motors are writes, encoders are register reads, and the gyro can be polled
> over blocking I2C. A Phase 1 build with zero NVIC peripheral interrupts is
> simpler and has fewer failure modes. Enable the entries below when micro-ROS
> arrives.

**Phase 2 only** — in **System Core → NVIC**, enable (you will be using FreeRTOS
with ISR notifications, so the priority rule below applies to all three):

| Interrupt | Why |
|---|---|
| `USART1 global interrupt` | micro-ROS serial transport errors/idle detection |
| `DMA1 Channel 4 global interrupt` | TX-complete bookkeeping |
| `DMA1 Channel 5 global interrupt` | RX-complete / framing errors |

Leave the **preemption priority at 0** only if the ISR never calls a FreeRTOS API.
You *are* using FreeRTOS, and the micro-ROS UART/DMA transport does notify from
its ISRs, so set all three to **5 or higher (numerically)** — see §9.1 for the
full rule. `HAL_Init()` calls `HAL_InitTick()` *before* `MX_FREERTOS_Init()`, so
CubeMX will not catch a violation for you; it manifests at runtime as a hard fault
inside `vPortValidateInterruptPriority()`.

### 5.8 Middleware — FreeRTOS (CMSIS-RTOS v2)

**Middleware → FREERTOS → Interface: `CMSIS_V2`** (matches your existing
FreeRTOS_Test project, which is why §9.1's timebase file uses the `osKernel*` API).

Config tab:

| Parameter | Value | Why |
|---|---|---|
| `configTICK_RATE_HZ` | `1000` | 1 ms tick — and this is what makes HAL's tick millisecond-accurate in §9.1 |
| `configUSE_PREEMPTION` | `Enabled` | Standard |
| `configCHECK_FOR_STACK_OVERFLOW` | `2` | Bring-up safety net on a 20 KB part |
| `configUSE_NEWLIB_REENTRANT` | `Disabled` unless several tasks use newlib | ~1 KB/task otherwise — see §9.2 |
| `configTOTAL_HEAP_SIZE` | Set explicitly, verify in the `.map` | CubeMX defaults can exceed available RAM |
| `USE_MUTEXES` | `Enabled` | Only if you guard shared motor state (§9.1) |

The `configTICK_RATE_HZ` setting is load-bearing: §9.1 hands HAL's millisecond
counter to the FreeRTOS tick, so if the tick rate is not 1000 Hz, every HAL
timeout silently changes scale.

Tasks tab: define tasks here so CubeMX generates skeletons in `freertos.c`
(inside protected `USER CODE` blocks). A sensible starting set — deliberately
small, because each task costs stack + TCB:

- **`motorCtrl`** — fixed period via `osDelayUntil()`; owns *all* PWM writes,
  direction writes, and encoder reads.
- **`microROS`** — owns the executor and the UART transport; never touches motors.

Keep the task count as low as you can justify; every task is stack you do not have.

**Phase 1 task set** — three tasks, and the priority *ordering* is the important
part:

| Task | Priority | Period | Owns |
|---|---|---|---|
| `motorCtrl` | `osPriorityAboveNormal` | 100 Hz (10 ms), `osDelayUntil()` | PWM writes, direction writes, encoder reads, PID |
| `gyroTask` | `osPriorityNormal` | 100–200 Hz | I2C2 gyro reads |
| `debugTask` | `osPriorityLow` | 2–10 Hz | RTT or USART telemetry |

**Why the ordering matters — this is the one real FreeRTOS hazard in Phase 1.**
`HAL_I2C_Mem_Read()` is a **busy-wait** transaction: 14 bytes at 400 kHz is roughly
400 µs of spinning. If that read lived inside `motorCtrl`, it would add ~400 µs of
jitter to your control period and directly corrupt the encoder-velocity estimate.
Putting the gyro in a *lower*-priority task means `motorCtrl` preempts it, and the
I2C wait only ever delays low-priority work. So: **never put I2C — or any other
blocking HAL call — inside the control task.**

Start `motorCtrl` at **100 Hz, not 1 kHz.** Encoder velocity is counts-per-period,
so a longer period means more counts per sample and much better velocity
resolution at low wheel speed. 10 ms is a good starting point for a hobby drivetrain
and you can raise it later if the control loop feels sluggish.

> If gyro timing later proves too jittery, move I2C2 to interrupt/DMA mode with a
> semaphore so `gyroTask` blocks instead of spinning. Do not do this until Phase 1
> works — it is an optimization, not a prerequisite.

Phase 2 adds the `microROS` task. Only then does the §9.1 NVIC priority rule become
live, because nothing in Phase 1 calls a `...FromISR()` API. Another reason to keep
the phases separate.

After generating, apply §9.1 (the RTOS timebase file).

---

## 6. Project Manager

**Project Manager → Project**

| Setting | Value | Why |
|---|---|---|
| Toolchain / IDE | `Makefile` | See below |
| Firmware Package | `STM32Cube FW_F1 V1.8.7` | Matches your other F103 projects |
| Minimum Heap Size | `0x200` | Default is fine |
| Minimum Stack Size | `0x400` | Give the ISRs headroom (default 0x200 is tight once micro-ROS is in) |

**Why Makefile instead of CubeIDE:** micro-ROS's official STM32 flow
(`micro_ros_stm32cubemx_utils`) expects a Makefile project, and your `SETUP.md`
already lists the open-source CLI route as the target. A Makefile project builds
with the `arm-none-eabi-gcc` you already have and flashes with the `openocd` you
already have — no IDE in the loop. Your existing `Debug/makefile` projects prove
you are already building from the command line anyway.

**Project Manager → Code Generator**

| Setting | Value |
|---|---|
| Copy only the necessary library files | ✅ enabled (keeps `Drivers/` small) |
| Generate peripheral initialization as a pair of `.c/.h` files per peripheral | ✅ enabled |
| Keep User Code when re-generating | ✅ enabled |
| Delete previously generated files when not re-generated | ✅ enabled |
| Set all free pins as analog | ❌ disabled (you want to see real pin states) |

Enabling per-peripheral `.c/.h` files means TIM1/TIM4 land in `tim.c`, I2C2 in
`i2c.c`, and so on, instead of one giant `main.c`. Worth it here — this project
will accumulate a lot of peripheral code.

**Project Manager → Advanced Settings:** leave `HAL` driver assignment at default,
but ensure **`ADC` and `TIM` are set to `HAL`** (not LL) for consistency.

---

## 7. Generate and verify

**Project → Generate Code.**

Then verify these three things before writing any firmware — each catches a
specific failure mode:

1. **PB3/PB4 were really released.** Open `Core/Src/stm32f1xx_hal_msp.c` and
   confirm you see `__HAL_AFIO_REMAP_SWJ_NOJTAG();` in the `HAL_MspInit()` body.
   If it is missing, §2.1 was not applied and your direction pins are dead.

2. **Every label exists.** In `Core/Inc/main.h`, confirm defines for
   `MOT1_DIR_A_Pin`, `MOT1_DIR_B_Pin`, `MOT2_DIR_A_Pin`, `MOT2_DIR_B_Pin`,
   `DEBUG_LED_Pin`, plus `_GPIO_Port` counterparts. A typo'd label is a compile
   error much later.
   > **Correction:** do *not* expect `M1_PWM_Pin` / `M2_PWM_Pin` macros. CubeMX
   > only generates `_Pin`/`_GPIO_Port` defines for pins configured as **GPIO**.
   > Timer-channel pins (PA8, PB6) are alternate-function pins and get no macros —
   > you address them through the timer handle instead
   > (`__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr)`). This matches your
   > existing F446 `driveMotor()`, which used `MotorIN1_Pin` for direction but
   > `htim2 / TIM_CHANNEL_3` for PWM.

3. **The clock is actually 72 MHz.** In `Core/Src/main.c` and the `.ioc`, check:
   `PLLMUL = RCC_PLL_MUL9`, `SYSCLKSource = PLLCLK`, `AHBCLKDivider = DIV1`,
   **`APB1CLKDivider = DIV2`, `APB2CLKDivider = DIV1`**, and `ADCFreqValue`
   ≈ **12 MHz**. See §9.8 — an APB2 divider left at `/2` silently overclocks the
   ADC past its 14 MHz limit.

**Then build it** (from `STM32/ros32bot_stm32/`):

```bash
make -j$(nproc)
```

---

## 8. Bring-up order

Follow your established "test small and build up" methodology. Order matters —
each step validates a prerequisite of the next:

| # | Test | Pass condition |
|---|---|---|
| 1 | Clock + GPIO: blink PC13, print banner on USART1 | Correct baud → proves HSE/PLL/72 MHz. Wrong baud → clock tree bug, fix before anything else |
| 2 | PWM output on M1_PWM and M2_PWM | Measure with meter/LED; 50% duty ≈ 1.65 V avg. Proves timer clocks |
| 3 | Direction pins + motor, open loop | Wheel spins, reverses, stops. **This is the PB3/PB4 (SWD) test** |
| 4 | Encoders, motors disconnected | Spin wheel by hand: count changes, wraps cleanly, and *sign matches direction* |
| 5 | ADC battery | Reading within ~2% of a multimeter on the pack |
| 6 | I2C2 bus scan | Gyro address ACKs (commonly `0x68` for MPU-6050) |
| 7 | micro-ROS ping/publisher to the Pi | A topic appears via `ros2 topic list` |

Do not skip step 1, and do not run step 3 before step 3's direction-pin check in
isolation — a wiring error on an H-bridge is much easier to diagnose with the
motor supply current-limited or disconnected.

**Steps 1–6 are Phase 1** and should be completed and stable before you touch
step 7. Step 3 is the PB3/PB4/SWD check; step 4 is where the TIM input filter
(§5.3) earns its place.

---

## 9. Verified constraints and gotchas

### 9.1 FreeRTOS + two PWM motors: yes, with one timebase fix

**Short answer: yes, you can.** Two different things get conflated here, so keep
them separate:

| Need | Provided by | Competes with TIM1–TIM4? |
|---|---|---|
| Motor PWM waveform | TIM1 / TIM4 hardware | — |
| RTOS scheduler tick | **SysTick** (a core timer, not a peripheral timer) | **No** |
| HAL 1 ms tick (`HAL_GetTick`) | must come from *somewhere* | **This is the only problem** |

- **PWM needs no CPU and no RTOS cooperation.** Once `HAL_TIM_PWM_Start()` is
  called, the timer generates the waveform autonomously in silicon. A FreeRTOS
  context switch cannot glitch it, because nothing in the preemption path touches
  TIM1 or TIM4. Both motors keep running while tasks block, yield, and preempt.
- **FreeRTOS uses none of TIM1–TIM4.** On Cortex-M it uses the core's SysTick plus
  PendSV/SVC. So "all four timers allocated" and "FreeRTOS" do not compete.

**The real problem: HAL also wants a 1 ms tick** — for `HAL_GetTick()`,
`HAL_Delay()`, and every HAL timeout (UART, I2C, ADC). By default HAL takes
SysTick, and FreeRTOS wants SysTick, so ST's convention is to move HAL's timebase
onto a spare timer. That is exactly why CubeMX generated
`Core/Src/stm32f1xx_hal_timebase_tim.c` in your FreeRTOS_Test project — open it
and you will see `HAL_InitTick()` seizing **TIM4**.

Verified in the local CMSIS header
(`.../STM32F1xx/Include/stm32f103xb.h`): the only timer bases defined are
**TIM1, TIM2, TIM3, TIM4**; a grep for `TIM6`/`TIM7` returns **zero** matches.
So there is no spare timer. All four are committed, and the encoder pair cannot
double as a timebase either — in encoder mode the counter *is* the position, so
it is not available to count milliseconds.

**The fix: derive HAL's tick from the FreeRTOS tick instead of a peripheral.**
`HAL_GetTick` is declared `__weak` in `stm32f1xx_hal.c` (verified locally, line
304), so overriding the timebase functions is a **supported extension point** —
the same one ST used for the timer version, just a different implementation.

Delete `Core/Src/stm32f1xx_hal_timebase_tim.c` and create
`Core/Src/stm32f1xx_hal_timebase_rtos.c`:

```c
/* HAL time base derived from the FreeRTOS tick.
 * Replaces stm32f1xx_hal_timebase_tim.c, which claimed TIM4: there is no spare
 * timer on STM32F103C8 (TIM1..TIM4 are all committed to PWM/encoders).
 * SysTick belongs to FreeRTOS; HAL's millisecond counter rides on it. */
#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  (void)TickPriority;   /* SysTick is configured by FreeRTOS, not by HAL */
  return HAL_OK;
}

uint32_t HAL_GetTick(void)
{
  if (osKernelGetState() == osKernelInactive)
  {
    return 0U;          /* before osKernelStart(): no RTOS tick exists yet */
  }
  return (uint32_t)osKernelGetTickCount();
}

void HAL_SuspendTick(void) { }   /* nothing to suspend */
void HAL_ResumeTick(void) { }
```

Then three checks:

1. In CubeMX, **SYS → Timebase Source = `SysTick`**, so it generates no
   timer-based timebase file.
2. **Confirm `stm32f1xx_hal_timebase_tim.c` is gone.** Leaving it in place defines
   a second `HAL_InitTick` → duplicate symbol at link time (or, worse, the linker
   silently picks the TIM4 one and your PWM dies).
3. **Confirm `Core/Src/stm32f1xx_it.c` has no `SysTick_Handler`** — FreeRTOS's
   `cmsis_os2.c` owns it. Your existing FreeRTOS_Test already satisfies this:
   `SysTick_Handler` appears only in `startup_stm32f103c8tx.s` and `cmsis_os2.c`,
   never in `it.c`. That is the correct pattern; preserve it.

**Two consequences to internalize:**

- **`HAL_Delay()` before `osKernelStart()` is a hang.** The scheduler is not
  running, so `HAL_GetTick()` returns 0 forever and `HAL_Delay()` never exits.
  Do `HAL_Delay()`-dependent driver init *inside* a task, or use `osDelay()` once
  the kernel is up.
- **HAL timeouts now resolve at 1 ms granularity** — fine for UART/I2C. But HAL
  *polling* functions called from a task busy-wait without yielding. Prefer the
  interrupt/DMA-backed HAL APIs from tasks, and never call `HAL_Delay()` inside a
  task — use `osDelay()` so other tasks can run.

**FreeRTOS-specific correctness, and this one bites silently:** any ISR that calls
a FreeRTOS `...FromISR()` API — which the micro-ROS UART/DMA transport does — must
run at a priority numerically **≥** `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`
(CubeMX default **5**). CubeMX defaults peripheral IRQ priorities to **0**, which
violates this and lands you in a hard fault inside
`vPortValidateInterruptPriority()`. In the **NVIC** panel set USART1 and both DMA1
channel IRQs to priority 5 or higher (numerically), and leave SysTick/PendSV at
the lowest priority CubeMX assigns.

**Actuation structure that keeps the H-bridge safe under preemption:**

- Keep all writes to a given motor in **one** task. `__HAL_TIM_SET_COMPARE()` is a
  single 16-bit store and is atomic, but the two direction pins are two separate
  stores — if two tasks drive the same motor, you can momentarily command IN1 and
  IN2 both high (brake) or invert direction under load.
- Use `osDelayUntil()` / `vTaskDelayUntil()` for the control loop, **not**
  `osDelay()`. Encoder velocity is computed from counts per unit time, so period
  drift directly injects error into your velocity estimate. Fixed-period scheduling
  is a correctness requirement here, not a style preference.
- Compute the encoder delta and the PID in the control task, and let the RTOS
  handle the timing. Never put the PID in an ISR.

### 9.2 micro-ROS will be RAM-tight

> **Phase note:** this section is primarily about Phase 2. For Phase 1 — FreeRTOS,
> two motors, two encoders, a gyro — the 20 KB part is comfortable and you have
> real headroom. The FreeRTOS stack/heap bullets further down still apply, but the
> micro-ROS allocator pressure does not arrive until you add the agent link.

The F103C8T6 has **64 KB flash / 20 KB RAM**. micro-ROS's default static
allocator plus transport buffers can consume a large fraction of that. Plan for:

- `-Os` / "Optimize for size" in the build flags.
- Reducing the micro-ROS static memory pool and the `RMW_UXRCE_MAX_*` session/
  subscription limits to the minimum your topic set needs.
- Keeping message types small and avoiding large fixed-size arrays in your
  control messages — a `geometry_msgs/Twist` is fine; a `sensor_msgs/PointCloud2`
  is not.
- Reserving the DMA buffers and checking the `.map` file's RAM report after
  linking `libmicroros`. If `.bss` + `.data` approaches 20 KB during link, that is
  your ceiling, not a bug.

If you hit that ceiling, a pin-compatible upgrade to an **STM32F103CBT6**
(128 KB flash / 20 KB RAM) or a move to the Nucleo-F446RE you already have solves
it.

**FreeRTOS makes the RAM budget tighter, so budget it deliberately.** Your
`FreeRTOS_Test.ioc` shows two settings worth re-examining now that micro-ROS is
also in the picture:

- **`configUSE_NEWLIB_REENTRANT=1`** — this gives *every* task its own `struct
  _reent`, which is on the order of 1 KB per task on this part. On a 20 KB chip
  with several tasks that is a large tax. It is only needed if multiple tasks call
  newlib functions (`printf`, `malloc`, `sprintf`). You already have SEGGER RTT
  (`SEGGER_RTT.c`) in that project — RTT avoids newlib entirely. If you can keep
  newlib usage in a single task, turn this off.
- **Task stacks** — `defaultTask,24,128` is a 128-word (512-byte) stack. That is
  fine for a bare blinky but **not** for a task calling micro-ROS or `printf`,
  which will silently overflow into another task's memory. Give the micro-ROS
  task 2048+ bytes and enable `configCHECK_FOR_STACK_OVERFLOW=2` during bring-up
  so a too-small stack becomes a `vApplicationStackOverflowHook()` hit rather than
  mysterious corruption.
- Set `configTOTAL_HEAP_SIZE` explicitly and watch it in the `.map`. CubeMX
  defaults can silently exceed the part's RAM and only fail at runtime.

### 9.3 Logical-3.3 V vs. L298N

The Blue Pill drives 3.3 V logic; the L298N is a 5 V TTL part with `V_IH` ≈ 2.3 V,
so 3.3 V is read as a clean HIGH. **This works, but only with a common ground** —
tie the Blue Pill GND, the L298N GND, and the battery negative together. A missing
common ground is the classic cause of "the direction pin does nothing."

Two hardware points on the L298N itself:

- **Remove the ENA/ENB jumpers** on the L298N board. If they stay in place, the
  enable pins are tied to 5 V and your PWM is ignored — the motor just runs at
  full speed.
- The L298N board's onboard 5 V regulator: if you power the board from >12 V, do
  not also feed its 5 V output into the Blue Pill while the Blue Pill is USB-
  connected. Power the Blue Pill from one source at a time during development.

### 9.4 Gyro vs. encoder noise

Motors are electrically noisy, and this board puts brushed-motor PWM on PA8/PB6
within a few millimeters of encoder inputs and the I2C bus. Expect to fight this:

- Keep encoder wiring physically away from motor leads; use twisted pairs.
- The TIM input filter (§5.3) is your first line of defense — use it.
- I2C is the most fragile link: a 400 kHz bus next to switching motor leads will
  throw errors. If the gyro misbehaves during motor operation but is fine at
  idle, that is your diagnosis, and the fix is wiring/shielding, not firmware.
- Consider a ferrite bead or bulk capacitance on the motor supply.

### 9.5 Things that are fine as-is

- **PA0 is WKUP.** It is your encoder input and works normally; the WKUP function
  only matters if you use STOP mode for wake-on-pin, which this design does not.
- **PC13 LED** is active-LOW and free — use it for bring-up, but it is on the
  backup domain's power and is not suitable for fast toggling.
- **PB2 (BOOT1)** must stay LOW for normal flash boot; do not repurpose it.
- No analog filter/OPAMP peripherals exist on F1, so there is nothing extra to
  configure for the ADC.

### 9.6 Is your RTC module useful here? Not for this — but your instinct was right

You suspected an RTC module is not the same kind of clock, and that is correct.

**Why it cannot serve as the timebase:** HAL needs a **1 kHz** (1 ms) periodic
interrupt. An RTC module's programmable square-wave output offers 1 Hz, 1.024 kHz,
4.096 kHz, 8.192 kHz, or 32.768 kHz — and even if you picked one, you would still
need a hardware counter to divide it down to 1 ms, which is precisely the spare
timer you do not have. Separately, the STM32F1's own RTC peripheral is a
*calendar/timekeeping* counter: it can raise a 1 Hz second-interrupt or an overflow
interrupt, not a millisecond tick. An RTC is a calendar, not a scheduler timebase.

**And you probably do not need an RTC at all.** micro-ROS performs time
synchronisation with the agent on the Pi (`rmw_uros_sync_session()`), so
authoritative ROS 2 timestamps come from the Pi over the serial link. That is the
right source of truth for a robot whose clock matters mainly for correlating sensor
data — and it costs no extra hardware.

**The one legitimate use, if you want it later:** an external 32.768 kHz source
*can* drive the STM32's own RTC. Wire the module's 32.768 kHz output to
**PC14-OSC32_IN** and set `RCC → LSE` to **Bypass Clock Source** instead of
Crystal/Resonator (this also sidesteps the Blue Pill's unpopulated LSE crystal
footprint mentioned in §2.2). That gives the STM32 wall-clock time that survives
power loss. It is a timekeeping feature, not a fix for anything in this guide —
file it under "later, if I want local timestamps when the Pi is off."

### 9.7 Troubleshooting: PA8 shows a yellow triangle and refuses PWM

**Symptom:** PA8 is assigned to TIM1_CH1 but shows a **yellow triangle**, offers no
PWM mode, and TIM1 does not appear as a configured peripheral — while
**PB6/TIM4_CH1, TIM2 and TIM3 all offer PWM normally.**

There are two distinct causes. **Cause A is the one that actually occurred in
`ros32bot_stm32.ioc`** — start there.

#### Cause A: the channel mode was never set (most likely)

Assigning a *signal* to a pin and choosing a *channel mode* are two separate
actions in CubeMX, and doing only the first produces exactly this yellow triangle.

Open `STM32/ros32bot_stm32/ros32bot_stm32.ioc` and compare these two lines:

```
SH.S_TIM4_CH1.0=TIM4_CH1,PWM Generation1 CH1     <- correct: mode present
SH.S_TIM1_CH1.0=TIM1_CH1                         <- broken: mode MISSING
```

Three independent markers confirm TIM1 was never instantiated:

- The `SH.S_TIM1_CH1.0` entry has **no mode string** after the comma (compare TIM4).
- **TIM1 is absent from the `Mcu.IP*` list.** That list contains `TIM2`, `TIM3`,
  `TIM4` — and no `TIM1`. An enabled timer always appears there.
- There is **no `TIM1.*` parameter block** at all (contrast
  `TIM4.IPParameters=Channel-PWM Generation1 CH1`).

**Cause:** the TIM1_CH1 signal was picked from the **pin popup** on PA8, which
assigns the pin but leaves the channel unconfigured.

**Fix — use the Timers panel, not the pin:** **Categories → Timers → TIM1 →
`Channel1` → `PWM Generation CH1`.** Then set the §5.2 parameters
(`PSC=0`, `ARR=3599`, PWM mode 1, pulse 0, OC preload enable, polarity High).

After that the `.ioc` should gain `Mcu.IP9=TIM1`, a `TIM1.IPParameters=...` line,
and `MX_TIM1_Init` in `ProjectManager.functionlistsort` — and the triangle clears.

##### If `PWM Generation CH1` still isn't offered in the TIM1 Channel1 dropdown

**First, it is not a device limitation — verified in CubeMX's own database.** On
this machine:

- `~/STM32CubeMX/db/mcu/STM32F103C(8-B)Tx.xml` line 187 assigns
  `<Signal Name="TIM1_CH1"/>` to PA8. The pin genuinely supports it.
- All four timers share **the same IP definition**
  (`Name="TIM1_8F1"`, `Version="gptimer2_v1_x_Cube"`), so TIM1 has the *identical*
  mode set to TIM2/TIM3/TIM4. If TIM4 offers PWM, TIM1 offers PWM.
- `~/STM32CubeMX/db/mcu/IP/TIM1_8F1-gptimer2_v1_x_Cube_Modes.xml` line 2798 defines
  `<Mode Name="PWM Generation1 CH1" UserName="PWM Generation CH1">`, requiring only
  `<Signal Name="CH1" Direction="Output"/>` plus
  `!(ClockTriggerSourceOnTI1) & !(XOR_Activated)` — all satisfied by default.

So the mode exists and is selectable; what blocks it is the **half-assigned project
state**. `PA8.Signal=S_TIM1_CH1` exists with **no mode**, which puts the channel in
an ambiguous direction (output modes are filtered against
`Direction="Output"`). That half-state is also what draws the yellow triangle.

**Fix — clear the half-state and redo it from the Timers panel:**

1. In the pinout view, **right-click PA8 → `Reset_State`** to remove the orphaned
   TIM1_CH1 signal assignment entirely. It should return to a plain pin.
2. Go to **Categories → Timers → TIM1**, and in the **Mode** pane set
   **Channel1 = `PWM Generation CH1`**. CubeMX will re-assign PA8 itself.
3. Then apply §5.2's parameters in the **Configuration → Parameter Settings** pane.

**The pane distinction is the other common trap:** the mode dropdown lives in the
upper **Mode** pane. The lower **Configuration → Parameter Settings** pane shows
only per-channel *values* (Pulse, Polarity, Preload) and contains **no** PWM option —
looking for "PWM" there will always come up empty.

**Last resort — encode it directly in the `.ioc`.** If the GUI still refuses, the
correct encoding is known exactly, by analogy with the working TIM4 block already in
your file. Close CubeMX and add:

```
Mcu.IP9=TIM1                  <- renumber Mcu.IPNb=10
SH.S_TIM1_CH1.0=TIM1_CH1,PWM Generation1 CH1     <- append the mode
TIM1.Channel-PWM\ Generation1\ CH1=TIM_CHANNEL_1
TIM1.IPParameters=Channel-PWM Generation1 CH1
```

CubeMX re-validates and rewrites the file on next open, so confirm afterwards that
`MX_TIM1_Init` appears in `ProjectManager.functionlistsort`.

#### Cause B: TIM1 is held by the HAL timebase

If the SYS timebase got assigned to a hardware timer instead of SysTick, that timer
is reserved for the 1 ms HAL tick and its channels vanish from the pin menu. If
`ros32bot_stm32.ioc` contains `VP_SYS_VS_tim1` instead of `VP_SYS_VS_Systick`,
that is the cause.

**Fix:** **System Core → SYS → Timebase Source = `SysTick`**, which releases TIM1.
Your file currently has `VP_SYS_VS_Systick` already, so **this is not your
problem** — but it remains the thing to check first on a fresh project.

**Why SysTick is the right choice:** §9.1 replaces CubeMX's timer-based HAL timebase
with an RTOS-tick-based one anyway, so a timer dedicated to the timebase is not just
wasteful — it is exactly the thing that would steal TIM1. FreeRTOS keeps SysTick and
HAL rides on the FreeRTOS tick.

**Order of operations that avoids Cause B:** set **SYS → Debug = Serial Wire** and
**SYS → Timebase Source = SysTick** *first*, then enable FreeRTOS, then assign
timers. CubeMX picks a timebase timer from whatever is free at the moment FreeRTOS
is enabled, so claiming TIM1–TIM4 beforehand makes the conflict more likely.

**Tertiary check:** **RCC → MCO** (clock output) can also claim PA8. It is off by
default and should stay off — this design has no clock output.

### 9.8 Pre-generation `.ioc` self-check

CubeMX reports configuration problems as **warnings**, not errors — a yellow
triangle or an out-of-spec value will still generate code happily, and you find out
at runtime. Before generating, or whenever something behaves oddly, grep the `.ioc`
for these four signatures.

**1. APB2 divider left at `/2` silently overclocks the ADC.** This is the most
easily missed item in this guide, because nothing about the motor or encoder pins
looks wrong.

```
RCC.APB2CLKDivider=RCC_HCLK_DIV2     <- should be DIV1
RCC.APB2Freq_Value=36000000          <- should be 72000000
RCC.ADCFreqValue=18000000            <- ADC overclocked! limit is 14 MHz
```

The F1's ADC is derived from PCLK2, so halving APB2 cascades into the ADC. **18 MHz
exceeds the 14 MHz maximum** and produces inaccurate, noisy conversions rather than
an outright failure. Fix: set **APB2 Prescaler = `/1`** (→ PCLK2 72 MHz) and
**ADC Prescaler = `/6`** (→ 12 MHz). Note **APB1 `/2` is correct and must stay** —
only APB2 should be `/1`.

**2. Timer-channel modes must all have a mode string.** Every line here should end
with a mode after the comma:

```
SH.S_TIM1_CH1.0=TIM1_CH1,PWM Generation1 CH1     <- must have PWM mode
SH.S_TIM2_CH1_ETR.0=TIM2_CH1,Encoder_Interface
SH.S_TIM2_CH2.0=TIM2_CH2,Encoder_Interface
SH.S_TIM3_CH1.0=TIM3_CH1,Encoder_Interface
SH.S_TIM3_CH2.0=TIM3_CH2,Encoder_Interface
SH.S_TIM4_CH1.0=TIM4_CH1,PWM Generation1 CH1
```

A bare `SH.S_TIMx_CHy.0=TIMx_CHy` with nothing after the comma is an unconfigured
channel — see §9.7.

**3. Encoder filters should cover *both* channels.** Asymmetric filtering means one
edge of a quadrature pair is debounced and the other is not:

```
TIM2.IPParameters=IC1Filter,IC2Filter    <- both, correct
TIM3.IPParameters=IC1Filter              <- IC2Filter missing
```

Fix: set **TIM3 IC1Filter = 10 *and* IC2Filter = 10**, matching TIM2. Also confirm
**Combined Channels = Encoder Mode** on both TIM2 and TIM3, and that the encoder
mode is **TI1 and TI2** (4× counting) rather than TI1 (2×) — the `.ioc` may omit
`EncoderMode` when it equals the default, so verify it in the GUI rather than by
grep.

**4. Task priorities must put the control loop on top.** CubeMX writes them as
`name,priority,stack,entry,...` where a **larger number is a higher priority**
(`osPriorityNormal` = 24, `osPriorityLow` = 8):

```
FREERTOS.Tasks01=defaultTask,24,...;motorControl,9,...;microROS,8,...
```

Here the auto-generated dummy `defaultTask` outranks `motorControl` — backwards.
Per §5.8 the control task must be highest, sensors below it. Either delete
`defaultTask` and re-create it as your **gyro task** at `osPriorityNormal`, or drop
its priority below `motorControl`. Also note all three stacks are **128 words
(512 bytes)** — adequate for plain HAL code, too small once anything calls `printf`
or micro-ROS (§9.2).

**Also worth confirming:** `ProjectManager.TargetToolchain` — this project is set to
`STM32CubeIDE`, which is fine (it generates a `Debug/makefile` you can build with
`make`), but the Makefile toolchain integrates more cleanly with micro-ROS in
Phase 2. Not a blocker, just a decision to make consciously.

### 9.9 Which timers exist here, and why TIM1 cannot be an encoder

**"Up to three general-purpose timers" does not exclude TIM1.** The datasheet counts
timers in two separate categories, and TIM1 is in the other one:

| Timer | Category | On F103C8? | Verified against |
|---|---|---|---|
| TIM1 | **Advanced-control** | ✅ | `TIM1_BASE` in `stm32f103xb.h` |
| TIM2 | General-purpose | ✅ | `TIM2_BASE` |
| TIM3 | General-purpose | ✅ | `TIM3_BASE` |
| TIM4 | General-purpose | ✅ | `TIM4_BASE` |
| TIM5 | General-purpose | ❌ | high-density parts only |
| TIM6, TIM7 | Basic | ❌ | **zero** matches in the header |

So "three GP timers" = **TIM2/TIM3/TIM4**, and TIM1 is counted *in addition* as the
single advanced-control timer. The part has **four** timers total. The datasheet's
"up to" is a family-level hedge — the same document covers parts with more and fewer
peripherals, which is why the wording is vague.

**TIM1 is fully usable — for PWM.** `TIM1_CH1` is on PA8 and nothing else competes
for it. That is your Motor 1 PWM, exactly as planned.

**TIM1 cannot be an encoder on this board.** That "conflicts with UART1" message is a
genuine, unavoidable pin conflict, confirmed from the CubeMX device database:

| TIM1 channel | LQFP48 pin | Your use |
|---|---|---|
| CH1 | **PA8** | free ✅ → Motor 1 PWM |
| CH2 | **PA9** | **USART1_TX** ❌ |
| CH3 | **PA10** | **USART1_RX** ❌ |
| CH4 | PA11 | free |
| CH1N | PA7 or PB13 | PA7 is Encoder 2 |
| CH2N | PB0 or **PB14** | PB14 is MOT1_DIR_A |
| CH3N | PB1 or **PB15** | PB15 is MOT1_DIR_B |

Encoder mode needs **both** TI1 and TI2 — even in plain TI1 mode, TI2 is still
required to sense direction. So a TIM1 encoder would consume PA8 **and PA9**, and PA9
is your micro-ROS link. There is no remap escape: on the 48-pin package the TIM1
remap pins (PE8–PE15) **do not exist**, so `TIM1_CH2` is PA9 and only PA9. This is
why the attempt failed.

**You do not need TIM1 as an encoder** — the plan already covers both encoders with
the general-purpose timers:

| Peripheral | Pin(s) | Role |
|---|---|---|
| `TIM1_CH1` | PA8 | Motor 1 PWM |
| `TIM4_CH1` | PB6 | Motor 2 PWM |
| `TIM2` CH1+CH2 | PA0, PA1 | Encoder 1 |
| `TIM3` CH1+CH2 | PA6, PA7 | Encoder 2 |

All four timers are in use, and each is doing the job it is suited to. What you were
actually fighting on TIM1 was the half-configured channel state in §9.7 — not a
missing peripheral, and not a reason to reassign anything.

> **Caution on TIM1 complementary modes.** When configuring `TIM1` Channel1, choose
> plain **`PWM Generation CH1`**. Do *not* pick **`PWM Generation CH1 CH1N`** — it
> drags in PB13 needlessly — and never pick complementary modes for Channels 2/3,
> whose `CH2N`/`CH3N` outputs are PB14 and PB15, i.e. your motor direction pins.

**If you ever do need PA9/PA10 back for something else:** USART1 cannot be remapped
(its remap is PB6/PB7, and PB6 is Motor 2's PWM). But **USART2 on PA2/PA3 is
completely free** and would serve the same purpose — both pins are unassigned in your
`.ioc`. Not needed for Phase 1; noted only so you know the UART choice is not
permanent.

---

## 10. Next steps after generation

**Phase 1 — get the robot working under FreeRTOS (no micro-ROS)**

1. Apply the one code-level item CubeMX cannot do: add
   `Core/Src/stm32f1xx_hal_timebase_rtos.c` and delete
   `stm32f1xx_hal_timebase_tim.c` (§9.1). **No NVIC priority work yet.**
2. Walk the bring-up order in §8, steps 1–6, one subsystem at a time.
3. Build out `motorCtrl`: encoder delta → PID → PWM compare, fixed period via
   `osDelayUntil()`. Add `gyroTask` and `debugTask` with the priorities from §5.8.
4. **Phase 1 acceptance test:** both wheels drive forward and reverse under
   command; both encoders report counts whose sign and magnitude track real wheel
   motion; gyro rate readings respond plausibly when you rotate the board by hand.

**Phase 2 — add micro-ROS**

5. Enable USART1 DMA (§5.5) and the USART1/DMA1 NVIC interrupts at priority ≥ 5
   (§5.7), then regenerate.
6. Add the `micro_ros_stm32cubemx_utils` sources and the prebuilt `libmicroros`
   for Cortex-M3; wire them into the generated `Makefile`.
7. Add the `microROS` task and expose the ROS 2 interface (`cmd_vel` in,
   `odom` + battery + IMU out); run `micro-ROS-agent` on the Pi over
   `/dev/ttyAMA0` at the matching baud.
8. Only once the link is stable, raise the baud rate — and watch RAM in the `.map`
   (§9.2), which is where the pressure actually appears.

---

## 11. Seeing and avoiding pin/channel conflicts

CubeMX tells you *that* two things collide, but not what else would have worked.
These are the places to look, in rough order of usefulness.

### 11.1 In the CubeMX GUI

| Where | What it shows |
|---|---|
| **Pinout view** (chip diagram) | The conflict map. Green = assigned and configured; **orange/yellow + triangle = conflict or incomplete**; grey = free. Hover any pin for a tooltip naming its signal — and on a conflict, naming the other claimant |
| **Click any pin** | Popup listing *every* function that pin can carry. Functions already taken are greyed out; hovering gives the reason. **This is the single best place to ask "what else could go here, and why can't it?"** |
| **Categories → Timers → TIMx**, upper **Mode** pane | That timer's Channel1–4 dropdowns, i.e. the per-timer channel allocation. This is the "what channels does this peripheral have" view |
| **Peripheral → DMA Settings**, and **System Core → DMA** | A **separate** conflict domain. On F1 the DMA1 request mapping is fixed per channel, so DMA conflicts appear here and nowhere else |
| **System Core → NVIC** | Interrupt-vector conflicts (less common, but real once micro-ROS adds DMA IRQs) |

**Two GUI traps worth knowing:**

- The **Mode** pane (upper) holds the per-channel mode dropdowns. The
  **Configuration → Parameter Settings** pane (lower) holds only per-channel
  *values* and contains **no** mode options — looking for "PWM" there always fails.
- Assigning a **signal** from the pin popup is not the same as choosing a **mode**
  in the peripheral pane. Doing only the former leaves an orphaned signal with no
  mode, which is the yellow-triangle state in §9.7.

### 11.2 The structural rule: allocate the constrained pins first

Conflicts are far easier to avoid if you know which functions have **no
alternative pin**. On the LQFP48 package, extracted from
`~/STM32CubeMX/db/mcu/STM32F103C(8-B)Tx.xml`:

| Peripheral | Pin options |
|---|---|
| **TIM1** CH1 / CH2 / CH3 / CH4 | PA8 / PA9 / PA10 / PA11 — **no alternatives** |
| **TIM4** CH1 / CH2 / CH3 / CH4 | PB6 / PB7 / PB8 / PB9 — **no alternatives** |
| **I2C2** SCL / SDA | PB10 / PB11 — **no alternatives** |
| **ADC1_IN4** | PA4 — no alternative |
| **USART2** TX / RX | PA2 / PA3 — no alternatives |
| TIM2 CH1 / CH2 / CH3 / CH4 | PA0 or PA15 / PA1 or PB3 / PA2 or PB10 / PA3 or PB11 |
| TIM3 CH1 / CH2 | PA6 or PB4 / PA7 or PB5 |
| USART1 TX / RX | PA9 or PB6 / PA10 or PB7 |

**TIM1 and TIM4 are completely rigid**: those eight pins are the *only* pins that
can carry their channels. So decide the PWM timers first — they consume exactly
PA8–PA11 and PB6–PB9 and nothing else can substitute. Everything else
(TIM2/TIM3 encoders, USART) can then be routed around them.

Also note the overlaps that cause most surprises:

- `USART1_TX` remaps to **PB6** = `TIM4_CH1`. Never enable the USART1 remap while
  TIM4 drives a motor.
- `TIM3` partial remap moves CH1/CH2 to **PB4/PB5**.
- `USART3_TX`/`RX` are **PB10/PB11** = `I2C2_SCL`/`SDA`.
- `TIM1_CH2N`/`CH3N` are **PB14/PB15**, and `TIM1_CH1N` is PA7 or PB13.

### 11.3 The `.ioc` is the ground truth — `tools/ioc_report.py`

The GUI shows state one panel at a time. This script dumps the whole allocation
from the `.ioc` and flags the traps, without opening CubeMX:

```bash
cd STM32
python3 tools/ioc_report.py ros32bot_stm32/ros32bot_stm32.ioc
```

It reports enabled peripherals, every pin with its signal/mode/label, per-timer
channel modes, planned `MX_*_Init` functions, clock values, FreeRTOS task
priorities, each used pin's **alternative functions**, and the **free pin list** —
then a summary of problems. It catches, specifically:

- timer channels with **no mode set** (the §9.7 yellow-triangle bug),
- an **encoder timer with only one channel** configured (encoder mode needs both),
- an enabled timer missing its `MX_TIMx_Init`,
- **ADC clock above 14 MHz**, and a **halved APB2**,
- the control task **not** being the highest-priority task.

Use it before every code generation. It reads the MCU XML from the CubeMX
installation, so pin alternatives come from the same database CubeMX itself uses.

---

*Configuration above reflects the pin list as specified. If any subsequent
hardware revision changes a pin, update §4 first and re-check §9.5's conflict list
before regenerating — remap/conflict errors are silent at compile time and only
appear as dead or erratic pins at runtime.*
