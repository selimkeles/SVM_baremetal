# SVM_baremetal

Open-loop Space Vector PWM (SVPWM) 3-phase inverter firmware for AC motor drive. Final year thesis project — Dokuz Eylül University, Electrical & Electronics Engineering, 2022.

**MCU**: STM32F407VG Discovery board · **Toolchain**: STM32CubeIDE · **RTOS**: FreeRTOS · **HAL**: STM32 HAL

---

## What This Is

A bare-metal + FreeRTOS firmware implementing open-loop V/f control with Space Vector PWM modulation. The firmware drives a custom 3-phase IGBT inverter (6× FGH40T65SHDF) via IS314W optocoupler-isolated gate drivers.

**Key parameters**

| Parameter | Value |
|-----------|-------|
| SYSCLK | 168 MHz (HSE + PLL) |
| PWM carrier | 10 kHz, center-aligned |
| Dead time | 1 µs |
| Control law | Open-loop V/f (M = f/50, clamped to 1.0) |
| Frequency input | ADC on PC1, potentiometer-scaled (0–~81 Hz) |
| Enable | PA0 button toggles TIM1 MOE |

**Gate outputs** — TIM1 complementary outputs (AF1)

| Phase | High side | Low side |
|-------|-----------|----------|
| U | PE9 (CH1) | PE8 (CH1N) |
| V | PE11 (CH2) | PB0 (CH2N) |
| W | PE13 (CH3) | PB1 (CH3N) |

---

## Repository Layout

```
App/            SVPWM math, V/f control, inverter state machine
BSP/            Hardware wrappers (PWM, ADC, button, LED)
Core/           CubeMX-generated HAL init, FreeRTOS task stubs, IRQ handlers
USB_DEVICE/     USB CDC — parameter read/write over serial
SVM_baremetal_hardware/   Hardware submodule — schematics only (no PCB layout)
Final_Year_Project_Thesis.pdf   Theory reference
SVM_baremetal.ioc               CubeMX project file
```

The three-layer split is **App / BSP / Drivers**. CubeMX owns all peripheral init; application logic lives exclusively in `App/` and `BSP/`. Never edit CubeMX-generated files outside `USER CODE` markers.

---

## Getting Started

### Prerequisites

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) (tested on 1.x)
- STM32F407VG Discovery board
- ST-LINK/V2 (on-board on Discovery)

### Build & Flash

1. Clone with submodules:
   ```sh
   git clone --recurse-submodules <repo-url>
   ```
2. Open STM32CubeIDE → **File → Open Projects from File System** → select this directory.
3. **Project → Build Project** — output lands in `Debug/`.
4. **Run → Debug** (ST-LINK) to flash and halt at `main()`.
5. Press **Resume** — the green Discovery LED (PD12) lights when the inverter is initialised.

### First Run (bench, no motor)

1. Connect a potentiometer (center wiper to PC1, rails to 3.3 V / GND) for frequency input.
2. Power the Discovery over USB.
3. Press **PA0 (blue user button)** to toggle PWM output enable. The LED changes state.
4. Probe PE9/PE11/PE13 and PE8/PB0/PB1 on a scope — you should see complementary 10 kHz center-aligned PWM with ~1 µs dead band. Duty cycle shifts as you turn the pot.
5. To set parameters over USB, connect a CDC serial terminal (115200 8N1) and type `help`.

> **Do not connect the inverter power stage without first verifying gate signals on a scope.** See the hardware section below for known failure modes.

---

## Hardware

The `SVM_baremetal_hardware/` submodule contains **schematics only**. PCB layout files are not included in this repository.

Schematic sheets:

| File | Contents |
|------|----------|
| `inverter.kicad_sch` | Top-level sheet |
| `VSI.kicad_sch` | Voltage-source inverter (IGBT bridge + DC bus) |
| `GateDrive.kicad_sch` | Gate drive circuits (optocouplers + asymmetric Rg) |
| `Power.kicad_sch` | Isolated 15 V gate supplies, DC bus rectifier |
| `Connectors.kicad_sch` | External connectors |

Open with KiCad 7+.

### Known Hardware Issue (v1)

The v1 board uses single-rail 15 V/0 V isolated SMPS for gate bias. Zero negative off-bias means Miller-coupled dV/dt can spuriously turn on the high-side IGBTs, leading to shoot-through. **Do not exceed low DC bus voltages on v1 hardware** until negative gate bias (−15 V) is added. See the thesis (Section 4) for full analysis.

---

## Theory Reference

`Final_Year_Project_Thesis.pdf` covers:
- Space Vector PWM sector selection and dwell-time derivations
- V/f control law and modulation index clamping
- Hardware design rationale (gate drive topology, DC bus sizing)
- Simulation and experimental results (39 / 21 / 59 Hz test runs, FFT)

---

## Licence

Academic project. Contact the author before reuse.
