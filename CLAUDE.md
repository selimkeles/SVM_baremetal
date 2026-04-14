# SVM_baremetal — Project Context

Open-loop Space Vector PWM (SVPWM) 3-phase inverter for AC motor drive. Final year thesis project. Bare-metal, register-level STM32 code — no HAL.

## Project Layout

Single-file implementation — **all logic lives in [Core/Src/main.c](Core/Src/main.c)**:

- `main.c` — clock setup, GPIO/TIM1/ADC/EXTI init, and the SVPWM ISR (`TIM1_UP_TIM10_IRQHandler`) that computes dwell times and updates CCR registers every PWM cycle
- `Drivers/MyLib/Inverter.{c,h}` — empty stubs, not used
- `SVM_baremetal_hardware/` — git submodule with schematic / PCB
- `Final_Year_Project_Thesis.pdf` — thesis document (LaTeX-authored, has organized section headers — consult for theory, dwell-time derivations, and design rationale)

## Firmware Summary

| Parameter | Value |
|-----------|-------|
| SYSCLK | 168 MHz (HSE + PLL) |
| PWM carrier | **10 kHz** (center-aligned; `f = 10000*2` in code accounts for up-down counting -> `ARR = 8400`) |
| Dead time | 1 us (via `TIM1->BDTR`) |
| ISR rate | 10 kHz (one SVPWM update per PWM period, ~100 us budget) |
| Control | Open-loop V/f (M = freq/50, clamped to 1.0; field weakening above 50 Hz) |
| Frequency input | ADC1 PC1, scaled `freq = DR/50` (max ~81 Hz) |
| Enable button | PA0 (EXTI0) toggles `TIM1->BDTR.MOE` |

### Gate signal pin map (TIM1 complementary outputs, AF1)

| Phase | High side | Low side |
|-------|-----------|----------|
| U | PE9 (CH1) | PE8 (CH1N) |
| V | PE11 (CH2) | PB0 (CH2N) |
| W | PE13 (CH3) | PB1 (CH3N) |

## Hardware

- **MCU**: STM32F407VG Discovery board (Cortex-M4F, single-precision FPU enabled)
- **Switches**: FGH40T65SHDF discrete IGBTs (six — three half-bridges)
- **Gate isolation**: IS314W optocouplers between MCU and gate drivers
- **DC bus rectifier**: KBPC3510 full-bridge
- **DC bus cap**: AL20A681DF400 — single 680 uF / 400 V bulk
- **Gate supplies**: 4x isolated 15V / 0V SMPS (3 for high sides, 1 shared for low sides)

### Known hardware failure mode

Post-repair IGBT blow-ups attributed to **Miller-coupled parasitic turn-on** of the high-side IGBT when the low-side switches (dV/dt through Cgc). PCB repair likely increased gate-loop and/or power-loop parasitic inductance. Mitigations: negative gate off-bias, lower Rg_off, active Miller clamp, tighter power-loop layout, snubbers close to the IGBT package. Lowering DC bus voltage is a band-aid — reduces dV/dt but does not remove the coupling.

## Coding Conventions

- Direct CMSIS register access — do not introduce HAL dependencies
- Keep the ISR lean; it runs at 10 kHz with ~16,800 cycles budget
- `sin()` (double) is used in the ISR — `sinf()` or a sine lookup table are known optimizations if headroom becomes an issue
- No dynamic allocation
- Comments are right-aligned in the existing style; match it when editing `main.c`

## Build

STM32CubeIDE project. Standard "Build Project" produces output in `Debug/` (gitignored). Linker script: `STM32F407VGTX_FLASH.ld`.

## Working with this repo

- **Use the Context7 MCP** for up-to-date library and API documentation (CMSIS, STM32F4 reference manual references, peripheral headers). Prefer Context7 lookups over guessing register layouts or bit fields.
- **Consult the thesis PDF** (`Final_Year_Project_Thesis.pdf`) for theory background — SVPWM sector math, dwell-time derivations, V/f justification, hardware design choices. Section headers are organized; navigate by ToC.
- The STM32F4 reference manual (RM0090) and datasheet are authoritative for any register/peripheral question.
