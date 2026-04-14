# SVM_baremetal — Project Context

Open-loop Space Vector PWM (SVPWM) 3-phase inverter for AC motor drive. Final year thesis project. STM32 HAL + FreeRTOS, with the 10 kHz control ISR using direct register writes for CCRs where HAL overhead matters.

## Project Layout

Three-layer split: **App / BSP / Drivers**. CubeMX owns peripheral init via the `.ioc`; our code lives in `App/` and `BSP/`.

```
Core/
  Src/  main.c               ← HAL_Init, SystemClock_Config, MX_*_Init, inverter_init, osKernelStart
        tim.c, adc.c, gpio.c ← CubeMX-generated HAL init (don't edit outside USER CODE blocks)
        freertos.c           ← CubeMX-generated defaultTask stub
        stm32f4xx_it.c       ← HAL_TIM_IRQHandler dispatches to callbacks
        stm32f4xx_hal_msp.c, system_stm32f4xx.c, stm32f4xx_hal_timebase_tim.c
  Inc/  main.h, tim.h, adc.h, gpio.h, FreeRTOSConfig.h, stm32f4xx_hal_conf.h

BSP/
  Src/  bsp_pwm.c    ← wraps htim1: set_ccr, enable_output (MOE), trigger_comg
        bsp_adc.c    ← poll ADC1->DR; start via SWSTART
        bsp_button.c ← HAL_GPIO_EXTI_Callback → inverter_toggle_enable()
        bsp_led.c    ← PD12-15 (Discovery board LEDs); self-inits to guarantee mapping
        bsp_clock.c, bsp_gpio.c ← placeholders (CubeMX owns these)
  Inc/  bsp_*.h

App/
  Src/  svpwm.c      ← pure sector/dwell math (sinf), no HW
        vf_control.c ← freq → M, theta integrator, field-weakening
        inverter.c   ← glue: ISR entry calls vf_step → svpwm_compute → bsp_pwm_set_ccr;
                       owns inverter_state, setter API for future USB CLI
  Inc/  svpwm.h, vf_control.h, inverter.h

SVM_baremetal_hardware/       ← git submodule (schematic / PCB)
Final_Year_Project_Thesis.pdf ← theory reference (SVPWM math, V/f, design rationale)
```

**10 kHz ISR path** (TIM1 update → `HAL_TIM_IRQHandler` → `HAL_TIM_PeriodElapsedCallback` branch for TIM1 in `Core/Src/main.c`):
`bsp_pwm_trigger_comg()` → `bsp_adc_start()` → `inverter_on_pwm_update(bsp_adc_read())`.

TIM1 NVIC priority is overridden to **2** in `main()` (CubeMX defaulted it to 5, which would sit at the FreeRTOS cutoff).

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

- HAL for peripheral init (via CubeMX `.ioc` regeneration). Direct register access only on the 10 kHz hot path (`TIM1->CCRx`, `ADC1->DR`, `EGR |= COMG`) where HAL overhead matters
- Keep the ISR lean; it runs at 10 kHz with ~16,800 cycles budget
- `sinf()` is used in `App/Src/svpwm.c` (single-precision FPU). A sine LUT is the next optimization if headroom tightens
- No dynamic allocation on the control path. FreeRTOS heap (8 KB, `heap_4.c`) is for RTOS objects only
- New app logic → `App/`. New hardware wrappers → `BSP/`. Never add logic to CubeMX-owned files outside `USER CODE` markers
- `configMAX_SYSCALL_INTERRUPT_PRIORITY = 5`. TIM1 UP at priority 2 (above cutoff, cannot call any `*_FromISR` API). EXTI0 at 5 (may use FromISR)

## Build

STM32CubeIDE project. Standard "Build Project" produces output in `Debug/` (gitignored). Linker script: `STM32F407VGTX_FLASH.ld`.

## Working with this repo

- **Use the Context7 MCP** for up-to-date library and API documentation (CMSIS, STM32F4 reference manual references, peripheral headers). Prefer Context7 lookups over guessing register layouts or bit fields.
- **Consult the thesis PDF** (`Final_Year_Project_Thesis.pdf`) for theory background — SVPWM sector math, dwell-time derivations, V/f justification, hardware design choices. Section headers are organized; navigate by ToC.
- The STM32F4 reference manual (RM0090) and datasheet are authoritative for any register/peripheral question.
