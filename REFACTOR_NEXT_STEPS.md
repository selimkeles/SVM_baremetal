# Refactor — next steps

What I've done so far:

- `App/Inc/` + `App/Src/` — SVPWM math (`svpwm.c`), V/f control (`vf_control.c`), and glue (`inverter.c`) extracted from `Core/Src/main.c` as pure, HAL-independent code. `sinf()` replaces `sin()`.
- `BSP/Inc/` — API headers for `pwm`, `adc`, `button`, `led`, `clock`, `gpio`. No `.c` files yet — they need HAL to be regenerated first.
- Current `Core/Src/main.c` is untouched (will be overwritten by CubeMX).

## What you need to do next (interactive, in STM32CubeMX / CubeIDE)

1. **Snapshot first**: `git add -A && git commit -m "wip: App/ and BSP/ skeleton before CubeMX regen"` so regen is easy to diff/revert.
2. **Open `SVM_baremetal.ioc`** in CubeMX (or double-click in CubeIDE → "Device Configuration Tool").
3. **Configure peripherals** per the approved plan:
   - **TIM1**: center-aligned mode 1, ARR 8400, PSC 0, CH1/2/3 + N channels, deadtime = 168 (1 µs), enable Update IRQ, CCR preload ON.
   - **ADC1**: IN11 (PC1), single conversion, 12-bit right-aligned, sample 3 cycles, prescaler /4 → 21 MHz.
   - **EXTI0**: PA0 rising edge, NVIC priority 0.
   - **TIM1 UP/TIM10 IRQ**: NVIC priority 2.
   - **GPIO**: PE8/9/11/13, PB0/1 → AF1 push-pull high-speed. PD12-15 → output. PC1 → analog.
   - **FreeRTOS**: enable CMSIS-RTOS v2, heap 8 KB, tick 1 ms. Keep HAL on SysTick; set `configMAX_SYSCALL_INTERRUPT_PRIORITY` to 5 (the default). TIM1_UP at priority 2 stays above that (never calls RTOS APIs).
4. **Project Manager → Code Generator**: select *"Generate peripheral initialization as a pair of .c/.h files per peripheral"* so init isn't stuffed into main.c.
5. **Generate code**. Review the diff.
6. **Bump heap in `STM32F407VGTX_FLASH.ld`**: `_Min_Heap_Size = 0x2000;` (8 KB for FreeRTOS).
7. **Update `.cproject` include paths**: add `../App/Inc`, `../BSP/Inc`; add `../App/Src`, `../BSP/Src` as source folders. Remove `../Drivers/MyLib`.

Ping me after regen and I'll:

- Fill in `BSP/Src/*.c` implementations over the generated HAL handles.
- Wire the TIM1 update ISR to `inverter_on_pwm_update(bsp_adc_read())` in `stm32f4xx_it.c`.
- Wire the EXTI0 callback to `inverter_toggle_enable()`.
- Thin `main.c` down to RTOS init + default blinker task.
- Delete `Drivers/MyLib/`.
- Update `CLAUDE.md`.
