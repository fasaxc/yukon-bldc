# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Work-in-progress sinusoidal FOC (field-oriented control) driver for a brushless DC motor, written in MicroPython for the **Pimoroni Yukon** (an RP2040 board). All logic lives in a single file, `main.py`. The motor's rotor angle comes from a sensor that emits a PWM signal whose duty cycle encodes the absolute angle.

## Development workflow

There is **no host build/lint/test system** — this is firmware that runs on the board.

- **Deploy & run**: use the MicroPico (Pico-W-Go) VS Code extension (`paulober.pico-w-go`, listed in `.vscode/extensions.json`). Upload `main.py` to the board over serial; it auto-runs because the module ends with `try: run() finally: yukon.reset()`. `.micropico` just marks the project folder for the extension.
- **On-board dependency**: the `pimoroni_yukon` library must be present in the board firmware (Pimoroni's Yukon MicroPython build). `machine`, `rp2`, `uctypes`, `micropython` are MicroPython built-ins.
- **Runtime controls** (in `run()`): button **A** decreases / **B** increases `target_speed` (RPS); **BOOT** stops. The loop prints speed/power/angle diagnostics roughly once per second.
- **Only host-side check available**: `python3 -m py_compile main.py` catches syntax errors. It does **not** validate `@micropython.viper` semantics, register-level behavior, or anything hardware-dependent — those require a bench run.
- Type-checking is via Pylance in "basic" mode against MicroPico stubs (`~/.micropico-stubs`); `reportMissingModuleSource` is suppressed for the firmware imports.

## Architecture (the big picture)

The real-time commutation runs **entirely inside a hard DMA interrupt**; the main `run()` loop only does outer speed control and the button UI. Understanding the data path and the shared-state convention is essential before touching `Motor`.

**Sensor → state, with no CPU in the loop:**
1. A PIO program (`pio_read_pwm`) continuously measures the sensor PWM, packing high-time and total-period into one 32-bit word (each stored as `0xffff - count`). `Motor.update` / `_read_pwm_slow` decode angle as `high / period`.
2. A two-channel DMA ring copies each reading + a timestamp into RAM with no CPU involvement: `_sm_dma` (paced by the PIO RX DREQ) copies the reading, chains to `_time_dma` which copies `TIMERAWL`, which chains back to `_sm_dma`. `_time_dma`'s completion IRQ invokes `Motor.update`.
3. A third **`_dummy_dma`**, paced by DMA TIMER0, fires *extra* `update` IRQs between real sensor readings so the angle predictor runs more often than the sensor updates. `update` re-arms it a few times after each real reading (the retrigger count and the dummy DMA's own read-address-trigger register are themselves stored in the shared state array).

**Shared ISR state — the central contract:** `_set_up_isr_vars()` builds one `array.array("i")` (`_isr_backing_arr`) holding *every* field the ISR touches (DMA targets, angle/speed, pole pairs, offsets, power, LUT pointer, PWM duty register addresses, phase tap offsets, etc.). The ISR reads/writes it through a single raw pointer (`_isr_vars_addr` → `ptr32`) to avoid slow attribute lookups. The numbered index comments in `_set_up_isr_vars` are the source of truth, and three places must stay in sync with them:
- `update()` and `_set_duty()` (raw `vars[N]` access),
- the `@property` accessors on `Motor` (the safe API the main loop uses, e.g. `speed`, `drive_power`, `angle_offset`),
- `_set_up_isr_vars()` itself.

If you add/remove/reorder a field, update all of these together (and `addressof`-based init like `vars[16]`).

**Commutation math (`update`):** decode angle → compute speed from the angle/timestamp delta → predict current angle (`angle + speed * (now - timestamp + sensor_delay)`) → convert mechanical to electrical angle (`angle * num_pole_pairs + angle_offset`) → add the `drive_offset` (1024 = +90° electrical for forward torque, 3072 for reverse) → look up three sine duties 120° apart from the LUT → write them **directly into the PWM CC registers** (inlined rather than `duty_u16()` to save ~10µs; two channels are packed into one 32-bit write).

**Calibration (`calibrate`):** sweeps the field through a known electrical span, reads the resulting mechanical motion to derive `num_pole_pairs` and the `angle_offset`. Sensor rotation direction is corrected by **swapping two motor phases** (via `_swap_phases`, which exchanges the two non-zero LUT tap offsets stored in the state array); after the swap the rest of the system only ever sees one direction, so the hot path needs no direction factor and measured speed has the right sign for the control loop.

**PWM setup (`_configure_pwms`):** each motor uses two PWM slices (three of the four channels). Frequency is set to `2 * pwm_freq` because phase-correct mode (enabled via raw CSR writes) halves it; both slices are started together for sync. The slot→GPIO/PWM/PIO mapping is documented in the `Motor.__init__` docstring.

## Conventions & gotchas

- **Hard IRQ rules**: `update` is `@micropython.viper`, runs in a hard IRQ, and must not allocate. Its `try/except BaseException: stop=True` is the safety net that halts the main loop on any ISR fault. `micropython.alloc_emergency_exception_buf` is set up at import for this.
- **Viper/Pylance shim**: `ptr32 = ptr16 = lambda x: x` near the top exists only to stop Pylance erroring on Viper pointer casts; it is shadowed by the real Viper builtins at runtime.
- **Register-level assumptions**: direct `machine.mem32` pokes assume RP2040 register layout and a 125 MHz system clock (e.g. the DMA timer rate, PWM TOP vs. `drive_power` clamp, the `0x40054028` TIMERAWL address).
- **Empirical tuning constants** (don't treat as derived): sensor read/switch delay (`+1250` µs), the `4119`/`-15` sensor scale, `min_on_fraction`, and the dummy-DMA timer divisor were tuned on hardware.
- **Commits**: history uses short imperative one-line messages. Commit `main.py` only — `.vscode/`, `.micropico`, and the `.code-workspace` are local tooling.
