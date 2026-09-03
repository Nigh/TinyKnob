# TinyKnob agent notes

Follow the global ponytail rules (`~/AGENTS.md` / Cursor ponytail): YAGNI, stdlib first, fewest files, `ponytail:` on intentional ceilings. This file adds **hardware FOC bring-up** so later agents can re-run the same loop without rediscovering it.

## Build / flash

Target: **Waveshare RP2350-Zero** (`PICO_BOARD=waveshare_rp2350_zero`), CMake project `RP2350_TinyKnob`.

Motor: 24-slot / 22-pole (11 pole pairs), delta winding. Rated 12–36 V.
At 12 V: 4 A maximum, 0.65 A rated, 610 RPM rated. At 24 V: 4 A
maximum, 1.4 A rated, 1100 RPM rated.

```shell
make docker_clean          # wipe build/
make docker_build          # → build/src/RP2350_TinyKnob.uf2
# Device in UF2 (RP2350 mass storage), or app Bulk 0x7F / CDC UPLOAD:
cp build/src/RP2350_TinyKnob.uf2 /media/$USER/RP2350/fw.uf2 && sync
```

Wait until `lsusb` shows `acdc:4011` (TinyRoller) before host tests.

## FOC sense / current-loop check (automated)

Host tool: `tools/foc_sense_check.py` (needs `pyusb`, `numpy`).

```shell
source ~/venv/bin/activate   # or any env with pyusb+numpy
python3 tools/foc_sense_check.py
python3 tools/foc_sense_check.py --seconds 6 --revs 1.5
python3 tools/foc_sense_check.py --no-bootloader   # leave app running for hand test
```

Flow: STOP → START (align) → GOTO sweep → STOP → print diagnostics → default **UPLOAD** to UF2 (so the next firmware edit can flash immediately).

**Note:** default firmware is `CUR_LOOP_CTRL=1`: TEST/POS use PI; SPIN uses low-damping voltage control with PI overspeed braking; SPRING/STRESS remain voltage-driven for the best hardware feel. Sampling is PWM-timed low-side ADC/DMA; never restore ISR busy-wait sampling (it starves USB on SPIN/STRESS).

## Feel / SPRING / SPIN / cog (automated)

```shell
python3 tools/feel_check.py                  # telem≥800Hz; TEST/SPRING/SPIN smoke; SET_K; SPIN jitter warn
python3 tools/feel_check.py --cog-learn      # also host-learn fill/peak gates (no write)
python3 tools/cog_cal.py --host-learn --write src/cog_lut_default.h --no-bootloader
# defaults 24s / 12 rev each direction; steady samples only, exit 1 if either pass <90% of 1024 bins or |peak|≤1e-3 → rebuild+flash
```

Exit: `feel_check` `0` ok, `1` fail, `2` warn-only (SPIN rest σ high). Hand: SPRING voltage soft→wall envelope across ±π; SPIN/STRESS voltage (USB stays up). SPIN detents—if stickier, re-learn `--invert`.

### Firmware knobs (`src/pins.h`)

| Define | Role |
|--------|------|
| `CUR_LOOP_EN` | PWM-timed common-low ADC burst + DMA + Park telemetry |
| `CUR_LOOP_CTRL` | `1` enables PI for TEST/POS and SPIN overspeed braking; normal SPIN/SPRING/STRESS stay voltage-driven |
| `CS_SIGN` | ±1 overall current sign |
| `CS_PHASE_ORD` | 0..5 ABC permutation |
| `CS_TE_OFF` | Park angle shift vs align `te` (sense only; SVPWM on align `te`) |
| `CS_GAIN_V_PER_A` | Must match DRV8316 GAIN pin |
| `IQ_CMD_A` | Outer `uq_cmd∈[-1,1]` → Iq_ref (A) when CTRL=1 (future) |
| `SPRING_UQ_MAX` | Soft effort envelope inside blend |
| `SPRING_WALL_UQ` | Effort envelope past ±π blend |
| `SPRING_WALL_BLEND_RAD` | Envelope soft→wall across ±π (±~26°) |
| `SPRING_WALL_D` | Extra damping at wall (lerped in blend) |
| `SPRING_COG_FADE_RAD` | Fade cog FF near rest |

### Tuning order (do not skip)

1. `CUR_LOOP_CTRL=0`, `CS_TE_OFF=0`, synced CSA offset (firmware recalibrates on START/STOP with mid-low wait, IRQ off).
2. Sweep `CS_PHASE_ORD` 0..5; pick best `sign(Iq)==sign(Uq)` (or worst≈0% then flip `CS_SIGN`) using synchronized telemetry.
3. Fine `CS_TE_OFF` for high sign match and low `|Id|/|Iq|`.
4. Keep `CUR_LOOP_CTRL=1` after sense validation: TEST/POS and SPIN overspeed braking use PI; normal SPIN/SPRING/STRESS stay voltage-driven.
5. `feel_check.py` then hand-test SPRING / SPIN / TEST / STRESS (telem must stay ≥800 Hz under load / freewheel / stress ramps).
6. **Cogging FF**: bake the 1024-point LUT with continuous bidirectional `cog_cal.py --host-learn` (≥90% steady-sample fill per pass); wrong phase → `--invert`. SPIN/STRESS use cog; SPRING/TEST/POS skip.
7. **Per-mode output**: current PI for TEST/POS and SPIN overspeed braking; direct voltage for normal SPIN/SPRING/STRESS. TinyUSB on core0; `USBCTRL_IRQ` above PWM. Runtime sampling uses PWM timer + DMA; busy-wait is calibration-only.

### Pass / fail (script)

- Sense-only: `sign(Iq)==sign(Uq)` ≳90%, `|Id|/|Iq|` ≪1; strong `Iq↔2×fe` ⇒ offset/sample-window or reverse Park.
- Closed-loop: only after synchronized sense validation + `CUR_LOOP_CTRL=1`; want `sign(Iq)==sign(Iq_ref)` ≳90%.
- Feel: telem ≥800 Hz in IDLE/TEST/SPRING/SPIN/STRESS; cog learn ≥90% of 1024 bins per direction with speed CV ≤0.35.
- After a failing/passing automated run the device is usually in **UF2** unless `--no-bootloader` / feel_check default leave-app.

### Agent loop when changing FOC

1. Edit `pins.h` / `motor.c` as needed.
2. `make docker_build` + copy UF2 (enter bootloader via script UPLOAD or Bulk `0x7F` if still in app).
3. Run `foc_sense_check.py` and/or `feel_check.py`; on DIAG, change firmware (not only the script) and repeat.
4. `graphify update .` after code edits (workspace graphify rule).

## USB identity

`VID=0xACDC` `PID=0x4011` — HelloWorks TinyRoller. Vendor Bulk for telem/commands; CDC for logs. Protocol: `docs/usb-protocol.md`.

## STM32G431CBU6 target

Board: WeAct STM32G431 Core Board QFN48 V1.0. The accepted TinyKnob signal map
and USB-C solder-bridge conflicts are documented in `README.md`; keep those pins
consistent when adding drivers. The current STM32 build is a CMSIS/LL minimal
motor test: TinyUSB Vendor Bulk, TIM1 20 kHz three-phase PWM, DWT-timed MT6701 SSI, and a
fixed 12% open-loop forward/reverse sequence. It has no ADC/current protection;
use a 12 V / 1 A current-limited bench supply. `START` verifies/alines, `TEST`
runs continuously, and `STOP`, USB loss, CRC failure, or encoder stall disables
nSLEEP. Test it with `python3 tools/stm32g4_motor_test.py`; CDC is intentionally
disabled. Do not add the remaining RP2350 modes before synchronized ADC/DMA bring-up.

```shell
make submodules
make TARGET=stm32g4 build
# Put the MCU in its system USB DFU bootloader with BOOT0/reset first.
make TARGET=stm32g4 flash
```

Artifacts are under `build/stm32g4/platforms/stm32g4/`. The dependency is pinned
by the `third_party/STM32CubeG4` gitlink; initialize only its CMSIS Device and
STM32G4xx HAL Driver nested modules (the latter supplies LL headers), plus the
`third_party/tinyusb` gitlink.
