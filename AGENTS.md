# TinyKnob agent notes

Follow the global ponytail rules (`~/AGENTS.md` / Cursor ponytail): YAGNI, stdlib first, fewest files, `ponytail:` on intentional ceilings. This file adds **hardware FOC bring-up** so later agents can re-run the same loop without rediscovering it.

## Build / flash

```shell
make docker_build          # → build/src/RP2040_HID_Template.uf2
# Device in UF2 (RPI-RP2 mass storage), or app Bulk 0x7F / CDC UPLOAD:
cp build/src/RP2040_HID_Template.uf2 /media/$USER/RPI-RP2/fw.uf2 && sync
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

## Feel / SPRING / SPIN / cog (automated)

```shell
python3 tools/feel_check.py                  # telem≥800Hz; TEST/SPRING/SPIN smoke; SET_K; SPIN jitter warn
python3 tools/feel_check.py --cog-learn      # also host-learn fill/peak gates (no write)
python3 tools/cog_cal.py --host-learn --write src/cog_lut_default.h --no-bootloader
# defaults 20s / 2 rev; exit 1 if bins <56/64 or |peak|≤1e-3 → rebuild+flash
```

Exit: `feel_check` `0` ok, `1` fail, `2` warn-only (SPIN rest σ high). Hand: SPRING voltage soft→wall envelope across ±π (no effort cliff); no deadzone. SPIN detents—if stickier, re-learn `--invert`.

### Firmware knobs (`src/pins.h`)

| Define | Role |
|--------|------|
| `CUR_LOOP_EN` | Sample CSA + Park into telem |
| `CUR_LOOP_CTRL` | `0` = sense-only (voltage outer); `1` = Id/Iq PI |
| `CS_SIGN` | ±1 overall current sign |
| `CS_PHASE_ORD` | 0..5 ABC permutation |
| `CS_TE_OFF` | Park angle shift vs align `te` (sense/PI only; SVPWM stays on align `te` except SPIN) |
| `CS_GAIN_V_PER_A` | Must match DRV8316 GAIN pin |
| `IQ_CMD_A` | Outer `uq_cmd∈[-1,1]` → Iq_ref (A) when CTRL=1 |
| `SPRING_UQ_MAX` | Soft effort envelope inside blend |
| `SPRING_WALL_UQ` | Effort envelope past ±π blend |
| `SPRING_WALL_BLEND_RAD` | Envelope soft→wall across ±π (±~26°) |
| `SPRING_WALL_D` | Extra damping at wall (lerped in blend) |
| `SPRING_COG_FADE_RAD` | Fade cog FF near rest |

### Tuning order (do not skip)

1. `CUR_LOOP_CTRL=0`, `CS_TE_OFF=0`, synced CSA offset (firmware recalibrates on START/STOP in mid-low PWM window).
2. Sweep `CS_PHASE_ORD` 0..5; pick best `sign(Iq)==sign(Uq)` (or worst≈0% then flip `CS_SIGN`).
3. Fine `CS_TE_OFF` for high sign match and low `|Id|/|Iq|`.
4. Set `CUR_LOOP_CTRL=1`; re-run script — want `sign(Iq)==sign(Iq_ref)` ≳90% and small `|Iq−Iq_ref|`.
5. `feel_check.py` then hand-test SPRING / SPIN / TEST.
6. **Cogging FF**: bake with `cog_cal.py --host-learn` (fill gates); wrong phase → `--invert` (this board needed `--invert` after non-invert SPIN starved telem). SPRING/SPIN/STRESS use cog when LUT≠0; TEST/POS skip.
7. **Current PI** only on **SPIN / STRESS**. **SPRING / TEST / POS / COGCAL** = voltage SVPWM (SPRING on current PI starved USB under load). TinyUSB on core0; `USBCTRL_IRQ` above PWM. Mid-low sample stays sync wait in wrap ISR (no alarm-defer). No SPRING deadzone (edge bang-bang).

### Pass / fail (script)

- Sense-only: `sign(Iq)==sign(Uq)` ≳90%, `|Id|/|Iq|` ≪1; strong `Iq↔2×fe` ⇒ offset/sample-window or reverse Park.
- Closed-loop: `sign(Iq)==sign(Iq_ref)` ≳90%, median `|Iq−Iq_ref|` small, `|Id|/|Iq|` ≪1.
- Feel: telem ≥800 Hz in IDLE/TEST/SPRING/SPIN; cog learn ≥56/64 bins.
- After a failing/passing automated run the device is usually in **UF2** unless `--no-bootloader` / feel_check default leave-app.

### Agent loop when changing FOC

1. Edit `pins.h` / `motor.c` as needed.
2. `make docker_build` + copy UF2 (enter bootloader via script UPLOAD or Bulk `0x7F` if still in app).
3. Run `foc_sense_check.py` and/or `feel_check.py`; on DIAG, change firmware (not only the script) and repeat.
4. `graphify update .` after code edits (workspace graphify rule).

## USB identity

`VID=0xACDC` `PID=0x4011` — HelloWorks TinyRoller. Vendor Bulk for telem/commands; CDC for logs. Protocol: `docs/usb-protocol.md`.
