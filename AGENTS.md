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

### Firmware knobs (`src/pins.h`)

| Define | Role |
|--------|------|
| `CUR_LOOP_EN` | Sample CSA + Park into telem |
| `CUR_LOOP_CTRL` | `0` = sense-only (voltage outer); `1` = Id/Iq PI |
| `CS_SIGN` | ±1 overall current sign |
| `CS_PHASE_ORD` | 0..5 ABC permutation |
| `CS_TE_OFF` | Park angle shift vs align `te` (sense/PI only; SVPWM stays on align `te`) |
| `CS_GAIN_V_PER_A` | Must match DRV8316 GAIN pin |
| `IQ_CMD_A` | Outer `uq_cmd∈[-1,1]` → Iq_ref (A) when CTRL=1 |

**SPIN** always uses the voltage flywheel (`spin_uq`), even when `CUR_LOOP_CTRL=1` (current PI + `CS_TE_OFF` mix feels like cogging).

### Tuning order (do not skip)

1. `CUR_LOOP_CTRL=0`, `CS_TE_OFF=0`, synced CSA offset (firmware recalibrates on START/STOP in mid-low PWM window).
2. Sweep `CS_PHASE_ORD` 0..5; pick best `sign(Iq)==sign(Uq)` (or worst≈0% then flip `CS_SIGN`).
3. Fine `CS_TE_OFF` for high sign match and low `|Id|/|Iq|`.
4. Set `CUR_LOOP_CTRL=1`; re-run script — want `sign(Iq)==sign(Iq_ref)` ≳90% and small `|Iq−Iq_ref|`.
5. Hand-test SPRING / SPIN / TEST. If SPIN is coggy, do **not** force SPIN through the current PI.

### Pass / fail (script)

- Sense-only: `sign(Iq)==sign(Uq)` ≳90%, `|Id|/|Iq|` ≪1; strong `Iq↔2×fe` ⇒ offset/sample-window or reverse Park.
- Closed-loop: `sign(Iq)==sign(Iq_ref)` ≳90%, median `|Iq−Iq_ref|` small, `|Id|/|Iq|` ≪1.
- After a failing/passing automated run the device is usually in **UF2** unless `--no-bootloader`.

### Agent loop when changing FOC

1. Edit `pins.h` / `motor.c` as needed.
2. `make docker_build` + copy UF2 (enter bootloader via script UPLOAD or Bulk `0x7F` if still in app).
3. Run `foc_sense_check.py`; on DIAG, change firmware (not only the script) and repeat.
4. `graphify update .` after code edits (workspace graphify rule).

## USB identity

`VID=0xACDC` `PID=0x4011` — HelloWorks TinyRoller. Vendor Bulk for telem/commands; CDC for logs. Protocol: `docs/usb-protocol.md`.
