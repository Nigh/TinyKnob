#!/usr/bin/env python3
"""TinyKnob FOC sense-only check: START → GOTO sweep → STOP → analyze Id/Iq vs Uq.

Requires: pip install pyusb numpy
Firmware: CUR_LOOP_CTRL=0 (sense-only). Boot stays IDLE until this script sends START.

Usage:
  python3 tools/foc_sense_check.py
  python3 tools/foc_sense_check.py --revs 1 --seconds 5 --save foc_sense.npz
  python3 tools/foc_sense_check.py --no-bootloader   # keep app running after test
"""
from __future__ import annotations

import argparse
import struct
import sys
import time

try:
	import numpy as np
except ImportError:
	sys.exit("need numpy: pip install numpy")

try:
	import usb.core
	import usb.util
except ImportError:
	sys.exit("need pyusb: pip install pyusb")

VID, PID = 0xACDC, 0x4011
EP_OUT, EP_IN = 0x01, 0x81
POLE_PAIRS = 11
TELEM_FMT = "<BBi3hHhhhHh"  # 24 bytes
MRAD_2PI = 6283


def open_dev():
	dev = usb.core.find(idVendor=VID, idProduct=PID)
	if dev is None:
		raise SystemExit("TinyKnob not found (VID=0xACDC PID=0x4011)")
	# Composite device: cdc_acm holds IF0/IF1 → set_configuration EBUSY until detached.
	for cfg in dev:
		for intf in cfg:
			n = intf.bInterfaceNumber
			try:
				if dev.is_kernel_driver_active(n):
					dev.detach_kernel_driver(n)
			except (NotImplementedError, usb.core.USBError):
				pass
	try:
		dev.set_configuration()
	except usb.core.USBError as e:
		# Already configured by kernel is fine once drivers are detached.
		if getattr(e, "errno", None) not in (16, None) and "Resource busy" not in str(e):
			raise SystemExit(f"set_configuration failed: {e}") from e
		try:
			dev.get_active_configuration()
		except usb.core.USBError as e2:
			raise SystemExit(f"set_configuration failed: {e}") from e2
	cfg = dev.get_active_configuration()
	intf = usb.util.find_descriptor(cfg, bInterfaceClass=0xFF)
	if intf is None:
		raise SystemExit("Vendor interface (class 0xFF) not found")
	try:
		usb.util.claim_interface(dev, intf.bInterfaceNumber)
	except usb.core.USBError as e:
		raise SystemExit(
			f"claim interface failed: {e}\n"
			"Close serial monitors on /dev/ttyACM* and retry (CDC must be detached)."
		) from e
	return dev


def send(dev, *payload: int) -> None:
	dev.write(EP_OUT, bytes(payload), timeout=1000)


def goto_mrad(dev, mrad: int) -> None:
	dev.write(EP_OUT, struct.pack("<Bi", 0x06, int(mrad)), timeout=1000)


def read_telem(dev, timeout_ms: int = 50):
	try:
		data = bytes(dev.read(EP_IN, 64, timeout=timeout_ms))
	except usb.core.USBTimeoutError:
		return None
	if len(data) >= 3 and data[0] == 0x5A:
		return None
	if len(data) < 24 or data[0] != 0xA5:
		return None
	(_magic, mode, ang, _da, _db, _dc, seq, id_ma, iq_ma, iq_ref_ma, vbus_mv, uq_q15) = (
		struct.unpack_from(TELEM_FMT, data, 0)
	)
	return {
		"mode": mode,
		"angle_mrad": ang,
		"seq": seq,
		"id": id_ma / 1000.0,
		"iq": iq_ma / 1000.0,
		"iq_ref": iq_ref_ma / 1000.0,
		"vbus": vbus_mv / 1000.0,
		"uq": uq_q15 / 32767.0,
	}


def drain(dev, n: int = 80) -> None:
	for _ in range(n):
		if read_telem(dev, timeout_ms=20) is None:
			break


def wait_mode(dev, want: int, timeout: float = 25.0):
	t0 = time.time()
	last = None
	while time.time() - t0 < timeout:
		f = read_telem(dev, timeout_ms=100)
		if f is None:
			continue
		last = f
		if f["mode"] == want:
			return f
	raise SystemExit(f"timeout waiting mode={want} (last={last})")


def collect_sweep(dev, a0: int, revs: float, seconds: float) -> list[dict]:
	rows: list[dict] = []
	span = int(revs * MRAD_2PI)
	n_steps = max(int(seconds * 100), 50)  # ~100 GOTO/s
	dt = seconds / n_steps
	t_run_end = time.time() + seconds + 0.5
	for i in range(n_steps):
		if time.time() > t_run_end:
			break
		target = a0 + int(i / max(n_steps - 1, 1) * span)
		goto_mrad(dev, target)
		t_slice = time.time() + dt
		while time.time() < t_slice:
			f = read_telem(dev, timeout_ms=max(int(dt * 500), 5))
			if f is None:
				continue
			if f["mode"] == 8:
				raise SystemExit("FAULT during sweep — abort")
			if f["mode"] == 9:
				rows.append(f)
	return rows


def analyze(rows: list[dict], revs: float) -> int:
	"""Return process exit code 0=ok-ish, 1=needs tuning, 2=bad data."""
	if len(rows) < 80:
		print(f"FAIL: too few samples ({len(rows)})")
		return 2

	uq = np.array([r["uq"] for r in rows], dtype=float)
	iq = np.array([r["iq"] for r in rows], dtype=float)
	iq_ref = np.array([r["iq_ref"] for r in rows], dtype=float)
	id_ = np.array([r["id"] for r in rows], dtype=float)
	ang = np.array([r["angle_mrad"] for r in rows], dtype=float) * 1e-3
	seq = np.array([r["seq"] for r in rows], dtype=np.uint16)
	vbus = np.array([r["vbus"] for r in rows], dtype=float)

	dseq = np.diff(seq.astype(np.int32))
	dseq = np.where(dseq < 0, dseq + 65536, dseq)
	seq_gaps = int(np.sum(dseq > 2))

	mech_span = (ang[-1] - ang[0]) / (2.0 * np.pi)
	# Closed loop: Uq is PI output (can be small); prefer |Iq_ref| mask when active.
	mask_ref = np.abs(iq_ref) > 0.05
	mask_uq = np.abs(uq) > 0.05
	closed = int(mask_ref.sum()) >= 40
	mask = mask_ref if closed else mask_uq
	if int(mask.sum()) < 40:
		print("FAIL: too few samples with |Uq| or |Iq_ref|>0.05 (motor not tracking?)")
		return 2

	same = float(np.mean(np.sign(uq[mask]) == np.sign(iq[mask]))) * 100.0
	prod = np.sign(uq[mask] * iq[mask])
	flips = int(np.sum(prod[1:] * prod[:-1] < 0))
	flips_per_rev = flips / max(abs(mech_span), 1e-3)
	iq_med = float(np.median(np.abs(iq[mask])))
	id_med = float(np.median(np.abs(id_[mask])))
	ratio = id_med / max(iq_med, 1e-6)
	vbus_med = float(np.median(vbus))

	track_same = track_err = None
	if closed:
		track_same = float(np.mean(np.sign(iq_ref[mask]) == np.sign(iq[mask]))) * 100.0
		track_err = float(np.median(np.abs(iq[mask] - iq_ref[mask])))

	# 2×fe signature of αβ DC bias (or reverse Park): Iq ~ cos(2·te)
	te = ang * float(POLE_PAIRS)
	c2 = np.cos(2.0 * te[mask])
	s2 = np.sin(2.0 * te[mask])
	iq_m = iq[mask] - np.mean(iq[mask])
	den = float(np.sqrt(np.mean(iq_m**2)) + 1e-9)
	corr_2fe = float(np.sqrt(np.mean(iq_m * c2) ** 2 + np.mean(iq_m * s2) ** 2) / den)

	print("--- FOC sense check ---")
	print(f"samples={len(rows)}  mech_span≈{mech_span:.2f} rev (cmd≈{revs})")
	print(f"vbus median={vbus_med:.2f} V  seq_gaps≈{seq_gaps}")
	print(f"mode: {'CLOSED-LOOP' if closed else 'sense-only'}  mask_n={int(mask.sum())}")
	print(f"sign(Iq)==sign(Uq): {same:.1f}%   (want >90% sense-only)")
	if track_same is not None:
		print(f"sign(Iq)==sign(Iq_ref): {track_same:.1f}%   (want >90% closed)")
		print(f"median |Iq-Iq_ref|={track_err:.3f} A")
	print(f"Iq/Uq sign flips: {flips}  (~{flips_per_rev:.1f} / mech rev)")
	print(f"median |Id|={id_med:.3f} A  |Iq|={iq_med:.3f} A  |Id|/|Iq|={ratio:.2f}  (want <<1)")
	print(f"Iq↔2×fe corr: {corr_2fe:.2f}  (want ≪0.5; ~1 ⇒ αβ DC / reverse Park)")

	rc = 0
	if vbus_med < 2.0:
		print("DIAG: Vbus looks clamped/wrong — check GPIO29 0.1×VCC wiring")
		rc = max(rc, 1)
	if seq_gaps > len(rows) // 20:
		print("DIAG: many seq gaps — USB/ISR load or host read too slow")
		rc = max(rc, 1)
	if corr_2fe > 0.55 and flips_per_rev > POLE_PAIRS * 0.5:
		print("DIAG: strong 2×fe in Iq — CSA offset/sample window or Park sense reverse")
		print("      firmware: synced v_off at mid-low; then CS_PHASE_ORD / CS_SIGN")
		rc = max(rc, 1)
	elif (not closed) and same < 80.0:
		print("DIAG: Park/phase map likely wrong (sign match poor)")
		print("      keep CS_TE_OFF=0 while sweeping CS_PHASE_ORD 0..5, then CS_SIGN ±1")
		print("      then fine CS_TE_OFF (≠0 tilts Iq off the voltage Uq axis)")
		rc = max(rc, 1)
	elif (not closed) and flips_per_rev > POLE_PAIRS * 1.5 and same < 90.0:
		print("DIAG: many Iq/Uq sign flips — TE_OFF / PHASE_ORD still off, or |Iq| in noise")
		rc = max(rc, 1)
	if closed and track_same is not None and track_same < 80.0:
		print("DIAG: Iq not following Iq_ref — CS frame, CS_SIGN, or PI gains")
		rc = max(rc, 1)
	if closed and track_err is not None and track_err > 0.35:
		print("DIAG: large |Iq-Iq_ref| — raise CUR_KP/KI carefully or lower IQ_CMD_A")
		rc = max(rc, 1)
	if ratio > 0.5:
		print("DIAG: |Id| too large vs |Iq| under motion — same PHASE_ORD / SIGN / TE_OFF tune")
		rc = max(rc, 1)
	if closed and track_same is not None and track_same >= 90.0 and ratio < 0.45 and (
		track_err is None or track_err < 0.25
	):
		print("OK: current loop tracking looks usable")
		rc = 0
	elif (not closed) and same >= 90.0 and ratio < 0.35:
		if flips_per_rev < POLE_PAIRS * 0.25:
			print("OK: sense frame looks usable for CUR_LOOP_CTRL=1 trial")
			rc = 0
		else:
			print("OK-ish: sign/ratio good; flips still noisy (small |Iq|) — try CUR_LOOP_CTRL=1 gently")
			rc = 0
	elif rc == 0:
		print("MARGINAL: usable but tune TE_OFF/ORD before closing the current loop")
		rc = 1
	return rc


def main() -> int:
	ap = argparse.ArgumentParser(description=__doc__)
	ap.add_argument("--revs", type=float, default=1.0, help="mechanical revolutions to sweep")
	ap.add_argument("--seconds", type=float, default=4.0, help="sweep duration")
	ap.add_argument("--save", type=str, default="", help="optional .npz path")
	ap.add_argument("--skip-start", action="store_true", help="assume already aligned/IDLE")
	ap.add_argument(
		"--no-bootloader",
		action="store_true",
		help="do not send UPLOAD (0x7F) after the test",
	)
	args = ap.parse_args()

	dev = open_dev()
	drain(dev)
	send(dev, 0x02)  # STOP → IDLE
	time.sleep(0.15)
	drain(dev)

	if not args.skip_start:
		print("START (align)...")
		send(dev, 0x01)
		f0 = wait_mode(dev, 0, timeout=25.0)
	else:
		f0 = wait_mode(dev, 0, timeout=3.0)
	a0 = f0["angle_mrad"]
	print(f"IDLE at {a0} mrad; GOTO sweep {args.revs} rev over {args.seconds}s...")

	try:
		rows = collect_sweep(dev, a0, args.revs, args.seconds)
	finally:
		try:
			send(dev, 0x02)
			print("STOP")
		except usb.core.USBError:
			print("STOP (USB already gone)")

	if args.save:
		np.savez(
			args.save,
			angle_mrad=np.array([r["angle_mrad"] for r in rows]),
			id=np.array([r["id"] for r in rows]),
			iq=np.array([r["iq"] for r in rows]),
			uq=np.array([r["uq"] for r in rows]),
			iq_ref=np.array([r["iq_ref"] for r in rows]),
			vbus=np.array([r["vbus"] for r in rows]),
			seq=np.array([r["seq"] for r in rows]),
			mode=np.array([r["mode"] for r in rows]),
		)
		print(f"saved {args.save}")

	rc = analyze(rows, args.revs)

	if not args.no_bootloader:
		print("UPLOAD → UF2 bootloader...")
		try:
			send(dev, 0x7F)
		except usb.core.USBError:
			pass  # disconnect is expected
		try:
			usb.util.dispose_resources(dev)
		except Exception:
			pass
		print("device should reappear as RPI-RP2; flash then re-run this script")

	return rc


if __name__ == "__main__":
	sys.exit(main())
