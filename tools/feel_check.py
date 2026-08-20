#!/usr/bin/env python3
"""TinyKnob feel smoke: align → telem rate / TEST / SPRING / SPIN (+ optional cog learn gate).

Exit codes:
  0 = ok
  1 = telem/mode/SET_K/learn failure
  2 = warn-only (SPIN rest jitter above threshold; telem still healthy)

Usage:
  python3 tools/feel_check.py --no-bootloader
  python3 tools/feel_check.py --cog-learn --no-bootloader   # also run host-learn gates (no write)
"""
from __future__ import annotations

import argparse
import math
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
TELEM_FMT = "<BBi3hHhhhHh"
MRAD_2PI = 6283

MODE_IDLE = 0
MODE_TEST = 5
MODE_SPRING = 6
MODE_SPIN = 7

TELEM_MIN_HZ = 800
SPIN_JITTER_WARN_MRAD = 15.0


def open_dev():
	dev = usb.core.find(idVendor=VID, idProduct=PID)
	if dev is None:
		raise SystemExit("TinyKnob not found (VID=0xACDC PID=0x4011)")
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
	except usb.core.USBError:
		pass
	cfg = dev.get_active_configuration()
	intf = usb.util.find_descriptor(cfg, bInterfaceClass=0xFF)
	if intf is None:
		raise SystemExit("Vendor interface not found")
	usb.util.claim_interface(dev, intf.bInterfaceNumber)
	return dev


def send(dev, *payload: int) -> None:
	dev.write(EP_OUT, bytes(payload), timeout=2000)


def send_ack(dev, *payload: int, timeout_ms: int = 200) -> int | None:
	"""Send cmd; return ACK status (1/0) or None if no ACK seen."""
	send(dev, *payload)
	t0 = time.time()
	while time.time() - t0 < timeout_ms / 1000.0:
		try:
			data = bytes(dev.read(EP_IN, 64, timeout=30))
		except usb.core.USBTimeoutError:
			continue
		if len(data) >= 3 and data[0] == 0x5A and data[1] == payload[0]:
			return int(data[2])
	return None


def read_telem(dev, timeout_ms: int = 50):
	try:
		data = bytes(dev.read(EP_IN, 64, timeout=timeout_ms))
	except usb.core.USBTimeoutError:
		return None
	if len(data) >= 3 and data[0] == 0x5A:
		return None
	if len(data) < 24 or data[0] != 0xA5:
		return None
	(_magic, mode, ang, _da, _db, _dc, _seq, _id, _iq, _iq_ref, _vbus, _uq) = struct.unpack_from(
		TELEM_FMT, data, 0
	)
	return {"mode": mode, "angle_mrad": ang}


def drain(dev, n: int = 40) -> None:
	for _ in range(n):
		read_telem(dev, timeout_ms=15)


def wait_mode(dev, want: int, timeout: float):
	t0 = time.time()
	last = None
	while time.time() - t0 < timeout:
		f = read_telem(dev, timeout_ms=100)
		if f is None:
			continue
		last = f["mode"]
		if f["mode"] == want:
			return f
	raise SystemExit(f"timeout waiting mode={want} (last={last})")


def align(dev) -> None:
	send(dev, 0x02)
	time.sleep(0.15)
	drain(dev, 40)
	print("START (align)...")
	send(dev, 0x01)
	t0 = time.time()
	while time.time() - t0 < 5.0:
		f = read_telem(dev, timeout_ms=100)
		if f is None:
			continue
		if f["mode"] != MODE_IDLE:
			break
	else:
		raise SystemExit("START did not leave IDLE")
	wait_mode(dev, MODE_IDLE, timeout=35.0)
	time.sleep(0.25)
	drain(dev, 40)
	wait_mode(dev, MODE_IDLE, timeout=3.0)
	print("armed IDLE")


def telem_hz(dev, seconds: float = 1.0) -> tuple[float, dict[int, int]]:
	n = 0
	modes: dict[int, int] = {}
	t1 = time.time() + seconds
	while time.time() < t1:
		f = read_telem(dev, timeout_ms=30)
		if f is None:
			continue
		n += 1
		modes[f["mode"]] = modes.get(f["mode"], 0) + 1
	return n / seconds, modes


def expect_mode_hz(dev, label: str, want_mode: int, seconds: float = 1.0) -> float:
	hz, modes = telem_hz(dev, seconds)
	dom = max(modes, key=modes.get) if modes else None
	print(f"  {label}: {hz:.0f} Hz modes={modes}")
	if hz < TELEM_MIN_HZ:
		raise SystemExit(f"FAIL {label}: telem {hz:.0f} Hz < {TELEM_MIN_HZ}")
	if want_mode is not None and dom != want_mode:
		raise SystemExit(f"FAIL {label}: dominant mode {dom} want {want_mode}")
	return hz


def spin_rest_sigma_mrad(dev, seconds: float = 2.0) -> float:
	angs = []
	t1 = time.time() + seconds
	while time.time() < t1:
		f = read_telem(dev, timeout_ms=30)
		if f is None or f["mode"] != MODE_SPIN:
			continue
		angs.append(float(f["angle_mrad"]))
	if len(angs) < TELEM_MIN_HZ:
		raise SystemExit(f"FAIL SPIN jitter: only {len(angs)} samples in {seconds}s")
	return float(np.std(np.asarray(angs, dtype=np.float64)))


def main() -> int:
	ap = argparse.ArgumentParser(description=__doc__)
	ap.add_argument("--bootloader", action="store_true", help="UPLOAD to UF2 at end (default: leave app)")
	ap.add_argument("--cog-learn", action="store_true", help="run host-learn fill/peak gates (no write)")
	ap.add_argument("--skip-start", action="store_true")
	args = ap.parse_args()
	args.no_bootloader = not args.bootloader

	warn = False
	dev = open_dev()
	if not args.skip_start:
		align(dev)
	else:
		wait_mode(dev, MODE_IDLE, timeout=3.0)

	print("telem IDLE...")
	expect_mode_hz(dev, "IDLE", MODE_IDLE)

	print("TEST...")
	st = send_ack(dev, 0x05)
	if st == 0:
		raise SystemExit("FAIL TEST rejected (need START)")
	time.sleep(0.15)
	expect_mode_hz(dev, "TEST", MODE_TEST)
	send(dev, 0x02)
	wait_mode(dev, MODE_IDLE, timeout=3.0)

	print("SPRING + SET_K...")
	st = send_ack(dev, 0x03)
	if st == 0:
		raise SystemExit("FAIL SPRING rejected")
	time.sleep(0.1)
	expect_mode_hz(dev, "SPRING", MODE_SPRING, seconds=0.8)
	for kx10 in (6, 20):
		st = send_ack(dev, 0x20, kx10)
		if st == 0:
			raise SystemExit(f"FAIL SET_K {kx10} rejected")
		time.sleep(0.05)
		f = read_telem(dev, timeout_ms=100)
		if f is None or f["mode"] != MODE_SPRING:
			raise SystemExit(f"FAIL SET_K {kx10}: left SPRING")
	print("  SET_K 6→20 ok")
	send(dev, 0x02)
	wait_mode(dev, MODE_IDLE, timeout=3.0)

	print("SPIN rest jitter...")
	st = send_ack(dev, 0x04)
	if st == 0:
		raise SystemExit("FAIL SPIN rejected")
	time.sleep(0.15)
	hz, _ = telem_hz(dev, 0.5)
	if hz < TELEM_MIN_HZ:
		raise SystemExit(f"FAIL SPIN: telem {hz:.0f} Hz")
	sig = spin_rest_sigma_mrad(dev, 2.0)
	print(f"  σ(angle)={sig:.2f} mrad (warn>{SPIN_JITTER_WARN_MRAD})")
	if sig > SPIN_JITTER_WARN_MRAD:
		print("WARN SPIN rest jitter high (detent/hunt) — bake cog or tune SPIN_B")
		warn = True
	send(dev, 0x02)
	wait_mode(dev, MODE_IDLE, timeout=3.0)

	if args.cog_learn:
		print("cog host-learn gate (no write)...")
		# Import after path setup
		import pathlib

		sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
		import cog_cal

		lut = cog_cal.host_learn(dev, seconds=20.0, revs=2.0, invert=False)
		print(f"  learn ok peak={float(np.max(np.abs(lut))):.4f}")

	print("try SPIN/SPRING by hand: multi-turn spring; detent after cog bake")
	if not args.no_bootloader:
		print("UPLOAD → UF2")
		try:
			send(dev, 0x7F)
		except usb.core.USBError:
			pass
	return 2 if warn else 0


if __name__ == "__main__":
	sys.exit(main())
