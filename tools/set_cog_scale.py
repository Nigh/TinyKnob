#!/usr/bin/env python3
"""Set SPIN_COG_FF_SCALE, build, flash, align, and leave TinyKnob in SPIN."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import subprocess
import sys
import time

try:
	import usb.core
	import usb.util
except ImportError:
	sys.exit("need pyusb: pip install pyusb")

ROOT = Path(__file__).resolve().parents[1]
PINS = ROOT / "src" / "pins.h"
UF2 = ROOT / "build" / "src" / "RP2350_TinyKnob.uf2"
APP_VID, APP_PID = 0xACDC, 0x4011
BOOT_VID, BOOT_PID = 0x2E8A, 0x000F
SCALE_RE = re.compile(r"^(#define SPIN_COG_FF_SCALE )[^ ]+(.*)$", re.MULTILINE)


def run(*args: str) -> None:
	print("+", " ".join(args), flush=True)
	subprocess.run(args, cwd=ROOT, check=True)


def set_scale(scale: float) -> None:
	text = PINS.read_text(encoding="utf-8")
	match = SCALE_RE.search(text)
	if match is None:
		raise SystemExit(f"SPIN_COG_FF_SCALE not found in {PINS}")
	replacement = f"{match.group(1)}{scale:.3f}f{match.group(2)}"
	updated = text[: match.start()] + replacement + text[match.end() :]
	tmp = PINS.with_suffix(".h.tmp")
	tmp.write_text(updated, encoding="utf-8")
	os.replace(tmp, PINS)
	print(f"SPIN_COG_FF_SCALE={scale:.3f}", flush=True)


def wait_usb(vid: int, pid: int, timeout: float, label: str):
	deadline = time.monotonic() + timeout
	while time.monotonic() < deadline:
		dev = usb.core.find(idVendor=vid, idProduct=pid)
		if dev is not None:
			return dev
		time.sleep(0.2)
	raise SystemExit(f"timeout waiting for {label} ({vid:04x}:{pid:04x})")


def enter_bootloader() -> None:
	if usb.core.find(idVendor=BOOT_VID, idProduct=BOOT_PID) is not None:
		print("RP2350 already in UF2 bootloader", flush=True)
		return
	if usb.core.find(idVendor=APP_VID, idProduct=APP_PID) is None:
		raise SystemExit("neither TinyRoller app nor RP2350 bootloader was found")

	sys.path.insert(0, str(ROOT / "tools"))
	import feel_check

	print("UPLOAD -> UF2", flush=True)
	for attempt in range(1, 4):
		dev = feel_check.open_dev()
		status = feel_check.send_ack(dev, 0x02, timeout_ms=500)
		if status != 1:
			print(f"  attempt {attempt}: STOP ack={status}", flush=True)
		feel_check.drain(dev)
		time.sleep(0.1)
		try:
			feel_check.send(dev, 0x7F)
		except usb.core.USBError:
			pass
		usb.util.dispose_resources(dev)
		deadline = time.monotonic() + 3.0
		while time.monotonic() < deadline:
			if usb.core.find(idVendor=BOOT_VID, idProduct=BOOT_PID) is not None:
				return
			time.sleep(0.2)
	raise SystemExit("UPLOAD failed after 3 STOP/ACK attempts")


def find_mount(timeout: float) -> Path:
	configured = os.environ.get("RP2350_MOUNT")
	deadline = time.monotonic() + timeout
	while time.monotonic() < deadline:
		if configured:
			path = Path(configured)
			if path.is_mount():
				return path
		try:
			out = subprocess.run(
				("findmnt", "-rn", "-t", "vfat", "-o", "TARGET,LABEL"),
				text=True, capture_output=True, check=False,
			).stdout
			for line in out.splitlines():
				target, _, label = line.rpartition(" ")
				if label == "RP2350" and Path(target).is_mount():
					return Path(target)
		except FileNotFoundError:
			path = Path("/Volumes/RP2350")
			if path.is_mount():
				return path
		time.sleep(0.2)
	raise SystemExit("RP2350 bootrom found but no mounted filesystem labeled RP2350 appeared")


def flash() -> None:
	if not UF2.is_file():
		raise SystemExit(f"missing firmware: {UF2}")
	mount = find_mount(15.0)
	dst = mount / "fw.uf2"
	print(f"flash {UF2.relative_to(ROOT)} -> {dst}", flush=True)
	copy_cmd = ("cp", str(UF2), str(dst))
	try:
		subprocess.run(copy_cmd, check=True)
	except subprocess.CalledProcessError as exc:
		raise SystemExit(f"cannot write the real RP2350 mount {dst}: cp exit {exc.returncode}")
	subprocess.run(("sync",), check=True)
	wait_usb(APP_VID, APP_PID, 15.0, "TinyRoller app")
	if usb.core.find(idVendor=BOOT_VID, idProduct=BOOT_PID) is not None:
		raise SystemExit("TinyRoller app appeared but RP2350 bootloader did not disconnect")


def align_and_spin() -> None:
	sys.path.insert(0, str(ROOT / "tools"))
	import feel_check

	dev = feel_check.open_dev()
	feel_check.align(dev)
	status = feel_check.send_ack(dev, 0x04)
	if status != 1:
		raise SystemExit(f"SPIN rejected after align (status={status})")
	print("ready: aligned and SPIN active", flush=True)


def main() -> int:
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument("scale", type=float, help="COG compensation scale, e.g. 0.60")
	args = parser.parse_args()
	if not 0.0 <= args.scale <= 2.0:
		parser.error("scale must be in [0, 2]")

	set_scale(args.scale)
	run("make", "docker_build")
	enter_bootloader()
	flash()
	align_and_spin()
	return 0


if __name__ == "__main__":
	sys.exit(main())
