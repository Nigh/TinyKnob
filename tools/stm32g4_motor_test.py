#!/usr/bin/env python3
"""Test STM32G4 FOC and motor modes over TinyRoller Vendor Bulk USB."""
from __future__ import annotations

import argparse
import struct
import sys
import time

try:
	import usb.core
	import usb.util
except ImportError:
	sys.exit("need pyusb: pip install pyusb")

VID, PID = 0xACDC, 0x4011
EP_OUT, EP_IN = 0x01, 0x81
PRODUCT = "TinyRoller STM32G4"
TELEM_FMT = "<BBi3hHhhhHhb"


def open_device():
	devices = list(usb.core.find(find_all=True, idVendor=VID, idProduct=PID) or [])
	for dev in devices:
		try:
			if usb.util.get_string(dev, dev.iProduct) == PRODUCT:
				break
		except (ValueError, usb.core.USBError):
			continue
	else:
		raise SystemExit(f"{PRODUCT} not found (VID=0x{VID:04X} PID=0x{PID:04X})")

	try:
		dev.set_configuration()
	except usb.core.USBError:
		pass
	cfg = dev.get_active_configuration()
	intf = usb.util.find_descriptor(cfg, bInterfaceClass=0xFF)
	if intf is None:
		raise SystemExit("Vendor Bulk interface not found")
	n = intf.bInterfaceNumber
	try:
		if dev.is_kernel_driver_active(n):
			dev.detach_kernel_driver(n)
	except (NotImplementedError, usb.core.USBError):
		pass
	usb.util.claim_interface(dev, n)
	return dev, n


def read_packet(dev, timeout_ms=100):
	try:
		return bytes(dev.read(EP_IN, 64, timeout=timeout_ms))
	except usb.core.USBTimeoutError:
		return None


def send_ack(dev, cmd: int, *payload: int, timeout_s=1.0) -> bool:
	dev.write(EP_OUT, bytes([cmd, *payload]), timeout=1000)
	deadline = time.monotonic() + timeout_s
	while time.monotonic() < deadline:
		packet = read_packet(dev)
		if packet and len(packet) >= 3 and packet[0] == 0x5A and packet[1] == cmd:
			return packet[2] == 1
	return False


def read_diag(dev):
	dev.write(EP_OUT, b"\x30", timeout=1000)
	deadline = time.monotonic() + 1.0
	while time.monotonic() < deadline:
		packet = read_packet(dev)
		if packet and len(packet) >= 19 and packet[0] == 0x5C:
			return {
				"mode": packet[2], "fault": packet[3],
				"raw24": int.from_bytes(packet[4:7], "big"),
				"crc_ok": int.from_bytes(packet[7:11], "little"),
				"crc_fail": int.from_bytes(packet[11:15], "little"),
				"crc_run": int.from_bytes(packet[15:19], "little"),
			}
	return None


def telemetry(packet):
	if not packet or len(packet) < struct.calcsize(TELEM_FMT) or packet[0] != 0xA5:
		return None
	values = struct.unpack_from(TELEM_FMT, packet)
	return {
		"mode": values[1], "angle": values[2], "seq": values[6],
		"id": values[7], "iq": values[8], "iq_ref": values[9],
		"vbus": values[10], "uq": values[11], "dir": values[12],
	}


def wait_for(dev, predicate, timeout_s: float, label: str):
	deadline = time.monotonic() + timeout_s
	last = None
	while time.monotonic() < deadline:
		frame = telemetry(read_packet(dev))
		if frame:
			last = frame
			if frame["mode"] == 8:
				diag = read_diag(dev)
				raise RuntimeError(f"device entered FAULT while waiting for {label}; diagnostic={diag}")
			if predicate(frame):
				return frame
	raise RuntimeError(f"timeout waiting for {label}; last telemetry={last}; diagnostic={read_diag(dev)}")


def main() -> int:
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument("--seconds", type=float, default=12.0,
		help="observe TEST motion for this many seconds (default: 12)")
	parser.add_argument("--all-modes", action="store_true", help="also smoke-test SPRING, SPIN, POS, STRESS, and GEAR")
	parser.add_argument("--bootloader", action="store_true", help="send 0x7F and exit; device should enumerate as 0483:DF11")
	args = parser.parse_args()
	dev, intf = open_device()
	print(f"connected: {PRODUCT}, vendor interface {intf}, OUT 0x01 / IN 0x81")
	if args.bootloader:
		dev.write(EP_OUT, b"\x7f", timeout=1000)
		usb.util.dispose_resources(dev)
		deadline = time.monotonic() + 5.0
		while time.monotonic() < deadline:
			if usb.core.find(idVendor=0x0483, idProduct=0xDF11) is not None:
				print("PASS: STM32 ROM DFU enumerated as 0483:DF11")
				return 0
			time.sleep(0.1)
		print("FAIL: STM32 ROM DFU did not enumerate as 0483:DF11", file=sys.stderr)
		return 1
	try:
		if not send_ack(dev, 0x02):
			raise RuntimeError("STOP was not acknowledged")
		if not send_ack(dev, 0x01):
			raise RuntimeError("START was not acknowledged")
		wait_for(dev, lambda f: f["mode"] != 0, 3.0, "alignment to start")
		wait_for(dev, lambda f: f["mode"] == 0, 5.0, "READY after alignment")
		print("alignment complete; motor READY")
		if not send_ack(dev, 0x05):
			raise RuntimeError("TEST was rejected")

		deadline = time.monotonic() + args.seconds
		angles, samples, frames, test_frames = [], [], 0, 0
		while time.monotonic() < deadline:
			frame = telemetry(read_packet(dev))
			if not frame:
				continue
			frames += 1
			angles.append(frame["angle"])
			samples.append(frame)
			test_frames += frame["mode"] == 5
		span = max(angles) - min(angles) if angles else 0
		rate = frames / args.seconds
		print(f"TEST: {frames} frames ({rate:.0f} Hz), mode=5 frames {test_frames}, angle span {span} mrad")
		if not frames or not test_frames:
			raise RuntimeError("no TEST telemetry received")
		if span < 500:
			raise RuntimeError("encoder movement too small (<500 mrad)")
		active = [f for f in samples if abs(f["iq_ref"]) >= 100]
		match = sum((f["iq"] > 0) == (f["iq_ref"] > 0) for f in active) / len(active) if active else 0.0
		if not active or match < 0.70:
			raise RuntimeError(f"current-loop sign match too low ({match:.1%})")
		if max((f["vbus"] for f in samples), default=0) < 1000:
			raise RuntimeError("Vbus telemetry is missing")
		print(f"current-loop sign match: {match:.1%}")
		if args.all_modes:
			for cmd, mode, name, hold in ((0x03, 6, "SPRING", 2.5), (0x04, 7, "SPIN", 1.0)):
				if not send_ack(dev, cmd): raise RuntimeError(f"{name} rejected")
				wait_for(dev, lambda f, m=mode: f["mode"] == m, 1.0, name)
				time.sleep(hold)
			if not send_ack(dev, 0x02): raise RuntimeError("STOP before POS failed")
			angle = angles[-1] + 300
			dev.write(EP_OUT, b"\x06" + struct.pack("<i", angle), timeout=1000)
			wait_for(dev, lambda f: f["mode"] == 9, 1.0, "POS")
			for cmd, mode, name, hold in ((0x07, 10, "STRESS", 1.0), (0x0A, 12, "GEAR", 1.5)):
				if not send_ack(dev, cmd): raise RuntimeError(f"{name} rejected")
				wait_for(dev, lambda f, m=mode: f["mode"] == m, 1.0, name)
				time.sleep(hold)
			print("all requested modes entered successfully")
		print("PASS: Vendor Bulk, FOC telemetry, current loop, and encoder motion verified")
		return 0
	except (RuntimeError, usb.core.USBError) as exc:
		print(f"FAIL: {exc}", file=sys.stderr)
		return 1
	finally:
		try:
			send_ack(dev, 0x02, timeout_s=0.3)
		except usb.core.USBError:
			pass
		usb.util.dispose_resources(dev)


if __name__ == "__main__":
	raise SystemExit(main())
