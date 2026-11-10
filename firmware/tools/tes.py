#!/usr/bin/env python3
"""Manual BTS7960 test using the same checksummed frames as the ROS2 bridge."""

import argparse
import select
import sys
import time

import serial

STEERING_LIMIT = 900
THROTTLE_LIMIT = 1000
PERIOD_S = 0.05
DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"


def command_frame(body: str) -> bytes:
    payload = body.encode("ascii")
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return b"@" + payload + f"*{checksum:02X}\n".encode("ascii")


def make_frame(steering: int, throttle: int) -> bytes:
    if not -STEERING_LIMIT <= steering <= STEERING_LIMIT:
        raise ValueError("Steering harus -900 sampai 900")
    if not -THROTTLE_LIMIT <= throttle <= THROTTLE_LIMIT:
        raise ValueError("Throttle harus -1000 sampai 1000")
    return command_frame(f"RC,{steering},{throttle}")


def parse_command(raw: str, steering: int, throttle: int):
    words = raw.strip().lower().split()
    if not words:
        return steering, throttle, None
    action = words[0]

    if len(words) == 1:
        if action in ("q", "quit", "exit"):
            return steering, throttle, "quit"
        if action in ("stop", "x"):
            return steering, 0, "drive"
        if action in ("center", "c"):
            return 0, throttle, "drive"
        if action in ("status", "pwm", "help", "arm", "disarm", "estop", "reset"):
            return steering, throttle, action
        if action == "start":
            return steering, throttle, "arm"

    try:
        if len(words) == 2 and action in ("s", "steer", "steering"):
            new_steering, new_throttle = int(words[1]), throttle
        elif len(words) == 2 and action in ("t", "throttle", "speed"):
            new_steering, new_throttle = steering, int(words[1])
        elif len(words) == 2:
            new_steering, new_throttle = int(words[0]), int(words[1])
        else:
            raise ValueError("Perintah tidak dikenali")
        make_frame(new_steering, new_throttle)
    except ValueError as error:
        raise ValueError(f"Input salah: {error}") from error
    return new_steering, new_throttle, "drive"


def help_text() -> str:
    return (
        "\nInput: <steering> <throttle>\n"
        "  steering: -900..900, 0 = tengah (GPIO13)\n"
        "  throttle: -1000..1000, negatif=motor mundur, positif=maju\n"
        "  0 0          tengah dan berhenti\n"
        "  -300 200     steering -300, maju 200\n"
        "  300 -150     steering 300, mundur 150\n"
        "  s -450       ubah steering saja\n"
        "  t 100        ubah throttle saja\n"
        "  arm / start  START robot (bisa juga tombol GPIO12 / Foxglove)\n"
        "  disarm       nonaktifkan motor\n"
        "  estop        emergency stop terkunci\n"
        "  reset        reset e-stop (tetap disarmed)\n"
        "  stop         throttle nol\n"
        "  center       luruskan servo\n"
        "  status       status arm, throttle dan tombol\n"
        "  pwm          servo pulse dan duty motor\n"
        "  quit         berhenti dan keluar\n"
    )


def drain_serial(port: serial.Serial, buffer: bytearray, last_ack: str) -> str:
    if not port.in_waiting:
        return last_ack
    buffer.extend(port.read(port.in_waiting))
    if len(buffer) > 1024:
        buffer.clear()
        print("\nRespons serial terlalu panjang", flush=True)
        return last_ack
    while b"\n" in buffer:
        raw, _, remaining = buffer.partition(b"\n")
        buffer[:] = remaining
        response = raw.decode("ascii", "replace").strip()
        if not response:
            continue
        if response.startswith("ACK,") and response == last_ack:
            continue
        if response.startswith("ACK,"):
            last_ack = response
        print(f"\nESP32: {response}", flush=True)
    return last_ack


def interactive(port_name: str, hold: float) -> int:
    steering = 0
    throttle = 0
    expire_at = 0.0
    received = bytearray()
    last_ack = ""
    port = serial.Serial(port_name, 115200, timeout=0,
                         write_timeout=0.3, exclusive=True)
    try:
        print("ESP32 RCauto | BTS7960 GPIO14 MAJU, GPIO27 MUNDUR")
        print("Steering GPIO13 | START tombol GPIO12 / ARM Foxglove")
        print(help_text())
        print(f"Port: {port_name}, batas perintah motor: {hold:g} detik.")
        print("Status awal DISARMED: tekan tombol START atau ketik arm.")
        next_send = 0.0
        prompt = True
        while True:
            now = time.monotonic()
            if throttle != 0 and now >= expire_at:
                throttle = 0
                print("\nTimeout pengujian: throttle=0", flush=True)
                prompt = True
            if now >= next_send:
                port.write(make_frame(steering, throttle))
                next_send = now + PERIOD_S
            last_ack = drain_serial(port, received, last_ack)
            if prompt:
                print(f"\n[steering={steering:+d}, throttle={throttle:+d}] > ",
                      end="", flush=True)
                prompt = False

            remaining = max(0.0, min(PERIOD_S, next_send - time.monotonic()))
            if not select.select([sys.stdin], [], [], remaining)[0]:
                continue
            raw = sys.stdin.readline()
            if not raw:
                break
            try:
                new_steering, new_throttle, action = parse_command(
                    raw, steering, throttle)
            except ValueError as error:
                print(error, flush=True)
                prompt = True
                continue

            if action == "quit":
                break
            if action == "help":
                print(help_text(), flush=True)
            elif action in ("status", "pwm"):
                port.write((action.upper() + "\n").encode("ascii"))
            elif action in ("arm", "disarm", "estop", "reset"):
                throttle = 0
                port.write(make_frame(steering, 0))
                body = {
                    "arm": "ARM,1",
                    "disarm": "ARM,0",
                    "estop": "ESTOP,1",
                    "reset": "RESET,1",
                }[action]
                port.write(command_frame(body))
            elif action == "drive":
                if new_throttle != 0:
                    expire_at = time.monotonic() + hold
                steering, throttle = new_steering, new_throttle
                print(f"Set steering={steering}, throttle={throttle}", flush=True)
            prompt = True
    finally:
        try:
            for _ in range(3):
                port.write(make_frame(0, 0))
                time.sleep(PERIOD_S)
            port.write(command_frame("ARM,0"))
        finally:
            port.close()
        print("\nKeluar: steering=0, throttle=0, robot DISARMED.", flush=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="RCauto BTS7960 manual test")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--hold", type=float, default=3.0)
    args = parser.parse_args()
    if not 0.5 <= args.hold <= 30.0:
        parser.error("--hold harus 0.5..30 detik")
    try:
        return interactive(args.port, args.hold)
    except KeyboardInterrupt:
        return 130
    except (serial.SerialException, OSError) as error:
        print(f"Gagal komunikasi: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
