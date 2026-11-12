#!/usr/bin/env python3
import math
import pathlib
import sys

WIDTH, HEIGHT = 256, 128


def radiance(longitude, latitude):
    up = math.sin(latitude)
    value = 0.32 + 0.25 * max(up, 0.0) if up >= 0.0 else 0.32 + 0.24 * up
    boxes = ((math.radians(-60.0), math.radians(25.0), 0.35, 0.22, 7.0),
             (math.radians(120.0), math.radians(20.0), 0.30, 0.20, 4.0),
             (math.radians(30.0), math.radians(70.0), 0.70, 0.18, 3.0))
    for centre_longitude, centre_latitude, half_width, half_height, intensity in boxes:
        dl = math.atan2(math.sin(longitude - centre_longitude), math.cos(longitude - centre_longitude))
        inside = max(0.0, 1.0 - (abs(dl) / half_width) ** 6 - (abs(latitude - centre_latitude) / half_height) ** 6)
        value += intensity * min(1.0, inside * 4.0)
    return value


def rgbe(red, green, blue):
    brightest = max(red, green, blue)
    if brightest < 1e-32:
        return bytes((0, 0, 0, 0))
    mantissa, exponent = math.frexp(brightest)
    scale = mantissa * 256.0 / brightest
    return bytes((int(red * scale), int(green * scale), int(blue * scale), exponent + 128))


def encode_scanline(pixels):
    out = bytearray((2, 2, WIDTH >> 8, WIDTH & 0xFF))
    for channel in range(4):
        values = bytes(pixel[channel] for pixel in pixels)
        for start in range(0, WIDTH, 128):
            chunk = values[start:start + 128]
            out.append(len(chunk))
            out.extend(chunk)
    return bytes(out)


def main(path):
    lines = [b"#?RADIANCE\n", b"FORMAT=32-bit_rle_rgbe\n\n", f"-Y {HEIGHT} +X {WIDTH}\n".encode()]
    body = bytearray()
    for row in range(HEIGHT):
        latitude = math.pi / 2.0 - (row + 0.5) * math.pi / HEIGHT
        pixels = []
        for column in range(WIDTH):
            longitude = (column + 0.5) * 2.0 * math.pi / WIDTH - math.pi
            value = radiance(longitude, latitude)
            pixels.append(rgbe(value, value * 0.98, value * 0.95))
        body.extend(encode_scanline(pixels))
    pathlib.Path(path).write_bytes(b"".join(lines) + bytes(body))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else str(pathlib.Path(__file__).with_name("studio.hdr")))
