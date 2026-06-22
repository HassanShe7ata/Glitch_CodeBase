# Glitch Project — ESP32 Node Registry

## Current UDP network

The current implementation uses the Base as the WiFi AP and UDP coordinator.
Arm and Camera join `GLITCH` with static IPs:

| Node   | Role | Static IP | UDP port |
|--------|------|-----------|----------|
| Base   | AP, dashboard, UDP coordinator | `192.168.4.1` | `4210` |
| Arm    | Command receiver / ACK sender | `192.168.4.201` | `4210` |
| Camera | Scan receiver / pose sender | `192.168.4.202` | `4210` |

The high Arm/Camera addresses avoid collisions with the Base AP DHCP range
used by phones and laptops.

## Historical MAC registry

Verified MAC addresses for the three ESP32 nodes in the Glitch robot fleet.
All values are LSB-first (`WiFi.macAddress()` order, the same order ESP-NOW
expects inside a `uint8_t[6]`).

## Verified (efuse read on 2026-06-11)

| Node    | Board / Chip          | COM port (as found) | MAC (canonical, MSB-first) | MAC bytes (LSB-first, for code) |
|---------|-----------------------|---------------------|----------------------------|----------------------------------|
| Camera  | ESP32-S3 (QFN56)      | COM3 (CH340)        | `94:A9:90:08:B2:B8`        | `{0xB8, 0xB2, 0x08, 0x90, 0xA9, 0x94}` |
| Arm     | ESP32-D0WD-V3 (rev 3.1) | COM15 (CP210x)     | `68:FE:71:12:5D:A8`        | `{0xA8, 0x5D, 0x12, 0x71, 0xFE, 0x68}` |
| Base    | ESP32 (trust, prior test) | n/a — not enumerable 2026-06-11 | `80:F3:DA:42:3E:5C` (STA) / `80:F3:DA:42:3E:5D` (AP) | `{0x5C, 0x3E, 0x42, 0xDA, 0xF3, 0x80}` (STA) / `{0x5D, 0x3E, 0x42, 0xDA, 0xF3, 0x80}` (AP) |

> **Note:** The ESP32 has separate MAC addresses for STA and AP interfaces. The efuse read returns the STA MAC (`0x5C`). The AP MAC is STA + 1 (`0x5D`). ESP-NOW peers must use the AP MAC since the Base operates as `WIFI_IF_AP`. All active firmware correctly uses `0x5D` for the Base.

> Historical note: the old ESP-NOW code stored MACs MSB-first in `uint8_t[]`
> arrays (e.g. `{0x94, 0xA9, 0x90, 0x08, 0xB2, 0xB8}`). The current UDP
> implementation does not use peer MAC arrays.

## Historical cross-reference

| Stored MAC | Referenced in | Variable |
|------------|---------------|----------|
| Camera `94:A9:90:08:B2:B8` | historical ESP-NOW implementation | `cameraAddress` |
| Arm    `68:FE:71:12:5D:A8` | historical ESP-NOW implementation | `armAddress` |
| Base   `80:F3:DA:42:3E:5C` | historical ESP-NOW implementation | `baseAddress` |
| Base   `80:F3:DA:42:3E:5C` | historical ESP-NOW implementation | `baseMacAddress` |

Each board also self-identifies in its own firmware at boot:

- `base.cpp` — prints Base AP IP and UDP node addresses.
- `arm/arm.cpp` — prints `ARM MAC`, `ARM IP`, and WiFi channel.
- `camera/src/main.cpp` — prints `CAMERA MAC`, `CAMERA IP`, and WiFi channel.

So a re-verify is one flash + monitor away on every node.

## Re-verify procedure

```powershell
# Plug in board, replace COMx with what shows up after `list-ports`
.\esp-flash.ps1 -Action read-mac -Port COMx -Chip esp32       # for ESP32 (Base, Arm)
.\esp-flash.ps1 -Action read-mac -Port COMx -Chip esp32s3     # for S3 (Camera)
```

No flashing required — `esptool` reads the efuse directly, faster and zero
risk to the existing firmware.

## Status timeline

- 2026-06-11: Camera MAC verified via `esptool read_mac` on COM3.
- 2026-06-11: Arm MAC verified via `esptool read_mac` on COM15.
- 2026-06-11: Base not enumerable on USB at start of testing session;
  trusted user-confirmed MAC `80:F3:DA:42:3E:5C` (last flashed and tested
  successfully). Re-verify next time the Base enumerates.
