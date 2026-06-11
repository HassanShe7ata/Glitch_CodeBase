# Glitch Project — ESP32 Node Registry

Verified MAC addresses for the three ESP32 nodes in the Glitch robot fleet.
All values are LSB-first (`WiFi.macAddress()` order, the same order ESP-NOW
expects inside a `uint8_t[6]`).

## Verified (efuse read on 2026-06-11)

| Node    | Board / Chip          | COM port (as found) | MAC (canonical, MSB-first) | MAC bytes (LSB-first, for code) |
|---------|-----------------------|---------------------|----------------------------|----------------------------------|
| Camera  | ESP32-S3 (QFN56)      | COM3 (CH340)        | `94:A9:90:08:B2:B8`        | `{0xB8, 0xB2, 0x08, 0x90, 0xA9, 0x94}` |
| Arm     | ESP32-D0WD-V3 (rev 3.1) | COM15 (CP210x)     | `68:FE:71:12:5D:A8`        | `{0xA8, 0x5D, 0x12, 0x71, 0xFE, 0x68}` |
| Base    | ESP32 (trust, prior test) | n/a — not enumerable 2026-06-11 | `80:F3:DA:42:3E:5C` | `{0x5C, 0x3E, 0x42, 0xDA, 0xF3, 0x80}` |

> Note: the **code** files store them MSB-first in the `uint8_t[]` arrays
> (e.g. `{0x94, 0xA9, 0x90, 0x08, 0xB2, 0xB8}`). Both representations are
> equivalent — `memcpy(peer_addr, ..., 6)` doesn't care which way you read it.
> Just be consistent within a single file.

## Cross-reference — where each MAC is referenced

| Stored MAC | Referenced in | Variable |
|------------|---------------|----------|
| Camera `94:A9:90:08:B2:B8` | `basewithBlynk.ino:54` | `cameraAddress` |
| Arm    `68:FE:71:12:5D:A8` | `basewithBlynk.ino:39` | `armAddress` |
| Base   `80:F3:DA:42:3E:5C` | `ArmwithBlynk.ino:21` | `baseAddress` |
| Base   `80:F3:DA:42:3E:5C` | `firmware/cam_stream/src/main.cpp:68` | `baseMacAddress` |

Each board also self-identifies in its own firmware at boot:

- `basewithBlynk.ino:733-734` — prints `BASE MAC: ...`
- `ArmwithBlynk.ino:501-502` — prints `ARM MAC: ...`
- `firmware/cam_stream/src/main.cpp:1499-1500` — prints `CAMERA MAC: ...`

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
