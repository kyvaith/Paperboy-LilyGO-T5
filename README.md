# Paperboy LilyGO T5 4.7 e-Paper

Paperboy LilyGO T5 is an ESP32-S3 Game Boy emulator adapted for the
[LILYGO T5 4.7 inch e-Paper](https://lilygo.cc/en-us/products/t5-4-7-inch-e-paper-v2-3)
board.

This repository is an adaptation of Wenting Zhang's Paperboy project:
https://gitlab.com/zephray/paperboy

The port replaces the original display path with the LILYGO ED047TC1 e-paper
driver, adds board-specific SD, battery and GT911 touch wiring, and includes a
USB serial control fallback so the emulator can be tested even before the touch
coordinate map is fully calibrated.

## Current Status

- Target board: LILYGO T5 4.7 inch e-Paper ESP32-S3, 16 MB flash, 8 MB PSRAM.
- Display: ED047TC1 e-paper panel driven through the LILYGO I80/I2S-style
  e-paper driver.
- Touch: GT911 is initialized on the LILYGO touch pins and is detected on the
  tested board at I2C address `0x5D`.
- Input: USB-Serial/JTAG controls over the same COM port used for flashing.
- SD card: optional for first boot; required for loading your own ROM files and
  for persistent save-state storage.
- Built-in test ROM: GB Corp, a small MIT-licensed homebrew Game Boy ROM, is
  embedded for smoke testing when no SD card is mounted.

## Controls

Open a serial monitor at `115200` baud on the board COM port.

```powershell
python -m platformio device monitor -p COM6 -b 115200
```

Keyboard mapping:

| Key | Game Boy input |
| --- | --- |
| `W`, arrow up | Up |
| `S`, arrow down | Down |
| `A`, arrow left | Left |
| `D`, arrow right | Right |
| `J` or `X` | A |
| `K` or `Z` | B |
| `Enter` | Start |
| `Space` | Select |
| `P` | Save state, SD-backed ROMs only |
| `R` | Load state, SD-backed ROMs only |
| `C` | Clear ghosting |

## SD Card Usage

The firmware boots without an SD card by running the embedded GB Corp test ROM.
To load your own games, use a FAT32-formatted microSD card and place `.gb` or
`.gbc` files in the root directory or one directory level below it.

When SD mounting succeeds, the on-device picker lists ROM files and audio
settings. Save states and SRAM persistence are written next to the selected ROM.
When the built-in ROM is running, save and load commands are ignored because
there is no writable SD-backed ROM path.

## Build and Flash

The project uses PlatformIO with ESP-IDF.

Install PlatformIO, connect the board over USB, then build and flash:

```powershell
python -m platformio run -e lilygo-t5-epaper-s3
python -m platformio run -e lilygo-t5-epaper-s3 -t upload --upload-port COM6
```

Use the actual COM port assigned by your system if it is not `COM6`.

If PlatformIO or CMake has trouble with spaces in the checkout path, clone the
repository into a path without spaces.

## Repository Layout

- `main/` - emulator application, display pipeline, input, audio, touch and
  board support.
- `main/lilygo_epd/` - LILYGO ED047TC1 display driver code used by this port.
- `assets/gbcorp.gb` - embedded homebrew test ROM source asset.
- `boards/T5-ePaper-S3.json` - PlatformIO board definition.
- `platformio.ini` - PlatformIO build environment.
- `partitions.csv` - 16 MB flash partition layout.

## Test ROM

The embedded smoke-test ROM is GB Corp by Dr. Ludos. It is listed by Homebrew
Hub as MIT licensed:

- https://hh.gbdev.io/game/gb-corp/
- https://github.com/drludos/GBcorp

Only use ROM files that you have the legal right to use.

## Notes

This is still a hardware adaptation, not a polished handheld product. The
display path, serial input and built-in ROM fallback are working on the tested
LILYGO board. Touch is detected, but the touch button zones may need additional
coordinate calibration for comfortable standalone use.
