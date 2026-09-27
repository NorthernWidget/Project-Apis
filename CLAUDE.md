# Project-Apis

The Apis: an ATtiny1634 carrying a Garmin LIDAR-Lite v3HP and an ST LIS3DH, presenting itself to a logger as a Schema 1 device. Hardware, firmware and documentation. `Apis_Library` is the logger-side half.

## Standards

Follow the NW standards in the root `CLAUDE.md` one level above `github/`, and the Schema 1 layouts in [NW-Device-Specification](https://github.com/NorthernWidget/NW-Device-Specification). Run `python3 ../NW-Tests/style_check.py .` before any commit touching `.ino/.cpp/.h`.

## The two buses, which are easy to confuse

The 1634 has the opposite role on each, and the part has **no controller-capable TWI at all**:

- **To the logger it is a peripheral**, at 0x41, through the chip's **hardware TWI peripheral** (`TWSA`, `TWSCRA`, `TWSCRB`, `TWSSRA`, `TWSD`, vector 25), driven by `WireS`. Not the USI - that is the Walrus's path, through `Wire`.
- **To its own chips it is the controller**, bit-banging PA2/PA3 with `SlowSoftI2CMaster`. `PIN_A2` and `PIN_A3` resolve to ATTinyCore pins 6 and 5, which map to PA2 and PA3.

## Facts established from the datasheet, which contradict a careless reading

The v3HP manual is in `Documentation/`. Read it as **pages**; its two-column layout flattens badly and has produced two wrong conclusions here.

- **Bit 7 of the register address is an auto-increment flag** and is stripped from the address (I2C Protocol Information, step 6). `0x81` addresses `0x01`. `readByte`/`writeByte` OR it in deliberately: that is **correct**, and the `FIX!!!` comment beside it is stale uncertainty, not a defect. Removing it breaks every LiDAR access.
- **"Writing any non-zero value initiates an acquisition"** (Acquisition Command, page 4). So `ACQ_COMMAND` = `0x01` acquires. Bit 0 *also* nominally requests a hard reset, but only once `LEGACY_RESET_EN` (0x06) arms it, and this firmware never writes 0x06.
- The **last NACK in a read is optional** on this part, though the formal I2C protocol wants it. The reads NACK their last byte as of 2026-09-24; that is conformance, not a repair.
- The mode pin cannot indicate busy: the board holds it high through **R11 (1 kΩ)**. Readiness comes from polling `STATUS` (0x01) bit 0. See issue #24.

## Open, for the bench

Whether `0x04` differs from `0x01` in selecting bias correction. The manual's worked sequence writes 0x04 and says any non-zero works; a commented-out line in `getRange()` believed 0x01 meant "with correction bias". Garmin's own Arduino library would settle it. Do not change the value on manual-reading alone.

## Hard rule

**Never** create a git tag, GitHub release, push to a shared remote, or close an issue unless explicitly asked in the current message. If in doubt, ask.
