# Changelog

All notable changes to AM36-WildCam are documented in this file.

The project follows [Semantic Versioning](https://semver.org/).

## [1.0.6] - 2026-10-10

- Switched the LR2021 driver dependency to `lierda-iot/esp_lora_driver`
  1.0.0 from the ESP Component Registry, replacing the 0.0.7 staging
  release. The driver recalibrates the radio after large frequency changes
  and checks the radio state more strictly during initialization.
- Updated `espressif/esp_sccb_intf` to 0.0.9 in the dependency lock file.

## [1.0.5] - 2026-09-20

- Added Freq Reset and LP Freq Reset to the gateway Settings page to bring a
  lost or re-flashed node back to the default channel.
- Frequency changes are now confirmed by the node before they take effect on
  the gateway, so the two sides can no longer end up on different channels.
- Capture requests to an unreachable node now time out after 5 seconds and
  show RX TIMEOUT instead of locking the gateway screen.
- Added per-packet software CRC on the radio link for more reliable image and
  voice transfer. Gateway and node must both run this version.
- Faster and more stable LCD refresh on the gateway.
- Corrected the bilingual feature guides to preserve the K1-K5 board
  diagram, document role switching, and describe touch-screen swiping.
- Made clean target generation preserve the AM36 flash, partition, CPU,
  PSRAM, and release optimization settings.
- Removed a machine-specific ESP-ADF path from the VS Code settings.
- Read firmware version output from the ESP-IDF application descriptor.
- Documented the open demonstration SoftAP security boundary and Nayuki QR
  Code generator attribution.

## [1.0.4] - 2026-08-21

- Prepared the firmware and documentation for the initial public repository.
- Added gateway and field-node role switching.
- Added manual, timer, PIR, and sound-triggered image capture.
- Added LR2021 image transfer, link reporting, and the HTTP gallery.
