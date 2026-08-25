# Third-Party Notices

This document records third-party source material and services used to create
parts of AM36-WildCam.

## QR Code Generator

`main/qrcodegen.c` and `main/qrcodegen.h` are from Project Nayuki's QR Code
generator library. The files retain the upstream copyright and full MIT
license notice.

- Upstream project: QR Code generator library
- Copyright: Project Nayuki
- License: MIT License
- Source: https://www.nayuki.io/page/qr-code-generator-library

## SP0A39 Register Settings

The SP0A39 YUV422 register settings in `main/sp0a39_regs.h` are derived from
Espressif Systems' `esp-video-components` project and were modified for an
8-bit DVP interface and project-specific exposure targets.

- Upstream project: `espressif/esp-video-components`
- Upstream file: `esp_cam_sensor/sensors/sp0a39/private_include/sp0a39_spi_4bit_24Minput_yuv422_640x480_15fps.h`
- Copyright: 2026 Espressif Systems (Shanghai) CO LTD
- License: Apache License 2.0
- Source: https://github.com/espressif/esp-video-components

The original copyright and SPDX license identifiers are retained in the
derived source file.

## Warning Voice

The warning voice embedded in `main/warning_voice_opus.h` was generated using
Google Cloud Text-to-Speech and then encoded as Opus audio for this project.
Google Cloud Text-to-Speech was used as a generation service; no Google source
code, voice model, or software package is distributed in this repository.

- Service: Google Cloud Text-to-Speech
- Asset: `main/warning_voice_opus.h`
- Source: https://cloud.google.com/text-to-speech

Google Cloud and Google Cloud Text-to-Speech are trademarks of Google LLC.
