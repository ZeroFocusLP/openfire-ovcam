#pragma once

// Multi-board Camera Pin Configuration for OpenFIRE OVcam
// Allows selecting pin configuration via board/camera macro:
//   - CAMERA_MODEL_XIAO_ESP32S3
//   - CAMERA_MODEL_FREENOVE_ESP32S3_CAM (default)
// Or custom pin defines: -D P_XCLK=... etc.

#if defined(CAMERA_MODEL_XIAO_ESP32S3) || defined(ARDUINO_SEEED_XIAO_ESP32S3) || defined(ARDUINO_XIAO_ESP32S3)

#ifndef P_PWDN
#define P_PWDN   -1
#endif
#ifndef P_RESET
#define P_RESET  -1
#endif
#ifndef P_XCLK
#define P_XCLK   10
#endif
#ifndef P_SIOD
#define P_SIOD   40
#endif
#ifndef P_SIOC
#define P_SIOC   39
#endif

#ifndef P_D7
#define P_D7     48
#endif
#ifndef P_D6
#define P_D6     11
#endif
#ifndef P_D5
#define P_D5     12
#endif
#ifndef P_D4
#define P_D4     14
#endif
#ifndef P_D3
#define P_D3     16
#endif
#ifndef P_D2
#define P_D2     18
#endif
#ifndef P_D1
#define P_D1     17
#endif
#ifndef P_D0
#define P_D0     15
#endif

#ifndef P_VSYNC
#define P_VSYNC  38
#endif
#ifndef P_HREF
#define P_HREF   47
#endif
#ifndef P_PCLK
#define P_PCLK   13
#endif

#ifndef CAMERA_DEFAULT_XCLK_FREQ
#define CAMERA_DEFAULT_XCLK_FREQ 20000000
#endif

#define CAMERA_BOARD_NAME "Seeed Studio XIAO ESP32S3 Sense"

#else  // Default: CAMERA_MODEL_FREENOVE_ESP32S3_CAM

#ifndef P_PWDN
#define P_PWDN   -1
#endif
#ifndef P_RESET
#define P_RESET  -1
#endif
#ifndef P_XCLK
#define P_XCLK   15
#endif
#ifndef P_SIOD
#define P_SIOD   4
#endif
#ifndef P_SIOC
#define P_SIOC   5
#endif

#ifndef P_D7
#define P_D7     16
#endif
#ifndef P_D6
#define P_D6     17
#endif
#ifndef P_D5
#define P_D5     18
#endif
#ifndef P_D4
#define P_D4     12
#endif
#ifndef P_D3
#define P_D3     10
#endif
#ifndef P_D2
#define P_D2     8
#endif
#ifndef P_D1
#define P_D1     9
#endif
#ifndef P_D0
#define P_D0     11
#endif

#ifndef P_VSYNC
#define P_VSYNC  6
#endif
#ifndef P_HREF
#define P_HREF   7
#endif
#ifndef P_PCLK
#define P_PCLK   13
#endif

#ifndef CAMERA_DEFAULT_XCLK_FREQ
#define CAMERA_DEFAULT_XCLK_FREQ 27000000
#endif

#define CAMERA_BOARD_NAME "Freenove ESP32-S3 WROOM CAM"

#endif
