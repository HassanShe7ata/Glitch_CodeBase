/*
 * Square Platform Detection for ESP32-S3 Camera
 *
 * Edge-based detection of a square platform without fiducials.
 * Uses Sobel edge detection + contour analysis on grayscale frames.
 *
 * This is the deterministic fallback approach. For competition-grade
 * reliability, train a FOMO model on Edge Impulse and replace
 * detect_platform() with the inference call.
 *
 * Integration:
 *   #include "platform_detect.h"
 *   platform_detect_init(640, 480);
 *   PlatformResult r = detect_platform(gray_buf, 640, 480);
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// Detection result from the platform detector
struct PlatformResult {
    bool detected;          // true if a square platform was found
    float center_x;         // center X in image pixels
    float center_y;         // center Y in image pixels
    float width_px;         // estimated width in pixels
    float height_px;        // estimated height in pixels
    float angle_deg;        // rotation angle of the square
    float confidence;       // 0.0-1.0 detection quality
    float distance_mm;      // rough distance estimate (if platform size known)
};

// Known platform physical size in mm (update for your competition platform)
#define PLATFORM_SIZE_MM 150.0f

// Detection tuning parameters
#define PLATFORM_MIN_AREA_PX  800     // minimum contour area to consider
#define PLATFORM_MAX_AREA_PX  120000  // maximum contour area
#define PLATFORM_ASPECT_MIN   0.65f   // min aspect ratio (1.0 = perfect square)
#define PLATFORM_ASPECT_MAX   1.45f   // max aspect ratio
#define PLATFORM_SOLIDITY_MIN 0.75f   // min filled-area ratio
#define PLATFORM_EDGE_THRESHOLD 160   // Sobel edge magnitude threshold

// Initialize platform detector (allocates working buffers in PSRAM)
// Call once during setup, after camera init.
// Returns true if buffers allocated successfully.
bool platform_detect_init(int frame_w, int frame_h);

// Run platform detection on a grayscale image buffer.
// buf must be frame_w * frame_h bytes of grayscale pixel data.
// Returns detection result with best candidate platform.
PlatformResult detect_platform(const uint8_t *gray_buf, int w, int h);

// Free allocated buffers (optional, usually never called on embedded)
void platform_detect_free();
