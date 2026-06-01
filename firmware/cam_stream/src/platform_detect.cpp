/*
 * Square Platform Detection — Embedded Implementation
 *
 * Pipeline:
 *   1. Downsample grayscale (2x) to reduce compute
 *   2. Gaussian blur (3x3) for noise reduction
 *   3. Sobel edge detection (magnitude)
 *   4. Binary threshold on edge magnitude
 *   5. Connected component labeling
 *   6. Filter components by area, aspect ratio, and solidity
 *   7. Score remaining candidates and pick best square
 *
 * Memory: ~300KB in PSRAM for VGA (allocated once).
 * Speed: ~30-60ms per frame at VGA on ESP32-S3.
 *
 * NOTE: For competition use, consider replacing this with a
 * FOMO model trained on Edge Impulse for much higher reliability.
 * This code provides a working baseline for testing.
 */

#include "platform_detect.h"
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Internal downscale factor for processing speed
#define PD_DOWNSCALE 2

// Working buffers (allocated in PSRAM)
static uint8_t *pd_small = NULL;     // downsampled grayscale
static uint8_t *pd_blur = NULL;      // blurred
static int16_t *pd_gx = NULL;        // Sobel X gradient
static int16_t *pd_gy = NULL;        // Sobel Y gradient
static uint8_t *pd_edge = NULL;      // edge magnitude (thresholded)
static int16_t *pd_labels = NULL;    // connected component labels
static int pd_w = 0;
static int pd_h = 0;
static bool pd_initialized = false;

bool platform_detect_init(int frame_w, int frame_h) {
    pd_w = frame_w / PD_DOWNSCALE;
    pd_h = frame_h / PD_DOWNSCALE;
    int n = pd_w * pd_h;

    pd_small = (uint8_t *)ps_malloc(n);
    pd_blur  = (uint8_t *)ps_malloc(n);
    pd_gx    = (int16_t *)ps_malloc(n * sizeof(int16_t));
    pd_gy    = (int16_t *)ps_malloc(n * sizeof(int16_t));
    pd_edge  = (uint8_t *)ps_malloc(n);
    pd_labels = (int16_t *)ps_malloc(n * sizeof(int16_t));

    if (!pd_small || !pd_blur || !pd_gx || !pd_gy || !pd_edge || !pd_labels) {
        Serial.println("[PLATFORM] Buffer alloc failed!");
        platform_detect_free();
        return false;
    }

    pd_initialized = true;
    Serial.printf("[PLATFORM] Detector ready: %dx%d\n", pd_w, pd_h);
    return true;
}

void platform_detect_free() {
    if (pd_small)  { free(pd_small);  pd_small = NULL; }
    if (pd_blur)   { free(pd_blur);   pd_blur = NULL; }
    if (pd_gx)     { free(pd_gx);     pd_gx = NULL; }
    if (pd_gy)     { free(pd_gy);     pd_gy = NULL; }
    if (pd_edge)   { free(pd_edge);   pd_edge = NULL; }
    if (pd_labels) { free(pd_labels); pd_labels = NULL; }
    pd_initialized = false;
}

// Box downsample from full frame to working resolution
static void pd_downsample(const uint8_t *src, int sw, int sh) {
    for (int y = 0; y < pd_h; y++) {
        int sy = y * PD_DOWNSCALE;
        for (int x = 0; x < pd_w; x++) {
            int sx = x * PD_DOWNSCALE;
            int sum = 0;
            for (int dy = 0; dy < PD_DOWNSCALE; dy++) {
                const uint8_t *row = src + (sy + dy) * sw;
                for (int dx = 0; dx < PD_DOWNSCALE; dx++) {
                    sum += row[sx + dx];
                }
            }
            pd_small[y * pd_w + x] = (uint8_t)(sum / (PD_DOWNSCALE * PD_DOWNSCALE));
        }
    }
}

// Simple 3x3 Gaussian blur (1-2-1 kernel)
static void pd_gaussian_blur() {
    // Horizontal pass
    for (int y = 0; y < pd_h; y++) {
        const uint8_t *row = pd_small + y * pd_w;
        uint8_t *out = pd_blur + y * pd_w;
        out[0] = row[0];
        for (int x = 1; x < pd_w - 1; x++) {
            out[x] = (uint8_t)((row[x - 1] + 2 * row[x] + row[x + 1]) >> 2);
        }
        out[pd_w - 1] = row[pd_w - 1];
    }

    // Vertical pass (in-place on pd_blur → pd_small reused as temp)
    for (int x = 0; x < pd_w; x++) {
        pd_small[x] = pd_blur[x]; // first row
        for (int y = 1; y < pd_h - 1; y++) {
            pd_small[y * pd_w + x] = (uint8_t)(
                (pd_blur[(y - 1) * pd_w + x] +
                 2 * pd_blur[y * pd_w + x] +
                 pd_blur[(y + 1) * pd_w + x]) >> 2);
        }
        pd_small[(pd_h - 1) * pd_w + x] = pd_blur[(pd_h - 1) * pd_w + x]; // last row
    }

    // Copy back to pd_blur for Sobel input
    memcpy(pd_blur, pd_small, pd_w * pd_h);
}

// Sobel edge detection: compute gradient magnitude
static void pd_sobel_edges() {
    memset(pd_gx, 0, pd_w * pd_h * sizeof(int16_t));
    memset(pd_gy, 0, pd_w * pd_h * sizeof(int16_t));

    for (int y = 1; y < pd_h - 1; y++) {
        for (int x = 1; x < pd_w - 1; x++) {
            // Sobel 3x3 kernels
            int gx =
                -pd_blur[(y-1)*pd_w + (x-1)] + pd_blur[(y-1)*pd_w + (x+1)]
              - 2*pd_blur[y*pd_w + (x-1)]     + 2*pd_blur[y*pd_w + (x+1)]
                -pd_blur[(y+1)*pd_w + (x-1)] + pd_blur[(y+1)*pd_w + (x+1)];

            int gy =
                -pd_blur[(y-1)*pd_w + (x-1)] - 2*pd_blur[(y-1)*pd_w + x] - pd_blur[(y-1)*pd_w + (x+1)]
                +pd_blur[(y+1)*pd_w + (x-1)] + 2*pd_blur[(y+1)*pd_w + x] + pd_blur[(y+1)*pd_w + (x+1)];

            pd_gx[y * pd_w + x] = (int16_t)gx;
            pd_gy[y * pd_w + x] = (int16_t)gy;

            // Approximate magnitude: |gx| + |gy| (faster than sqrt)
            int mag = abs(gx) + abs(gy);
            pd_edge[y * pd_w + x] = (mag > PLATFORM_EDGE_THRESHOLD) ? 255 : 0;
        }
    }

    // Clear border pixels
    for (int x = 0; x < pd_w; x++) {
        pd_edge[x] = 0;
        pd_edge[(pd_h - 1) * pd_w + x] = 0;
    }
    for (int y = 0; y < pd_h; y++) {
        pd_edge[y * pd_w] = 0;
        pd_edge[y * pd_w + pd_w - 1] = 0;
    }
}

// Bounding box for a connected component
struct ComponentStats {
    int label;
    int pixel_count;     // number of edge pixels
    int min_x, max_x;
    int min_y, max_y;
    float center_x, center_y;
};

// Simple two-pass connected component labeling
// Returns number of components found (up to max_labels)
static int pd_label_components(ComponentStats *stats, int max_labels) {
    memset(pd_labels, 0, pd_w * pd_h * sizeof(int16_t));
    int next_label = 1;
    int num_components = 0;

    // First pass: assign labels with simple flood-fill
    for (int y = 1; y < pd_h - 1; y++) {
        for (int x = 1; x < pd_w - 1; x++) {
            int idx = y * pd_w + x;
            if (pd_edge[idx] == 0 || pd_labels[idx] != 0) continue;

            if (next_label >= max_labels) goto done;

            // BFS flood fill from this pixel
            int label = next_label++;
            ComponentStats *cs = &stats[num_components];
            cs->label = label;
            cs->pixel_count = 0;
            cs->min_x = x; cs->max_x = x;
            cs->min_y = y; cs->max_y = y;
            float sum_x = 0, sum_y = 0;

            // Simple stack-based flood fill (limited depth)
            static int16_t stack_x[2048];
            static int16_t stack_y[2048];
            int sp = 0;
            stack_x[sp] = x;
            stack_y[sp] = y;
            sp++;
            pd_labels[idx] = label;

            while (sp > 0) {
                sp--;
                int cx = stack_x[sp];
                int cy = stack_y[sp];

                cs->pixel_count++;
                sum_x += cx;
                sum_y += cy;
                if (cx < cs->min_x) cs->min_x = cx;
                if (cx > cs->max_x) cs->max_x = cx;
                if (cy < cs->min_y) cs->min_y = cy;
                if (cy > cs->max_y) cs->max_y = cy;

                // 4-connected neighbors
                const int dx[] = {-1, 1, 0, 0};
                const int dy[] = {0, 0, -1, 1};
                for (int d = 0; d < 4; d++) {
                    int nx = cx + dx[d];
                    int ny = cy + dy[d];
                    if (nx < 0 || nx >= pd_w || ny < 0 || ny >= pd_h) continue;
                    int ni = ny * pd_w + nx;
                    if (pd_edge[ni] == 255 && pd_labels[ni] == 0) {
                        pd_labels[ni] = label;
                        if (sp < 2047) {
                            stack_x[sp] = nx;
                            stack_y[sp] = ny;
                            sp++;
                        }
                    }
                }
            }

            cs->center_x = sum_x / cs->pixel_count;
            cs->center_y = sum_y / cs->pixel_count;
            num_components++;
        }
    }

done:
    return num_components;
}

// Score a component as a square platform candidate
static float score_square_candidate(const ComponentStats *cs, int img_w, int img_h) {
    int bbox_w = cs->max_x - cs->min_x + 1;
    int bbox_h = cs->max_y - cs->min_y + 1;

    if (bbox_w < 10 || bbox_h < 10) return 0.0f;

    int bbox_area = bbox_w * bbox_h;

    // Area filter
    if (bbox_area < PLATFORM_MIN_AREA_PX / (PD_DOWNSCALE * PD_DOWNSCALE)) return 0.0f;
    if (bbox_area > PLATFORM_MAX_AREA_PX / (PD_DOWNSCALE * PD_DOWNSCALE)) return 0.0f;

    // Aspect ratio (1.0 = perfect square)
    float aspect = (float)bbox_w / (float)bbox_h;
    if (aspect < PLATFORM_ASPECT_MIN || aspect > PLATFORM_ASPECT_MAX) return 0.0f;

    // Solidity: edge pixels / bounding box perimeter
    // A square's edge pixel count should be proportional to its perimeter
    int expected_perimeter = 2 * (bbox_w + bbox_h);
    float perimeter_ratio = (float)cs->pixel_count / (float)expected_perimeter;

    // A good square outline has perimeter_ratio near 1.0
    // Too few pixels = broken edges, too many = filled blob
    float score = 0.0f;

    // Aspect ratio score (1.0 = perfect square)
    float aspect_score = 1.0f - fabsf(aspect - 1.0f);
    if (aspect_score < 0.0f) aspect_score = 0.0f;
    score += 0.35f * aspect_score;

    // Perimeter completeness score
    float perim_score = 0.0f;
    if (perimeter_ratio >= 0.3f && perimeter_ratio <= 3.0f) {
        perim_score = 1.0f - fabsf(perimeter_ratio - 1.0f);
        if (perim_score < 0.0f) perim_score = 0.0f;
    }
    score += 0.30f * perim_score;

    // Size preference: prefer medium-sized detections (not too small/large)
    float area_frac = (float)bbox_area / (float)(img_w * img_h);
    float size_score = 0.0f;
    if (area_frac > 0.005f && area_frac < 0.5f) {
        size_score = 1.0f - fabsf(area_frac - 0.1f) / 0.1f;
        if (size_score < 0.0f) size_score = 0.0f;
        if (size_score > 1.0f) size_score = 1.0f;
    }
    score += 0.20f * size_score;

    // Center preference: prefer detections near image center
    float cx_norm = cs->center_x / img_w - 0.5f;
    float cy_norm = cs->center_y / img_h - 0.5f;
    float center_dist = sqrtf(cx_norm * cx_norm + cy_norm * cy_norm);
    float center_score = 1.0f - center_dist * 2.0f;
    if (center_score < 0.0f) center_score = 0.0f;
    score += 0.15f * center_score;

    return score;
}

PlatformResult detect_platform(const uint8_t *gray_buf, int w, int h) {
    PlatformResult result = {};
    result.detected = false;
    result.confidence = 0.0f;

    if (!pd_initialized) return result;

    // Step 1: Downsample
    pd_downsample(gray_buf, w, h);

    // Step 2: Blur
    pd_gaussian_blur();

    // Step 3: Sobel edge detection
    pd_sobel_edges();

    // Step 4: Connected component labeling
    const int MAX_COMPONENTS = 64;
    ComponentStats components[MAX_COMPONENTS];
    int num_comp = pd_label_components(components, MAX_COMPONENTS);

    // Step 5: Score each component as a square candidate
    float best_score = 0.0f;
    int best_idx = -1;

    for (int i = 0; i < num_comp; i++) {
        float s = score_square_candidate(&components[i], pd_w, pd_h);
        if (s > best_score) {
            best_score = s;
            best_idx = i;
        }
    }

    // Step 6: Build result if a good candidate was found
    if (best_idx >= 0 && best_score > 0.25f) {
        const ComponentStats *cs = &components[best_idx];

        result.detected = true;
        result.center_x = cs->center_x * PD_DOWNSCALE;
        result.center_y = cs->center_y * PD_DOWNSCALE;
        result.width_px = (float)(cs->max_x - cs->min_x + 1) * PD_DOWNSCALE;
        result.height_px = (float)(cs->max_y - cs->min_y + 1) * PD_DOWNSCALE;
        result.angle_deg = 0.0f; // TODO: compute from edge gradients
        result.confidence = best_score;

        // Rough distance estimate from known platform size
        // distance ≈ (focal_length * real_size) / pixel_size
        float focal_px = (w / 2.0f) / tanf(62.0f * M_PI / 360.0f);
        float avg_px_size = (result.width_px + result.height_px) * 0.5f;
        if (avg_px_size > 10.0f) {
            result.distance_mm = (focal_px * PLATFORM_SIZE_MM) / avg_px_size;
        }
    }

    return result;
}
