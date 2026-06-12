/*
 * ESP32-S3 Vision - On-device QR + Pose Estimation
 * Communicates with base ESP32 via ESP-NOW.
 * Serves MJPEG stream and JSON API for dashboard.
 */

#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <esp_now.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include "quirc.h"
#include "platform_detect.h"
#include <math.h>
#include <freertos/semphr.h>

// =================== NETWORK CONFIG ===================
// Connect to the shared laptop hotspot so dashboard browser
// and ESP-NOW can both work simultaneously.
#define USE_AP_MODE false

static const char *ap_ssid = "ESP32S3-CAM";
static const char *ap_password = "12345678";
static const char *sta_ssid = "GLITCH";
static const char *sta_password = "12345678";

// =================== QR COLOR PARSING ===================
enum ArmColorCode : uint8_t {
    ARM_COLOR_UNKNOWN = 0,
    ARM_COLOR_R = 1,
    ARM_COLOR_G = 2,
    ARM_COLOR_B = 3,
};

static bool starts_with_ci(const char *text, int text_len, const char *prefix) {
    int n = (int)strlen(prefix);
    if (!text || text_len < n) return false;
    for (int i = 0; i < n; i++) {
        char a = text[i];
        char b = prefix[i];
        if (a >= 'a' && a <= 'z') a = (char)(a - 'a' + 'A');
        if (b >= 'a' && b <= 'z') b = (char)(b - 'a' + 'A');
        if (a != b) return false;
    }
    return true;
}

static ArmColorCode arm_pose_color_from_text(const char *text, int text_len) {
    if (!text || text_len <= 0) return ARM_COLOR_UNKNOWN;
    if (starts_with_ci(text, text_len, "R") || starts_with_ci(text, text_len, "RED")) return ARM_COLOR_R;
    if (starts_with_ci(text, text_len, "G") || starts_with_ci(text, text_len, "GREEN")) return ARM_COLOR_G;
    if (starts_with_ci(text, text_len, "B") || starts_with_ci(text, text_len, "BLUE")) return ARM_COLOR_B;

    for (int i = 0; i < text_len; i++) {
        char c = text[i];
        if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
        if (c == 'R') return ARM_COLOR_R;
        if (c == 'G') return ARM_COLOR_G;
        if (c == 'B') return ARM_COLOR_B;
    }
    return ARM_COLOR_UNKNOWN;
}

// =================== ESP-NOW CONFIG ===================
// Base ESP32 MAC Address — update this to match your base
// Find it in Serial Monitor when base boots (prints "BASE MAC: xx:xx:xx:xx:xx:xx")
static uint8_t baseMacAddress[] = {0x80, 0xF3, 0xDA, 0x42, 0x3E, 0x5C};

// ESP-NOW packet: Base → Camera (scan request)
struct __attribute__((packed)) ScanRequest {
    uint8_t task_id;
    uint8_t mode;        // 0=scan_qr, 1=scan_platform
    uint8_t reserved[2];
};

// ESP-NOW packet: Camera → Base (pose reply)
struct __attribute__((packed)) PoseReply {
    uint8_t task_id;
    uint8_t pose_valid;
    uint8_t color;       // ArmColorCode enum
    uint8_t estimated;
    float tx_mm;
    float ty_mm;
    float tz_mm;
    float yaw_deg;
    float confidence;
};

static bool g_espnow_ready = false;
static volatile bool g_scan_requested = false;
static volatile uint8_t g_scan_task_id = 0;
static volatile uint8_t g_scan_mode = 0;
static portMUX_TYPE g_scan_mux = portMUX_INITIALIZER_UNLOCKED;

// Latest platform detection result (for /platform HTTP endpoint)
static PlatformResult g_last_platform = {};
static SemaphoreHandle_t g_platform_mutex = NULL;
static uint32_t g_platform_detect_ms = 0;

#define STREAM_FRAME_SIZE FRAMESIZE_VGA
#define STREAM_JPEG_QUALITY 12
#define STREAM_FPS_TARGET 4
#define LED_FLASH_PIN -1

// QR preprocessing and detection quality controls.
// Downscale=2 is currently the best latency/robustness compromise for GC2145.
#define QR_DOWNSCALE 1
#define QR_USE_ADAPTIVE_BINARIZE 1
#define QR_MAIN_USE_ADAPTIVE 0

#define QR_SIZE_MM 50.0f
#define CAMERA_FOV_DEG 62.0f
#define MAX_QR_DETECTIONS 8
#define MAX_QR_TEXT_LEN 256

#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   15
#define SIOD_GPIO_NUM   4
#define SIOC_GPIO_NUM   5
#define Y9_GPIO_NUM     16
#define Y8_GPIO_NUM     17
#define Y7_GPIO_NUM     18
#define Y6_GPIO_NUM     12
#define Y5_GPIO_NUM     10
#define Y4_GPIO_NUM     8
#define Y3_GPIO_NUM     9
#define Y2_GPIO_NUM     11
#define VSYNC_GPIO_NUM  6
#define HREF_GPIO_NUM   7
#define PCLK_GPIO_NUM   13

struct CameraIntrinsics {
    // Intrinsic matrix terms:
    // [fx  0  cx]
    // [0  fy  cy]
    // [0   0   1]
    float fx, fy, cx, cy;
};

struct QRDetection {
    // Payload fields
    char text[MAX_QR_TEXT_LEN];
    int text_len;
    bool decoded;

    // Tracker flags
    bool estimated;
    float confidence;
    uint32_t age_ms;

    // Image-space corner points in pixels, order from quirc extract
    float corners[4][2];

    // Pose in camera frame (mm + deg)
    bool pose_valid;
    float tx, ty, tz;
    float roll, pitch, yaw;
};

struct QRTrackState {
    // Single-target temporal tracker state used to bridge short detection dropouts.
    bool active;
    QRDetection det;
    float confidence;
    uint32_t last_obs_ms;
    uint32_t last_update_ms;
};

static httpd_handle_t g_httpd = NULL;
static httpd_handle_t g_stream_httpd = NULL;
static bool g_camera_ok = false;

static CameraIntrinsics g_K;
static struct quirc *g_qr = NULL;
static uint8_t *g_gray_buf = NULL;
static uint8_t *g_proc_buf = NULL;
static uint8_t *g_proc_gray_buf = NULL;
static uint8_t *g_proc_raw_buf = NULL;
static int g_frame_w = 0;
static int g_frame_h = 0;
static int g_proc_w = 0;
static int g_proc_h = 0;
static SemaphoreHandle_t g_cam_mutex = NULL;

static SemaphoreHandle_t g_data_mutex = NULL;
static QRDetection g_detections[MAX_QR_DETECTIONS];

// Reusable working buffers for multi-pass quirc scans.
// Keeping these global avoids large stack allocations inside qr_task.
static QRDetection g_scan_dets[MAX_QR_DETECTIONS];
static QRDetection g_alt_scan_dets[MAX_QR_DETECTIONS];
static QRDetection g_inv_scan_dets[MAX_QR_DETECTIONS];
static int g_num_detections = 0;
static uint32_t g_frame_id = 0;
static uint32_t g_processing_ms = 0;
static float g_qr_fps = 0.0f;
static int g_qr_raw_count = 0;
static int g_qr_decoded_count = 0;
static int g_last_decode_err = QUIRC_SUCCESS;
static int g_last_decode_err_flip = QUIRC_SUCCESS;
static QRTrackState g_track = {0};

// Tracker tuning knobs:
// - HOLD/TAU/MIN_CONF define how long estimates remain usable after observation loss.
// - OBS_ACCEPT_PROB controls how strict we are before accepting new observations.
#define TRACK_MAX_HOLD_MS 3500U
#define TRACK_DECAY_TAU_MS 2600.0f
#define TRACK_MIN_CONF 0.10f
#define OBS_ACCEPT_PROB 0.15f

// Robustness-first but decode-enabled: user prefers payload visibility over speed.
#define ENABLE_PAYLOAD_DECODE 1

static TaskHandle_t g_qr_task = NULL;

static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char *STREAM_BOUNDARY = "\r\n--frame\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// Convert RGB565 camera frame into grayscale luminance used by QR pipeline.
static void rgb565_to_gray(const uint8_t *src, uint8_t *dst, int w, int h) {
    int n = w * h;
    for (int i = 0; i < n; i++) {
        uint16_t px = ((uint16_t)src[i * 2 + 1] << 8) | src[i * 2];
        uint8_t r = (px >> 11) & 0x1F;
        uint8_t g = (px >> 5) & 0x3F;
        uint8_t b = px & 0x1F;
        dst[i] = (uint8_t)((r * 630 + g * 609 + b * 240) >> 8);
    }
}

// Box downsampling to reduce compute cost while preserving QR structure.
static void downsample_gray(const uint8_t *src, int sw, int sh, uint8_t *dst) {
    if (QR_DOWNSCALE <= 1) {
        memcpy(dst, src, sw * sh);
        return;
    }

    int dw = sw / QR_DOWNSCALE;
    int dh = sh / QR_DOWNSCALE;
    for (int y = 0; y < dh; y++) {
        int sy0 = y * QR_DOWNSCALE;
        uint8_t *drow = dst + y * dw;
        for (int x = 0; x < dw; x++) {
            int sx0 = x * QR_DOWNSCALE;
            int sum = 0;
            for (int yy = 0; yy < QR_DOWNSCALE; yy++) {
                const uint8_t *srow = src + (sy0 + yy) * sw;
                for (int xx = 0; xx < QR_DOWNSCALE; xx++) {
                    sum += srow[sx0 + xx];
                }
            }
            drow[x] = (uint8_t)(sum / (QR_DOWNSCALE * QR_DOWNSCALE));
        }
    }
}

// Contrast stretch with percentile clipping to improve threshold separation.
static void contrast_stretch(uint8_t *buf, int n) {
    uint32_t hist[256] = {0};
    for (int i = 0; i < n; i++) hist[buf[i]]++;

    int lo_target = n / 50;
    int hi_target = n / 50;
    int lo = 0, lo_sum = 0;
    while (lo < 255 && lo_sum + (int)hist[lo] < lo_target) lo_sum += (int)hist[lo++];
    int hi = 255, hi_sum = 0;
    while (hi > lo + 1 && hi_sum + (int)hist[hi] < hi_target) hi_sum += (int)hist[hi--];

    if (hi <= lo) return;

    float scale = 255.0f / (float)(hi - lo);
    for (int i = 0; i < n; i++) {
        int v = (int)(((int)buf[i] - lo) * scale);
        buf[i] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
    }
}

// Lightweight local thresholding (blockwise mean - offset) for uneven illumination.
static void adaptive_binarize(uint8_t *buf, int w, int h) {
    const int BLK = 16;
    const int bw = (w + BLK - 1) / BLK;
    const int bh = (h + BLK - 1) / BLK;
    uint8_t means[64 * 64];

    if (bw * bh > (int)(sizeof(means) / sizeof(means[0]))) return;

    for (int by = 0; by < bh; by++) {
        for (int bx = 0; bx < bw; bx++) {
            int x0 = bx * BLK;
            int y0 = by * BLK;
            int x1 = x0 + BLK < w ? x0 + BLK : w;
            int y1 = y0 + BLK < h ? y0 + BLK : h;
            int sum = 0;
            int cnt = 0;
            for (int y = y0; y < y1; y++) {
                uint8_t *row = buf + y * w;
                for (int x = x0; x < x1; x++) {
                    sum += row[x];
                    cnt++;
                }
            }
            means[by * bw + bx] = (uint8_t)(sum / (cnt > 0 ? cnt : 1));
        }
    }

    int global_sum = 0;
    for (int i = 0; i < bw * bh; i++) global_sum += means[i];
    int global_mean = global_sum / (bw * bh > 0 ? bw * bh : 1);
    // Good-light profile: keep the adaptive threshold soft so near-frontal QR
    // modules are not over-binarized.
    int base_bias = 4;
    if (global_mean < 90) base_bias = 2;
    else if (global_mean < 120) base_bias = 3;
    else if (global_mean > 170) base_bias = 5;

    for (int y = 0; y < h; y++) {
        int by = y / BLK;
        uint8_t *row = buf + y * w;
        for (int x = 0; x < w; x++) {
            int bx = x / BLK;
            int thr = (int)means[by * bw + bx] - base_bias;
            if (thr < 16) thr = 16;
            row[x] = (row[x] < thr) ? 0 : 255;
        }
    }
}

// Adaptive threshold with caller-provided bias for decode recovery sweeps.
static void adaptive_binarize_with_bias(uint8_t *buf, int w, int h, int bias) {
    const int BLK = 16;
    const int bw = (w + BLK - 1) / BLK;
    const int bh = (h + BLK - 1) / BLK;
    uint8_t means[64 * 64];

    if (bw * bh > (int)(sizeof(means) / sizeof(means[0]))) return;

    for (int by = 0; by < bh; by++) {
        for (int bx = 0; bx < bw; bx++) {
            int x0 = bx * BLK;
            int y0 = by * BLK;
            int x1 = x0 + BLK < w ? x0 + BLK : w;
            int y1 = y0 + BLK < h ? y0 + BLK : h;
            int sum = 0;
            int cnt = 0;
            for (int y = y0; y < y1; y++) {
                uint8_t *row = buf + y * w;
                for (int x = x0; x < x1; x++) {
                    sum += row[x];
                    cnt++;
                }
            }
            means[by * bw + bx] = (uint8_t)(sum / (cnt > 0 ? cnt : 1));
        }
    }

    for (int y = 0; y < h; y++) {
        int by = y / BLK;
        uint8_t *row = buf + y * w;
        for (int x = 0; x < w; x++) {
            int bx = x / BLK;
            int thr = (int)means[by * bw + bx] - bias;
            if (thr < 16) thr = 16;
            row[x] = (row[x] < thr) ? 0 : 255;
        }
    }
}

// Normalize angles to [-180, 180] to avoid discontinuities in temporal blending.
static float wrap_angle_deg(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static float blend_angle_deg(float prev, float obs, float alpha) {
    float d = wrap_angle_deg(obs - prev);
    return wrap_angle_deg(prev + alpha * d);
}

// Polygon area proxy used as one confidence feature (larger projected QR is usually more stable).
static float quad_area(const float corners[4][2]) {
    float sum = 0.0f;
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) & 3;
        sum += corners[i][0] * corners[j][1] - corners[j][0] * corners[i][1];
    }
    return fabsf(sum) * 0.5f;
}

static void quad_center(const float corners[4][2], float &cx, float &cy) {
    cx = 0.0f;
    cy = 0.0f;
    for (int i = 0; i < 4; i++) {
        cx += corners[i][0];
        cy += corners[i][1];
    }
    cx *= 0.25f;
    cy *= 0.25f;
}

static bool decode_quirc_payload(struct quirc_code &code, QRDetection &det) {
    struct quirc_data data;
    quirc_decode_error_t derr = QUIRC_ERROR_DATA_ECC;
    quirc_decode_error_t derr_flip = QUIRC_SUCCESS;
    if (ENABLE_PAYLOAD_DECODE) {
        derr = quirc_decode(&code, &data);
        if (derr != QUIRC_SUCCESS) {
            struct quirc_code flipped = code;
            quirc_flip(&flipped);
            derr_flip = quirc_decode(&flipped, &data);
            derr = derr_flip;
        }
    }
    g_last_decode_err = (int)derr;
    g_last_decode_err_flip = (int)derr_flip;

    if (derr == QUIRC_SUCCESS) {
        int len = data.payload_len;
        if (len >= MAX_QR_TEXT_LEN) len = MAX_QR_TEXT_LEN - 1;
        memcpy(det.text, data.payload, len);
        det.text[len] = '\0';
        det.text_len = len;
        det.decoded = true;
        return true;
    }

    const char *label = ENABLE_PAYLOAD_DECODE ? "QR_UNDECODED" : "QR_RAW";
    int len = snprintf(det.text, MAX_QR_TEXT_LEN, "%s", label);
    if (len < 0) len = 0;
    if (len >= MAX_QR_TEXT_LEN) len = MAX_QR_TEXT_LEN - 1;
    det.text[len] = '\0';
    det.text_len = len;
    det.decoded = false;
    return false;
}

// Compute observation probability from pose validity, decode status, projected size,
// and temporal continuity relative to the tracked state.
static float detection_probability(const QRDetection &obs, const QRTrackState &track) {
    float p = 0.10f;
    if (obs.pose_valid) p += 0.35f;
    if (obs.decoded) p += 0.20f;

    float area = quad_area(obs.corners);
    float area_score = area / 22000.0f;
    if (area_score < 0.0f) area_score = 0.0f;
    if (area_score > 1.0f) area_score = 1.0f;
    p += 0.25f * area_score;

    if (track.active) {
        float cxo, cyo, cxt, cyt;
        quad_center(obs.corners, cxo, cyo);
        quad_center(track.det.corners, cxt, cyt);
        float dx = cxo - cxt;
        float dy = cyo - cyt;
        float dist = sqrtf(dx * dx + dy * dy);
        float continuity = expf(-dist / 130.0f);
        p += 0.30f * continuity;

        if (obs.pose_valid && track.det.pose_valid) {
            float dz = fabsf(obs.tz - track.det.tz);
            p += 0.15f * expf(-dz / 120.0f);
        }
    }

    if (p < 0.0f) p = 0.0f;
    if (p > 1.0f) p = 1.0f;
    return p;
}

// Tracker update when a new observation is accepted.
// Corners and pose are confidence-weight blended for stability.
static void track_from_observation(const QRDetection &obs, float p, uint32_t now_ms) {
    if (!g_track.active) {
        g_track.active = true;
        g_track.det = obs;
        g_track.confidence = p;
        g_track.det.estimated = false;
        g_track.det.confidence = g_track.confidence;
        g_track.det.age_ms = 0;
        g_track.last_obs_ms = now_ms;
        g_track.last_update_ms = now_ms;
        return;
    }

    float alpha_corner = 0.20f + 0.70f * p;
    float alpha_pose = 0.15f + 0.75f * p;
    if (alpha_corner > 1.0f) alpha_corner = 1.0f;
    if (alpha_pose > 1.0f) alpha_pose = 1.0f;

    for (int i = 0; i < 4; i++) {
        g_track.det.corners[i][0] += alpha_corner * (obs.corners[i][0] - g_track.det.corners[i][0]);
        g_track.det.corners[i][1] += alpha_corner * (obs.corners[i][1] - g_track.det.corners[i][1]);
    }

    if (obs.pose_valid) {
        if (!g_track.det.pose_valid) {
            g_track.det.tx = obs.tx;
            g_track.det.ty = obs.ty;
            g_track.det.tz = obs.tz;
            g_track.det.roll = obs.roll;
            g_track.det.pitch = obs.pitch;
            g_track.det.yaw = obs.yaw;
            g_track.det.pose_valid = true;
        } else {
            g_track.det.tx += alpha_pose * (obs.tx - g_track.det.tx);
            g_track.det.ty += alpha_pose * (obs.ty - g_track.det.ty);
            g_track.det.tz += alpha_pose * (obs.tz - g_track.det.tz);
            g_track.det.roll = blend_angle_deg(g_track.det.roll, obs.roll, alpha_pose);
            g_track.det.pitch = blend_angle_deg(g_track.det.pitch, obs.pitch, alpha_pose);
            g_track.det.yaw = blend_angle_deg(g_track.det.yaw, obs.yaw, alpha_pose);
        }
    }

    if (obs.decoded || !g_track.det.decoded) {
        memcpy(g_track.det.text, obs.text, MAX_QR_TEXT_LEN);
        g_track.det.text_len = obs.text_len;
        g_track.det.decoded = obs.decoded;
    }

    g_track.confidence = 0.65f * g_track.confidence + 0.35f * p;
    if (g_track.confidence < p) g_track.confidence = p;
    if (g_track.confidence > 1.0f) g_track.confidence = 1.0f;

    g_track.det.estimated = false;
    g_track.det.confidence = g_track.confidence;
    g_track.det.age_ms = 0;
    g_track.last_obs_ms = now_ms;
    g_track.last_update_ms = now_ms;
}

// Tracker-only prediction mode used when no valid observation is available.
// Confidence decays exponentially with elapsed time since last observation.
static bool track_predict_only(uint32_t now_ms) {
    if (!g_track.active) return false;

    uint32_t age = now_ms - g_track.last_obs_ms;
    float conf = expf(-((float)age) / TRACK_DECAY_TAU_MS);
    g_track.confidence = conf;
    g_track.det.estimated = true;
    g_track.det.confidence = conf;
    g_track.det.age_ms = age;
    g_track.last_update_ms = now_ms;

    if (age > TRACK_MAX_HOLD_MS || conf < TRACK_MIN_CONF) {
        g_track.active = false;
        return false;
    }
    return true;
}

// Solve 8x8 linear system (augmented matrix) for homography estimation.
static bool gauss_solve_8(float A[8][9]) {
    for (int col = 0; col < 8; col++) {
        int pivot = col;
        float best = fabsf(A[col][col]);
        for (int row = col + 1; row < 8; row++) {
            float v = fabsf(A[row][col]);
            if (v > best) {
                best = v;
                pivot = row;
            }
        }
        if (best < 1e-10f) return false;
        if (pivot != col) {
            for (int j = 0; j < 9; j++) {
                float t = A[col][j];
                A[col][j] = A[pivot][j];
                A[pivot][j] = t;
            }
        }
        for (int row = col + 1; row < 8; row++) {
            float f = A[row][col] / A[col][col];
            for (int j = col; j < 9; j++) A[row][j] -= f * A[col][j];
        }
    }
    for (int row = 7; row >= 0; row--) {
        for (int j = row + 1; j < 8; j++) A[row][8] -= A[row][j] * A[j][8];
        A[row][8] /= A[row][row];
    }
    return true;
}

// Direct linear transform setup for planar homography from 4 corner pairs.
static bool compute_homography(const float world[4][2], const float img[4][2], float H[3][3]) {
    float A[8][9] = {0};
    for (int i = 0; i < 4; i++) {
        float X = world[i][0], Y = world[i][1];
        float u = img[i][0], v = img[i][1];

        int r = i * 2;
        A[r][0] = X; A[r][1] = Y; A[r][2] = 1;
        A[r][6] = -u * X; A[r][7] = -u * Y; A[r][8] = u;

        A[r + 1][3] = X; A[r + 1][4] = Y; A[r + 1][5] = 1;
        A[r + 1][6] = -v * X; A[r + 1][7] = -v * Y; A[r + 1][8] = v;
    }
    if (!gauss_solve_8(A)) return false;

    H[0][0] = A[0][8]; H[0][1] = A[1][8]; H[0][2] = A[2][8];
    H[1][0] = A[3][8]; H[1][1] = A[4][8]; H[1][2] = A[5][8];
    H[2][0] = A[6][8]; H[2][1] = A[7][8]; H[2][2] = 1.0f;
    return true;
}

// Convert normalized homography into rotation and translation up to scale.
static bool decompose_homography(const float H[3][3], float R[3][3], float t[3]) {
    float h1[3] = {H[0][0], H[1][0], H[2][0]};
    float h2[3] = {H[0][1], H[1][1], H[2][1]};
    float h3[3] = {H[0][2], H[1][2], H[2][2]};

    float n1 = sqrtf(h1[0] * h1[0] + h1[1] * h1[1] + h1[2] * h1[2]);
    float n2 = sqrtf(h2[0] * h2[0] + h2[1] * h2[1] + h2[2] * h2[2]);
    if (n1 < 1e-6f || n2 < 1e-6f) return false;

    float s = 0.5f * (n1 + n2);
    float r1[3] = {h1[0] / n1, h1[1] / n1, h1[2] / n1};

    // Gram-Schmidt orthonormalization keeps R stable when corner ordering/noise
    // perturbs homography columns.
    float dot12 = r1[0] * h2[0] + r1[1] * h2[1] + r1[2] * h2[2];
    float u2[3] = {
        h2[0] - dot12 * r1[0],
        h2[1] - dot12 * r1[1],
        h2[2] - dot12 * r1[2]
    };
    float u2n = sqrtf(u2[0] * u2[0] + u2[1] * u2[1] + u2[2] * u2[2]);
    if (u2n < 1e-6f) return false;
    float r2[3] = {u2[0] / u2n, u2[1] / u2n, u2[2] / u2n};

    float r3[3] = {
        r1[1] * r2[2] - r1[2] * r2[1],
        r1[2] * r2[0] - r1[0] * r2[2],
        r1[0] * r2[1] - r1[1] * r2[0]
    };
    float r3n = sqrtf(r3[0] * r3[0] + r3[1] * r3[1] + r3[2] * r3[2]);
    if (r3n < 1e-6f) return false;
    r3[0] /= r3n;
    r3[1] /= r3n;
    r3[2] /= r3n;

    // Recompute r2 from cross(r3, r1) to enforce orthogonality and right-handedness.
    r2[0] = r3[1] * r1[2] - r3[2] * r1[1];
    r2[1] = r3[2] * r1[0] - r3[0] * r1[2];
    r2[2] = r3[0] * r1[1] - r3[1] * r1[0];

    for (int i = 0; i < 3; i++) {
        R[i][0] = r1[i];
        R[i][1] = r2[i];
        R[i][2] = r3[i];
        t[i] = h3[i] / s;
    }

    if (t[2] < 0) {
        for (int i = 0; i < 3; i++) {
            R[i][0] = -R[i][0];
            R[i][1] = -R[i][1];
            R[i][2] = -R[i][2];
            t[i] = -t[i];
        }
    }
    return true;
}

// Canonical corner ordering prevents pose flips when quirc corner start index varies.
// Output order is top-left, top-right, bottom-right, bottom-left in image coordinates.
static void reorder_corners_tl_tr_br_bl(const float in[4][2], float out[4][2]) {
    int idx[4] = {0, 1, 2, 3};

    // Sort by y ascending to split top and bottom pairs.
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            if (in[idx[j]][1] < in[idx[i]][1]) {
                int t = idx[i];
                idx[i] = idx[j];
                idx[j] = t;
            }
        }
    }

    int top0 = idx[0], top1 = idx[1];
    int bot0 = idx[2], bot1 = idx[3];

    int tl = (in[top0][0] <= in[top1][0]) ? top0 : top1;
    int tr = (in[top0][0] <= in[top1][0]) ? top1 : top0;
    int bl = (in[bot0][0] <= in[bot1][0]) ? bot0 : bot1;
    int br = (in[bot0][0] <= in[bot1][0]) ? bot1 : bot0;

    out[0][0] = in[tl][0]; out[0][1] = in[tl][1];
    out[1][0] = in[tr][0]; out[1][1] = in[tr][1];
    out[2][0] = in[br][0]; out[2][1] = in[br][1];
    out[3][0] = in[bl][0]; out[3][1] = in[bl][1];
}

// Rotation matrix to Euler angles (deg), with singularity handling.
static void rotation_to_euler(const float R[3][3], float &roll, float &pitch, float &yaw) {
    float sy = sqrtf(R[0][0] * R[0][0] + R[1][0] * R[1][0]);
    if (sy > 1e-6f) {
        roll = atan2f(R[2][1], R[2][2]) * 180.0f / M_PI;
        pitch = atan2f(-R[2][0], sy) * 180.0f / M_PI;
        yaw = atan2f(R[1][0], R[0][0]) * 180.0f / M_PI;
    } else {
        roll = atan2f(-R[1][2], R[1][1]) * 180.0f / M_PI;
        pitch = atan2f(-R[2][0], sy) * 180.0f / M_PI;
        yaw = 0.0f;
    }

    // Keep roll on a compact branch to avoid +-180deg display jumps while
    // preserving original yaw/pitch mapping.
    roll = wrap_angle_deg(roll);
    if (roll > 90.0f) roll -= 180.0f;
    if (roll < -90.0f) roll += 180.0f;
    pitch = wrap_angle_deg(pitch);
    yaw = wrap_angle_deg(yaw);
}

// Estimate QR pose from 2D corners using known physical QR size and camera intrinsics.
static bool compute_qr_pose(const float corners[4][2], const CameraIntrinsics &K, QRDetection &det) {
    float half = QR_SIZE_MM * 0.5f;
    // QR object points (meters in mm units here), centered on QR plane.
    // Order matches canonical image order: TL, TR, BR, BL.
    float world[4][2] = {
        {-half,  half},
        { half,  half},
        { half, -half},
        {-half, -half},
    };

    float ordered[4][2];
    reorder_corners_tl_tr_br_bl(corners, ordered);

    float norm_img[4][2];
    for (int i = 0; i < 4; i++) {
        norm_img[i][0] = (ordered[i][0] - K.cx) / K.fx;
        norm_img[i][1] = (ordered[i][1] - K.cy) / K.fy;
    }

    float H[3][3], R[3][3], t[3];
    if (!compute_homography(world, norm_img, H)) return false;
    if (!decompose_homography(H, R, t)) return false;

    det.tx = t[0];
    det.ty = t[1];
    det.tz = t[2];
    rotation_to_euler(R, det.roll, det.pitch, det.yaw);
    det.pose_valid = true;
    return true;
}

// One quirc scan pass over provided preprocessed image buffer.
// Fills detection array with corners, payload decode result, and pose estimate.
static void run_quirc_scan(uint8_t *src, QRDetection *dets, int &valid, int &count, int &decoded_ok) {
    uint8_t *img = quirc_begin(g_qr, NULL, NULL);
    memcpy(img, src, g_proc_w * g_proc_h);
    quirc_end(g_qr);

    valid = 0;
    count = quirc_count(g_qr);
    decoded_ok = 0;
    for (int i = 0; i < count && valid < MAX_QR_DETECTIONS; i++) {
        struct quirc_code code;
        quirc_extract(g_qr, i, &code);

        QRDetection &d = dets[valid];
        memset(&d, 0, sizeof(d));

        if (decode_quirc_payload(code, d)) {
            decoded_ok++;
        }

        for (int c = 0; c < 4; c++) {
            d.corners[c][0] = (float)(code.corners[c].x * QR_DOWNSCALE);
            d.corners[c][1] = (float)(code.corners[c].y * QR_DOWNSCALE);
        }
        float ordered[4][2];
        reorder_corners_tl_tr_br_bl(d.corners, ordered);
        for (int c = 0; c < 4; c++) {
            d.corners[c][0] = ordered[c][0];
            d.corners[c][1] = ordered[c][1];
        }
        compute_qr_pose(d.corners, g_K, d);
        valid++;
    }
}

// Initialize QR engine and allocate all processing buffers.
static void init_qr(int frame_w, int frame_h) {
    g_frame_w = frame_w;
    g_frame_h = frame_h;
    g_proc_w = frame_w / QR_DOWNSCALE;
    g_proc_h = frame_h / QR_DOWNSCALE;

    g_qr = quirc_new();
    if (!g_qr) {
        Serial.println("quirc_new failed");
        return;
    }
    if (quirc_resize(g_qr, g_proc_w, g_proc_h) < 0) {
        Serial.println("quirc_resize failed");
        quirc_destroy(g_qr);
        g_qr = NULL;
        return;
    }
    g_gray_buf = (uint8_t *)ps_malloc(g_frame_w * g_frame_h);
    g_proc_buf = (uint8_t *)ps_malloc(g_proc_w * g_proc_h);
    g_proc_gray_buf = (uint8_t *)ps_malloc(g_proc_w * g_proc_h);
    g_proc_raw_buf = (uint8_t *)ps_malloc(g_proc_w * g_proc_h);
    if (!g_gray_buf || !g_proc_buf || !g_proc_gray_buf || !g_proc_raw_buf) {
        Serial.println("gray buffer alloc failed");
        quirc_destroy(g_qr);
        g_qr = NULL;
        if (g_gray_buf) {
            free(g_gray_buf);
            g_gray_buf = NULL;
        }
        if (g_proc_buf) {
            free(g_proc_buf);
            g_proc_buf = NULL;
        }
        if (g_proc_gray_buf) {
            free(g_proc_gray_buf);
            g_proc_gray_buf = NULL;
        }
        if (g_proc_raw_buf) {
            free(g_proc_raw_buf);
            g_proc_raw_buf = NULL;
        }
        return;
    }

    Serial.printf("QR engine ready: frame=%dx%d proc=%dx%d\n",
                  g_frame_w, g_frame_h, g_proc_w, g_proc_h);
}

// Core QR processing routine executed by dedicated qr_task.
// Pipeline:
// 1) capture frame under camera mutex
// 2) grayscale + preprocessing
// 3) multi-pass quirc scan (adaptive, gray, inverted)
// 4) select/buffer best detections
// 5) tracker update or predict-only fallback
// 6) publish thread-safe result snapshot
static void process_qr_frame() {
    if (!g_cam_mutex) return;
    xSemaphoreTake(g_cam_mutex, portMAX_DELAY);
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        xSemaphoreGive(g_cam_mutex);
        return;
    }

    uint32_t t0 = millis();
    int w = fb->width, h = fb->height;

    if (fb->format == PIXFORMAT_RGB565) {
        rgb565_to_gray(fb->buf, g_gray_buf, w, h);
    } else if (fb->format == PIXFORMAT_GRAYSCALE) {
        memcpy(g_gray_buf, fb->buf, w * h);
    } else {
        esp_camera_fb_return(fb);
        xSemaphoreGive(g_cam_mutex);
        return;
    }
    esp_camera_fb_return(fb);
    xSemaphoreGive(g_cam_mutex);

    if (!g_proc_buf || !g_proc_gray_buf) return;

    downsample_gray(g_gray_buf, w, h, g_proc_raw_buf);
    memcpy(g_proc_gray_buf, g_proc_raw_buf, g_proc_w * g_proc_h);
    contrast_stretch(g_proc_gray_buf, g_proc_w * g_proc_h);

    // Quirc-native main pass: raw grayscale first. Heavier preprocessing is
    // retained in fallback passes to recover difficult frames.
    memcpy(g_proc_buf, g_proc_raw_buf, g_proc_w * g_proc_h);
    if (QR_USE_ADAPTIVE_BINARIZE && QR_MAIN_USE_ADAPTIVE) {
        adaptive_binarize(g_proc_buf, g_proc_w, g_proc_h);
    }

    QRDetection *dets = g_scan_dets;
    int valid = 0;
    uint32_t now_ms = millis();
    int count = 0;
    int decoded_ok = 0;
    run_quirc_scan(g_proc_buf, dets, valid, count, decoded_ok);

    // Pass 2+: retry passes are expensive; throttle only lightly so decode
    // recovery still runs often enough to matter during live alignment.
    static uint32_t s_decode_fail_streak = 0;
    if (decoded_ok == 0) s_decode_fail_streak++;
    else s_decode_fail_streak = 0;
    bool run_retry_passes = (count == 0);
    if (!run_retry_passes && decoded_ok == 0) {
        // Retry less often to control frame time; keep periodic recovery active.
        run_retry_passes = ((s_decode_fail_streak % 4) == 0);
    }

    // Pass 2: contrast-stretched grayscale can recover some failed adaptive cases.
    if (run_retry_passes) {
        int alt_valid = 0;
        int alt_count = 0;
        int alt_decoded_ok = 0;
        run_quirc_scan(g_proc_gray_buf, g_alt_scan_dets, alt_valid, alt_count, alt_decoded_ok);

        if (alt_decoded_ok > decoded_ok || (decoded_ok == 0 && alt_count > count)) {
            memcpy(dets, g_alt_scan_dets, sizeof(g_alt_scan_dets));
            valid = alt_valid;
            count = alt_count;
            decoded_ok = alt_decoded_ok;
        }

        // Pass 3: inverted grayscale can recover polarity-sensitive decode failures.
        // Run it sparsely because it is expensive and often low-yield.
        if (decoded_ok == 0 && (s_decode_fail_streak % 8) == 0) {
            // Pass 2b: raw downsample without contrast stretch can preserve module
            // transitions that get distorted by global stretch.
            int raw_valid = 0;
            int raw_count = 0;
            int raw_decoded_ok = 0;
            run_quirc_scan(g_proc_raw_buf, g_alt_scan_dets, raw_valid, raw_count, raw_decoded_ok);
            if (raw_decoded_ok > decoded_ok || (decoded_ok == 0 && raw_count > count)) {
                memcpy(dets, g_alt_scan_dets, sizeof(g_alt_scan_dets));
                valid = raw_valid;
                count = raw_count;
                decoded_ok = raw_decoded_ok;
            }

            int n = g_proc_w * g_proc_h;
            for (int i = 0; i < n; i++) {
                g_proc_buf[i] = 255 - g_proc_gray_buf[i];
            }
            int inv_valid = 0;
            int inv_count = 0;
            int inv_decoded_ok = 0;
            run_quirc_scan(g_proc_buf, g_inv_scan_dets, inv_valid, inv_count, inv_decoded_ok);

            if (inv_decoded_ok > decoded_ok || (decoded_ok == 0 && inv_count > count)) {
                memcpy(dets, g_inv_scan_dets, sizeof(g_inv_scan_dets));
                valid = inv_valid;
                count = inv_count;
                decoded_ok = inv_decoded_ok;
            }
        }

        // Pass 4+: decode recovery sweep with fixed adaptive biases.
        // This runs only when payload decode is still failing.
        if (decoded_ok == 0 && QR_USE_ADAPTIVE_BINARIZE) {
            const int kBiases[] = {4};
            for (int bi = 0; bi < (int)(sizeof(kBiases) / sizeof(kBiases[0])); bi++) {
                memcpy(g_proc_buf, g_proc_gray_buf, g_proc_w * g_proc_h);
                adaptive_binarize_with_bias(g_proc_buf, g_proc_w, g_proc_h, kBiases[bi]);

                int fb_valid = 0;
                int fb_count = 0;
                int fb_decoded_ok = 0;
                run_quirc_scan(g_proc_buf, g_alt_scan_dets, fb_valid, fb_count, fb_decoded_ok);

                if (fb_decoded_ok > decoded_ok || (decoded_ok == 0 && fb_count > count)) {
                    memcpy(dets, g_alt_scan_dets, sizeof(g_alt_scan_dets));
                    valid = fb_valid;
                    count = fb_count;
                    decoded_ok = fb_decoded_ok;
                }
                if (decoded_ok > 0) break;
            }
        }
    }

    int best_idx = -1;
    float best_p = -1.0f;
    int best_decoded_idx = -1;
    float best_decoded_p = -1.0f;
    for (int i = 0; i < valid; i++) {
        float p = detection_probability(dets[i], g_track);
        if (p > best_p) {
            best_p = p;
            best_idx = i;
        }
        if (dets[i].decoded && p > best_decoded_p) {
            best_decoded_p = p;
            best_decoded_idx = i;
        }
    }

    // Prefer decoded observations when available so payload stability does not get
    // starved by slightly higher-probability undecoded candidates.
    if (best_decoded_idx >= 0) {
        best_idx = best_decoded_idx;
        best_p = best_decoded_p;
    }

    bool observed = false;
    // Observation gating avoids feeding weak/noisy detections into the tracker.
    float accept_prob = OBS_ACCEPT_PROB;
    if (best_idx >= 0 && dets[best_idx].decoded) {
        accept_prob *= 0.45f;
    }

    if (best_idx >= 0 && best_p >= accept_prob) {
        track_from_observation(dets[best_idx], best_p, now_ms);
        observed = true;
    } else {
        track_predict_only(now_ms);
    }

    // Send pose to base via ESP-NOW when a scan was requested or periodically.
    const QRDetection *pose_src = NULL;
    if (g_track.active && g_track.det.pose_valid) {
        pose_src = &g_track.det;
    } else if (best_idx >= 0 && dets[best_idx].pose_valid) {
        pose_src = &dets[best_idx];
    }

    // Send ESP-NOW pose reply when scan was requested or on every Nth frame
    static uint32_t s_espnow_frame_counter = 0;
    s_espnow_frame_counter++;

    bool local_scan_req = false;
    uint8_t local_task_id = 0;
    uint8_t local_scan_mode = 0;

    portENTER_CRITICAL(&g_scan_mux);
    if (g_scan_requested) {
        local_scan_req = true;
        local_task_id = g_scan_task_id;
        local_scan_mode = g_scan_mode;
        g_scan_requested = false;
        g_scan_mode = 0;
    }
    portEXIT_CRITICAL(&g_scan_mux);

    bool should_send = local_scan_req || (s_espnow_frame_counter % 10 == 0);

    if (should_send && g_espnow_ready) {
        PoseReply reply = {};
        reply.task_id = local_task_id;

        if (pose_src) {
            reply.pose_valid = 1;
            reply.tx_mm = pose_src->tx;
            reply.ty_mm = pose_src->ty;
            reply.tz_mm = pose_src->tz;
            reply.yaw_deg = pose_src->yaw;
            reply.color = (uint8_t)arm_pose_color_from_text(pose_src->text, pose_src->text_len);
            reply.estimated = pose_src->estimated ? 1 : 0;
            reply.confidence = pose_src->confidence;
        } else {
            reply.pose_valid = 0;
            reply.color = (uint8_t)ARM_COLOR_UNKNOWN;
            reply.estimated = 0;
            reply.confidence = 0.0f;
        }

        esp_now_send(baseMacAddress, (uint8_t *)&reply, sizeof(reply));

        if (local_scan_req) {
            Serial.printf("[ESPNOW] Sent pose reply for task %d: valid=%d color=%d conf=%.2f yaw=%.1f\n",
                          reply.task_id, reply.pose_valid, reply.color, reply.confidence, reply.yaw_deg);
        }
    }

    // Platform detection: run periodically and on scan_mode==1 request
    static uint32_t s_platform_frame_counter = 0;
    s_platform_frame_counter++;
    bool run_platform = (local_scan_mode == 1) || (s_platform_frame_counter % 15 == 0);

    if (run_platform && g_gray_buf) {
        uint32_t pt0 = millis();
        PlatformResult plat = detect_platform(g_gray_buf, g_frame_w, g_frame_h);
        
        xSemaphoreTake(g_platform_mutex, portMAX_DELAY);
        g_platform_detect_ms = millis() - pt0;
        g_last_platform = plat;
        xSemaphoreGive(g_platform_mutex);

        if (plat.detected) {
            Serial.printf("[PLATFORM] Detected: cx=%.0f cy=%.0f w=%.0f h=%.0f conf=%.2f dist=%.0fmm %ums\n",
                          plat.center_x, plat.center_y, plat.width_px, plat.height_px,
                          plat.confidence, plat.distance_mm, (unsigned)g_platform_detect_ms);

            // Send platform position via ESP-NOW when explicitly requested
            if (local_scan_mode == 1 && g_espnow_ready) {
                PoseReply preply = {};
                preply.task_id = local_task_id;
                preply.pose_valid = 1;
                preply.color = 0xFF; // special: platform detection
                preply.estimated = 0;
                float cx_offset = plat.center_x - (g_frame_w / 2.0f);
                float fov_px = g_K.fx;
                preply.yaw_deg = atan2f(cx_offset, fov_px) * 180.0f / M_PI;
                preply.tz_mm = plat.distance_mm;
                preply.tx_mm = plat.center_x;
                preply.ty_mm = plat.center_y;
                preply.confidence = plat.confidence;
                esp_now_send(baseMacAddress, (uint8_t *)&preply, sizeof(preply));
            }
        }
    }

    if (count > 0 || valid > 0 || g_track.active) {
        Serial.printf("[QR] grids=%d decoded=%d obs=%d p=%.2f tracked=%d conf=%.2f age=%ums proc=%ums\n",
                      count,
                      decoded_ok,
                  observed ? 1 : 0,
                  best_p < 0.0f ? 0.0f : best_p,
                      g_track.active ? 1 : 0,
                      g_track.confidence,
                      (unsigned)(g_track.active ? g_track.det.age_ms : 0U),
                      (unsigned)(millis() - t0));
        if (g_track.active && g_track.det.pose_valid) {
            Serial.printf("[POSE] decoded=%s estimated=%s text=\"%s\" x=%.1f y=%.1f z=%.1f roll=%.1f pitch=%.1f yaw=%.1f\n",
                          g_track.det.decoded ? "true" : "false",
                          g_track.det.estimated ? "true" : "false",
                          g_track.det.text,
                          g_track.det.tx,
                          g_track.det.ty,
                          g_track.det.tz,
                          g_track.det.roll,
                          g_track.det.pitch,
                          g_track.det.yaw);
        }
    }

    // Publish snapshot for /data and /status handlers.
    xSemaphoreTake(g_data_mutex, portMAX_DELAY);
    if (g_track.active) {
        g_detections[0] = g_track.det;
        g_num_detections = 1;
    } else {
        memcpy(g_detections, dets, sizeof(QRDetection) * valid);
        g_num_detections = valid;
    }
    g_qr_raw_count = count;
    g_qr_decoded_count = decoded_ok;
    g_frame_id++;
    g_processing_ms = millis() - t0;
    xSemaphoreGive(g_data_mutex);
}

// Escape text payload for JSON serialization.
static int json_escape(char *dst, int dst_sz, const char *src, int src_len) {
    int j = 0;
    for (int i = 0; i < src_len && j < dst_sz - 2; i++) {
        char c = src[i];
        if (c == '"' || c == '\\') {
            dst[j++] = '\\';
            dst[j++] = c;
        } else if ((uint8_t)c >= 0x20) {
            dst[j++] = c;
        }
    }
    dst[j] = '\0';
    return j;
}

// /data endpoint:
// returns latest tracker-centric result with per-detection payload, corners, and pose.
static esp_err_t data_handler(httpd_req_t *req) {
    QRDetection dets[MAX_QR_DETECTIONS];
    int n;
    uint32_t fid, proc_ms;
    float fps;
    int raw_count, decoded_count;

    xSemaphoreTake(g_data_mutex, portMAX_DELAY);
    n = g_num_detections;
    fid = g_frame_id;
    proc_ms = g_processing_ms;
    fps = g_qr_fps;
    raw_count = g_qr_raw_count;
    decoded_count = g_qr_decoded_count;
    memcpy(dets, g_detections, sizeof(QRDetection) * n);
    xSemaphoreGive(g_data_mutex);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char part[384];
    int off = snprintf(part, sizeof(part),
        "{\"frame_id\":%u,\"processing_ms\":%u,\"qr_fps\":%.1f,\"raw_count\":%d,\"decoded_count\":%d,\"qr_codes\":[",
        (unsigned)fid, (unsigned)proc_ms, fps, raw_count, decoded_count);
    httpd_resp_send_chunk(req, part, off);

    for (int i = 0; i < n; i++) {
        char esc[MAX_QR_TEXT_LEN * 2];
        json_escape(esc, sizeof(esc), dets[i].text, dets[i].text_len);
        off = snprintf(part, sizeof(part),
            "%s{\"text\":\"%s\",\"decoded\":%s,\"corners\":[[%.0f,%.0f],[%.0f,%.0f],[%.0f,%.0f],[%.0f,%.0f]],"
            "\"estimated\":%s,\"confidence\":%.3f,\"age_ms\":%u,"
            "\"pose_valid\":%s,\"tx\":%.1f,\"ty\":%.1f,\"tz\":%.1f,"
            "\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f}",
            (i > 0) ? "," : "", esc,
            dets[i].decoded ? "true" : "false",
            dets[i].corners[0][0], dets[i].corners[0][1],
            dets[i].corners[1][0], dets[i].corners[1][1],
            dets[i].corners[2][0], dets[i].corners[2][1],
            dets[i].corners[3][0], dets[i].corners[3][1],
            dets[i].estimated ? "true" : "false",
            dets[i].confidence,
            (unsigned)dets[i].age_ms,
            dets[i].pose_valid ? "true" : "false",
            dets[i].tx, dets[i].ty, dets[i].tz,
            dets[i].roll, dets[i].pitch, dets[i].yaw);
        httpd_resp_send_chunk(req, part, off);
    }
    httpd_resp_send_chunk(req, "]}", 2);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// /stream endpoint (port 81): MJPEG stream primarily for laptop-side visualization.
// Vision processing does not depend on this endpoint.
static esp_err_t stream_handler(httpd_req_t *req) {
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    uint32_t frame_delay = 1000 / STREAM_FPS_TARGET;
    char part_buf[64];

    while (true) {
        uint32_t t0 = millis();
        if (!g_cam_mutex) return ESP_FAIL;
        xSemaphoreTake(g_cam_mutex, portMAX_DELAY);
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            xSemaphoreGive(g_cam_mutex);
            return ESP_FAIL;
        }

        uint8_t *jpg_buf = NULL;
        size_t jpg_len = 0;
        bool converted = false;

        if (fb->format == PIXFORMAT_JPEG) {
            jpg_buf = fb->buf;
            jpg_len = fb->len;
        } else {
            converted = frame2jpg(fb, STREAM_JPEG_QUALITY, &jpg_buf, &jpg_len);
            if (!converted) {
                esp_camera_fb_return(fb);
                xSemaphoreGive(g_cam_mutex);
                continue;
            }
        }

        size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, (unsigned)jpg_len);
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_len);

        if (converted && jpg_buf) free(jpg_buf);
        esp_camera_fb_return(fb);
        xSemaphoreGive(g_cam_mutex);
        if (res != ESP_OK) break;

        uint32_t elapsed = millis() - t0;
        if (elapsed < frame_delay) delay(frame_delay - elapsed);
    }
    return res;
}

// /capture endpoint: single JPEG snapshot.
static esp_err_t capture_handler(httpd_req_t *req) {
    if (!g_cam_mutex) return ESP_FAIL;
    xSemaphoreTake(g_cam_mutex, portMAX_DELAY);
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        xSemaphoreGive(g_cam_mutex);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t res;
    if (fb->format == PIXFORMAT_JPEG) {
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    } else {
        uint8_t *jpg_buf = NULL;
        size_t jpg_len = 0;
        if (frame2jpg(fb, STREAM_JPEG_QUALITY, &jpg_buf, &jpg_len)) {
            res = httpd_resp_send(req, (const char *)jpg_buf, jpg_len);
            free(jpg_buf);
        } else {
            httpd_resp_send_500(req);
            res = ESP_FAIL;
        }
    }
    esp_camera_fb_return(fb);
    xSemaphoreGive(g_cam_mutex);
    return res;
}

// /status endpoint: lightweight health/telemetry summary.
static esp_err_t status_handler(httpd_req_t *req) {
    char buf[320];

    xSemaphoreTake(g_data_mutex, portMAX_DELAY);
    uint32_t proc_ms = g_processing_ms;
    float fps = g_qr_fps;
    int raw = g_qr_raw_count;
    int dec = g_qr_decoded_count;
    int dets = g_num_detections;
    int err = g_last_decode_err;
    int err_f = g_last_decode_err_flip;
    uint32_t fid = g_frame_id;
    bool track_act = g_track.active;
    float track_cf = g_track.confidence;
    uint32_t track_age = g_track.active ? g_track.det.age_ms : 0U;
    xSemaphoreGive(g_data_mutex);

    snprintf(buf, sizeof(buf),
        "{\"camera\":\"%s\",\"free_heap\":%lu,\"psram_free\":%lu,\"qr_processing_ms\":%u,\"qr_fps\":%.1f,\"qr_raw\":%d,\"qr_decoded\":%d,\"qr_detections\":%d,\"decode_err\":%d,\"decode_err_flip\":%d,\"track_active\":%s,\"track_conf\":%.3f,\"track_age_ms\":%u,\"frame_id\":%u}",
        g_camera_ok ? "OK" : "FAILED",
        (unsigned long)ESP.getFreeHeap(),
        (unsigned long)ESP.getFreePsram(),
        (unsigned)proc_ms,
        fps,
        raw,
        dec,
        dets,
        err,
        err_f,
        track_act ? "true" : "false",
        track_cf,
        (unsigned)track_age,
        (unsigned)fid);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, buf);
}

// /platform endpoint: latest platform detection result.
static esp_err_t platform_handler(httpd_req_t *req) {
    char buf[256];
    
    xSemaphoreTake(g_platform_mutex, portMAX_DELAY);
    PlatformResult plat = g_last_platform;
    uint32_t p_ms = g_platform_detect_ms;
    xSemaphoreGive(g_platform_mutex);

    snprintf(buf, sizeof(buf),
        "{\"detected\":%s,\"center_x\":%.1f,\"center_y\":%.1f,\"width\":%.1f,\"height\":%.1f,\"angle\":%.1f,\"confidence\":%.3f,\"distance_mm\":%.1f,\"processing_ms\":%u}",
        plat.detected ? "true" : "false",
        plat.center_x,
        plat.center_y,
        plat.width_px,
        plat.height_px,
        plat.angle_deg,
        plat.confidence,
        plat.distance_mm,
        (unsigned)p_ms);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, buf);
}

// Start two HTTP servers:
// - port 80 for JSON API + capture + platform
// - port 81 for MJPEG stream
static void startServer() {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.ctrl_port = 32768;
    cfg.max_uri_handlers = 8;
    cfg.stack_size = 8192;

    httpd_uri_t api_uris[] = {
        {"/capture", HTTP_GET, capture_handler, NULL},
        {"/data", HTTP_GET, data_handler, NULL},
        {"/status", HTTP_GET, status_handler, NULL},
        {"/platform", HTTP_GET, platform_handler, NULL},
    };

    if (httpd_start(&g_httpd, &cfg) == ESP_OK) {
        for (auto &u : api_uris) httpd_register_uri_handler(g_httpd, &u);
        Serial.println("API server:    port 80  (/capture /data /status /platform)");
    }

    httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
    scfg.server_port = 81;
    scfg.ctrl_port = 32769;
    scfg.max_uri_handlers = 2;
    scfg.stack_size = 8192;

    httpd_uri_t stream_uri = {"/stream", HTTP_GET, stream_handler, NULL};
    if (httpd_start(&g_stream_httpd, &scfg) == ESP_OK) {
        httpd_register_uri_handler(g_stream_httpd, &stream_uri);
        Serial.println("Stream server: port 81  (/stream)");
    }
}

// Initialize camera driver and tune sensor defaults for QR readability.
static bool initCamera() {
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = STREAM_FRAME_SIZE;
    config.jpeg_quality = STREAM_JPEG_QUALITY;
    config.fb_count = 2;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        // Keep sensor output close to neutral and let the software pipeline do
        // most of the contrast shaping for QR decode recovery.
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, -1);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_gain_ctrl(s, 1);
    }
    return true;
}

// ESP-NOW callback: receive scan requests from base
static void onEspNowRecv(const uint8_t *mac,
                         const uint8_t *data, int len) {
    if (len == sizeof(ScanRequest)) {
        ScanRequest req;
        memcpy(&req, data, sizeof(req));
        portENTER_CRITICAL_ISR(&g_scan_mux);
        g_scan_task_id = req.task_id;
        g_scan_mode = req.mode;
        g_scan_requested = true;
        portEXIT_CRITICAL_ISR(&g_scan_mux);
        Serial.printf("[ESPNOW] Scan request: task=%d mode=%d\n", req.task_id, req.mode);
    }
}

static void onEspNowSend(const uint8_t *mac, esp_now_send_status_t status) {
    // Optional: log send failures for debugging
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("[ESPNOW] Send failed");
    }
}

// Initialize network and ESP-NOW
static void initWiFi() {
    if (USE_AP_MODE) {
        WiFi.mode(WIFI_AP);
        WiFi.softAP(ap_ssid, ap_password);
        Serial.printf("AP Mode - SSID: %s IP: %s\n", ap_ssid, WiFi.softAPIP().toString().c_str());
    } else {
        WiFi.mode(WIFI_STA);
        WiFi.begin(sta_ssid, sta_password);
        Serial.print("Connecting to WiFi");
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 60) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("\nConnected - IP: %s\n", WiFi.localIP().toString().c_str());
        } else {
            Serial.println("\nWiFi failed, fallback AP mode");
            WiFi.mode(WIFI_AP);
            WiFi.softAP(ap_ssid, ap_password);
            Serial.printf("AP fallback - SSID: %s IP: %s\n", ap_ssid, WiFi.softAPIP().toString().c_str());
        }
    }

    Serial.print("CAMERA MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.print("CAMERA CHANNEL: ");
    Serial.println(WiFi.channel());
}

static void initEspNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNOW] Init FAILED");
        return;
    }

    esp_now_register_recv_cb(onEspNowRecv);
    esp_now_register_send_cb(onEspNowSend);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, baseMacAddress, 6);
    peerInfo.channel = WiFi.channel();
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESPNOW] Failed to add base peer");
        return;
    }

    g_espnow_ready = true;
    Serial.println("[ESPNOW] Ready — base peer added");
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    if (LED_FLASH_PIN >= 0) {
        pinMode(LED_FLASH_PIN, OUTPUT);
        digitalWrite(LED_FLASH_PIN, LOW);
    }

    g_data_mutex = xSemaphoreCreateMutex();
    g_cam_mutex = xSemaphoreCreateMutex();
    g_platform_mutex = xSemaphoreCreateMutex();

    initWiFi();
    initEspNow();

    g_camera_ok = false;
    for (int i = 0; i < 3; i++) {
        if (initCamera()) {
            g_camera_ok = true;
            break;
        }
        delay(1500);
    }

    int fw = 640, fh = 480;
    float fov_rad = CAMERA_FOV_DEG * M_PI / 180.0f;
    g_K.fx = (fw / 2.0f) / tanf(fov_rad / 2.0f);
    g_K.fy = g_K.fx;
    g_K.cx = fw / 2.0f;
    g_K.cy = fh / 2.0f;

    if (g_camera_ok) init_qr(fw, fh);

    // Initialize platform detector
    if (g_camera_ok) {
        if (!platform_detect_init(fw, fh)) {
            Serial.println("[WARN] Platform detector init failed (non-critical)");
        }
    }

    startServer();

    if (g_camera_ok && g_qr && g_gray_buf && g_proc_buf) {
        xTaskCreatePinnedToCore(
            [](void *) {
                // qr_task loops forever and updates g_qr_fps every 2 seconds.
                uint32_t fps_count = 0;
                uint32_t fps_t0 = millis();
                while (true) {
                    process_qr_frame();
                    vTaskDelay(1);
                    fps_count++;
                    uint32_t now = millis();
                    if (now - fps_t0 >= 2000) {
                        g_qr_fps = fps_count * 1000.0f / (now - fps_t0);
                        fps_count = 0;
                        fps_t0 = now;
                    }
                }
            },
            "qr_task",
            32768,
            NULL,
            1,
            &g_qr_task,
            1
        );
    }

    Serial.println("Ready");
}

void loop() {
    // loop() is intentionally lightweight.
    // Real vision work runs continuously in qr_task and HTTP handlers.
    static uint32_t last = 0;

    // Emit a low-rate heartbeat for field diagnostics (every 10 seconds).
    if (millis() - last > 10000) {
        Serial.printf("[Health] heap=%lu psram=%lu wifi=%s cam=%s espnow=%s qr_fps=%.1f det=%d proc=%ums\n",
            (unsigned long)ESP.getFreeHeap(),
            (unsigned long)ESP.getFreePsram(),
            (WiFi.status() == WL_CONNECTED || USE_AP_MODE) ? "OK" : "DOWN",
            g_camera_ok ? "OK" : "FAIL",
            g_espnow_ready ? "OK" : "DOWN",
            g_qr_fps,
            g_num_detections,
            (unsigned)g_processing_ms);
        last = millis();
    }

    // Sleep to avoid busy-waiting; qr_task keeps processing frames meanwhile.
    delay(1000);
}
