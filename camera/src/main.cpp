/*
 * ESP32-S3 Vision - On-device QR + Pose Estimation
 * Integrated with Glitch robot project via ESP-NOW.
 */

#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include "quirc.h"
#include "arm_pose_link.h"
#include <math.h>
#include <freertos/semphr.h>

// WiFi: join GLITCH AP on channel 11
static const char *sta_ssid = "GLITCH";
static const char *sta_password = "Gl1tch2024!Secure";
#define WIFI_CHANNEL 11

// Base ESP32 MAC Address (for ESP-NOW)
static uint8_t baseAddress[] = {0x80, 0xF3, 0xDA, 0x42, 0x3E, 0x5D}; // base AP MAC (NOT STA MAC)

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

// ESP-NOW packet types (must match base.ino)
enum EspNowPacketType : uint8_t {
    ESPNOW_TYPE_SCAN_REQ   = 0x20,
    ESPNOW_TYPE_POSE_REPLY = 0x30,
};

// ESP-NOW packet: Base -> Camera (scan request)
struct __attribute__((packed)) ScanRequest {
    uint8_t type;        // ESPNOW_TYPE_SCAN_REQ
    uint8_t task_id;
    uint8_t mode;        // 0=scan_qr, 1=scan_platform
    uint8_t reserved;
};

// ESP-NOW packet: Camera -> Base (pose reply)
struct __attribute__((packed)) PoseReply {
    uint8_t type;        // ESPNOW_TYPE_POSE_REPLY
    uint8_t task_id;
    uint8_t pose_valid;
    uint8_t color;       // ArmColorCode enum
    uint8_t estimated;
    float tx_mm;
    float ty_mm;
    float tz_mm;
    float yaw_deg;
    float confidence;
    uint8_t status;      // 0=Accumulating, 1=DONE
};

static volatile bool g_scan_requested = false;
static volatile uint8_t g_scan_task_id = 0;
static volatile uint8_t g_scan_mode = 0;
static portMUX_TYPE g_scan_mux = portMUX_INITIALIZER_UNLOCKED;

// Scan lifecycle states
enum ScanState : uint8_t {
    SCAN_SLEEP  = 0,   // qr_task blocked on semaphore, no processing
    SCAN_ACTIVE = 1,   // processing frames, accumulating detections
};
static volatile ScanState g_scan_state = SCAN_SLEEP;
static volatile bool g_scan_restart = false;  // set by callback, checked by qr_task
static SemaphoreHandle_t g_scan_wake_sem = NULL;

// =================== ESP-NOW TRANSPORT ===================
static bool g_espnow_ready = false;
static uint32_t espNowSendOk = 0;
static uint32_t espNowSendFail = 0;
static uint32_t espNowRxValid = 0;
static uint32_t espNowRxInvalid = 0;

static bool basePeerKnown() {
    for (uint8_t b : baseAddress) {
        if (b != 0) return true;
    }
    return false;
}

static bool sendEspNowPacketToBase(const uint8_t *data, size_t len) {
    if (!g_espnow_ready || !basePeerKnown()) return false;
    esp_err_t result = esp_now_send(baseAddress, data, len);
    bool ok = (result == ESP_OK);
    if (ok) espNowSendOk++;
    else espNowSendFail++;
    return ok;
}

// ESP-NOW receive callback — handles scan requests from base
static void OnEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (!mac || !data || len < 1) { espNowRxInvalid++; return; }
    uint8_t pktType = data[0];
    if (pktType == ESPNOW_TYPE_SCAN_REQ && len == (int)sizeof(ScanRequest)) {
        ScanRequest req;
        memcpy(&req, data, sizeof(req));
        portENTER_CRITICAL(&g_scan_mux);
        g_scan_task_id = req.task_id;
        g_scan_mode = req.mode;
        g_scan_requested = true;
        portEXIT_CRITICAL(&g_scan_mux);
        espNowRxValid++;
        Serial.printf("[ESP-NOW] Scan request: task=%d mode=%d\n", req.task_id, req.mode);

        // Wake qr_task from sleep, or restart if already scanning
        if (!g_scan_wake_sem) { espNowRxInvalid++; return; }  // camera not initialized
        portENTER_CRITICAL(&g_scan_mux);
        ScanState st = g_scan_state;
        portEXIT_CRITICAL(&g_scan_mux);
        if (st == SCAN_SLEEP) {
            // Task will set SCAN_ACTIVE after taking semaphore — just wake it
            xSemaphoreGive(g_scan_wake_sem);
        } else {
            // Already scanning — signal qr_task to restart (no direct mutation)
            portENTER_CRITICAL(&g_scan_mux);
            g_scan_restart = true;
            portEXIT_CRITICAL(&g_scan_mux);
            Serial.printf("[SCAN] Restart requested by new scan (task=%d)\n", req.task_id);
        }
        return;
    }
    espNowRxInvalid++;
}

static void initEspNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init failed");
        return;
    }
    esp_now_register_recv_cb(OnEspNowRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, baseAddress, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESP-NOW] Failed to add base peer");
    } else {
        Serial.println("[ESP-NOW] Base peer added — ready");
        g_espnow_ready = true;
    }
}

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

// Accumulation state (only accessed from qr_task, no lock needed)
static QRDetection g_best_detection;
static float g_best_prob = -1.0f;
static uint32_t g_stable_count = 0;
static uint32_t g_scan_frame_count = 0;
static uint32_t g_scan_start_ms = 0;
static uint8_t g_scan_retries = 0;

// Track last sent values to avoid redundant ESP-NOW sends
static float g_last_sent_confidence = -1.0f;
static float g_last_sent_tx = 0, g_last_sent_ty = 0, g_last_sent_tz = 0;
static float g_last_sent_yaw = 0;
static uint8_t g_last_sent_color = 0;
static bool g_initial_sent = false;

// Scan accumulation thresholds
#define SCAN_STABLE_FRAMES     7        // consecutive stable frames before DONE
#define SCAN_TIMEOUT_MS        10000UL  // max scan duration per attempt (10 seconds)
#define SCAN_MAX_RETRIES       3        // auto-retry on timeout with no detection
#define SCAN_MIN_CONFIDENCE    0.65f    // min tracker confidence for stability
#define SCAN_UPDATE_CONF_STEP  0.10f    // send update when confidence improves by >=10%
#define SCAN_UPDATE_POSE_STEP  5.0f     // send update when pose changes by >=5mm
#define SCAN_UPDATE_YAW_STEP   5.0f     // send update when yaw changes by >=5 degrees

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
#define TRACK_MAX_HOLD_MS 5000U
#define TRACK_DECAY_TAU_MS 3500.0f
#define TRACK_MIN_CONF 0.08f
#define OBS_ACCEPT_PROB 0.10f

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
    if (ENABLE_PAYLOAD_DECODE) {
        derr = quirc_decode(&code, &data);
    }
    g_last_decode_err = (int)derr;
    g_last_decode_err_flip = 0;

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

// Determine if accumulated data has changed enough to justify an ESP-NOW send.
static bool shouldSendUpdate() {
    if (!g_initial_sent) return true;

    const QRDetection *src = g_track.active ? &g_track.det :
                             (g_best_prob >= 0 ? &g_best_detection : NULL);
    if (!src) return false;

    uint8_t color = (uint8_t)arm_pose_color_from_text(src->text, src->text_len);
    if (color != g_last_sent_color) return true;
    if (src->confidence - g_last_sent_confidence >= SCAN_UPDATE_CONF_STEP) return true;
    if (fabsf(src->tx - g_last_sent_tx) >= SCAN_UPDATE_POSE_STEP) return true;
    if (fabsf(src->ty - g_last_sent_ty) >= SCAN_UPDATE_POSE_STEP) return true;
    if (fabsf(src->tz - g_last_sent_tz) >= SCAN_UPDATE_POSE_STEP) return true;
    if (fabsf(src->yaw - g_last_sent_yaw) >= SCAN_UPDATE_YAW_STEP) return true;
    return false;
}

// Send a PoseReply with the current accumulated data.
// status: 0=Accumulating, 1=DONE
static void sendPoseUpdate(uint8_t status) {
    PoseReply r = {};
    r.type = ESPNOW_TYPE_POSE_REPLY;
    portENTER_CRITICAL(&g_scan_mux);
    r.task_id = g_scan_task_id;
    portEXIT_CRITICAL(&g_scan_mux);
    r.status = status;

    const QRDetection *src = NULL;
    if (g_track.active && g_track.det.pose_valid) {
        src = &g_track.det;
    } else if (g_best_prob >= 0.0f && g_best_detection.pose_valid) {
        src = &g_best_detection;
    }

    if (src) {
        r.tx_mm = src->tx;
        r.ty_mm = src->ty;
        r.tz_mm = src->tz;
        r.yaw_deg = src->yaw;
        r.color = (uint8_t)arm_pose_color_from_text(src->text, src->text_len);
        r.estimated = src->estimated ? 1 : 0;
        r.confidence = src->confidence;
        r.pose_valid = 1;

        g_last_sent_confidence = r.confidence;
        g_last_sent_tx = r.tx_mm;
        g_last_sent_ty = r.ty_mm;
        g_last_sent_tz = r.tz_mm;
        g_last_sent_yaw = r.yaw_deg;
        g_last_sent_color = r.color;
    } else {
        r.color = (uint8_t)ARM_COLOR_UNKNOWN;
        r.pose_valid = 0;
        r.confidence = 0;
    }

    sendEspNowPacketToBase((const uint8_t *)&r, sizeof(r));
    Serial.printf("[SCAN] -> %s: color=%d conf=%.2f pose=(%.0f,%.0f,%.0f) frames=%lu\n",
                  status == 1 ? "DONE" : "ACC",
                  r.color, r.confidence, r.tx_mm, r.ty_mm, r.tz_mm,
                  (unsigned long)g_scan_frame_count);
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

    if (!g_proc_gray_buf) return;

    // --- Build the raw grayscale source buffer ---
    // When DOWNSCALE=1, g_gray_buf IS the processing-resolution image —
    // skip the downsample memcpy entirely.
    uint8_t *raw = g_gray_buf;
    if (QR_DOWNSCALE > 1) {
        downsample_gray(g_gray_buf, w, h, g_proc_raw_buf);
        raw = g_proc_raw_buf;
    }

    // --- Pass 1: raw grayscale ---
    // run_quirc_scan copies src into quirc's internal buffer (read-only on src),
    // so pass raw directly — no intermediate copy to g_proc_buf needed.
    // If adaptive binarize is enabled for the main pass, we do need a mutable copy.
    uint8_t *pass1_src = raw;
    if (QR_USE_ADAPTIVE_BINARIZE && QR_MAIN_USE_ADAPTIVE) {
        memcpy(g_proc_buf, raw, g_proc_w * g_proc_h);
        adaptive_binarize(g_proc_buf, g_proc_w, g_proc_h);
        pass1_src = g_proc_buf;
    }

    QRDetection *dets = g_scan_dets;
    int valid = 0;
    uint32_t now_ms = millis();
    int count = 0;
    int decoded_ok = 0;
    run_quirc_scan(pass1_src, dets, valid, count, decoded_ok);

    // Early exit: if pass 1 decoded successfully, skip all retry passes
    static uint32_t s_decode_fail_streak = 0;
    if (decoded_ok > 0) {
        s_decode_fail_streak = 0;
    } else {
        s_decode_fail_streak++;
    }

    // Pass 2+: only run contrast-stretched pass (cap at 2 passes total)
    bool run_retry_passes = (count == 0);
    if (!run_retry_passes && decoded_ok == 0) {
        run_retry_passes = ((s_decode_fail_streak % 4) == 0);
    }

    // Pass 2: contrast-stretched grayscale (final retry — passes 3/4 removed for speed)
    // Deferred: only prepare g_proc_gray_buf when pass 2 is actually needed.
    if (run_retry_passes) {
        memcpy(g_proc_gray_buf, raw, g_proc_w * g_proc_h);
        contrast_stretch(g_proc_gray_buf, g_proc_w * g_proc_h);

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

    // Accumulation: send partial results during scan, final result on completion
    if (g_scan_state == SCAN_ACTIVE) {
        g_scan_frame_count++;

        // Update best detection across all frames
        if (best_idx >= 0 && best_p > g_best_prob) {
            g_best_detection = dets[best_idx];
            g_best_prob = best_p;
        }

        // Check stability: tracker active, pose valid, decoded, confidence high, same color
        bool frame_stable = (g_track.active &&
                             g_track.det.pose_valid &&
                             g_track.det.decoded &&
                             g_track.confidence >= SCAN_MIN_CONFIDENCE);

        if (frame_stable) {
            uint8_t frame_color = (uint8_t)arm_pose_color_from_text(
                g_track.det.text, g_track.det.text_len);
            uint8_t best_color = (uint8_t)arm_pose_color_from_text(
                g_best_detection.text, g_best_detection.text_len);
            if (frame_color == best_color && best_color != ARM_COLOR_UNKNOWN) {
                g_stable_count++;
            } else {
                g_stable_count = 0;
            }
        } else {
            g_stable_count = 0;
        }

        uint32_t elapsed = millis() - g_scan_start_ms;
        bool timeout_reached = (elapsed >= SCAN_TIMEOUT_MS);
        bool stable_enough = (g_stable_count >= SCAN_STABLE_FRAMES);
        const QRDetection *src = g_track.active ? &g_track.det :
                                 (g_best_prob >= 0 ? &g_best_detection : NULL);
        bool has_detection = (src && src->pose_valid && src->confidence > 0.0f &&
                              arm_pose_color_from_text(src->text, src->text_len) != ARM_COLOR_UNKNOWN);

        if (stable_enough || (timeout_reached && has_detection)) {
            // SUCCESS: valid detection found — send DONE, return to sleep
            sendPoseUpdate(1);
            portENTER_CRITICAL(&g_scan_mux);
            g_scan_state = SCAN_SLEEP;
            portEXIT_CRITICAL(&g_scan_mux);
            g_stable_count = 0;
            g_scan_frame_count = 0;
            g_scan_retries = 0;
            g_initial_sent = false;
            g_last_sent_confidence = -1.0f;
        } else if (timeout_reached && !has_detection) {
            // TIMEOUT with no detection — auto-retry or give up
            g_scan_retries++;
            if (g_scan_retries < SCAN_MAX_RETRIES) {
                Serial.printf("[SCAN] No detection on attempt %d/%d — retrying\n",
                              g_scan_retries, SCAN_MAX_RETRIES);
                g_scan_start_ms = millis();
                g_scan_frame_count = 0;
                g_stable_count = 0;
                memset(&g_best_detection, 0, sizeof(g_best_detection));
                g_best_prob = -1.0f;
                g_track.active = false;
                g_initial_sent = false;
                g_last_sent_confidence = -1.0f;
                g_last_sent_yaw = 0;
                g_last_sent_color = 0;
            } else {
                // Exhausted retries — give up, send DONE with no detection
                Serial.printf("[SCAN] No detection after %d attempts — giving up\n", g_scan_retries);
                sendPoseUpdate(1);
                portENTER_CRITICAL(&g_scan_mux);
                g_scan_state = SCAN_SLEEP;
                portEXIT_CRITICAL(&g_scan_mux);
                g_stable_count = 0;
                g_scan_frame_count = 0;
                g_scan_retries = 0;
                g_initial_sent = false;
                g_last_sent_confidence = -1.0f;
            }
        } else if (shouldSendUpdate()) {
            // PARTIAL: send with status=Accumulating
            sendPoseUpdate(0);
            g_initial_sent = true;
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

    char *buf = (char *)malloc(4096);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int off = snprintf(buf, 4096,
        "{\"frame_id\":%u,\"processing_ms\":%u,\"qr_fps\":%.1f,\"raw_count\":%d,\"decoded_count\":%d,\"qr_codes\":[",
        (unsigned)fid, (unsigned)proc_ms, fps, raw_count, decoded_count);

    for (int i = 0; i < n; i++) {
        if (i > 0) buf[off++] = ',';
        char esc[MAX_QR_TEXT_LEN * 2];
        json_escape(esc, sizeof(esc), dets[i].text, dets[i].text_len);
        off += snprintf(buf + off, 4096 - off,
            "{\"text\":\"%s\",\"decoded\":%s,\"corners\":[[%.0f,%.0f],[%.0f,%.0f],[%.0f,%.0f],[%.0f,%.0f]],"
            "\"estimated\":%s,\"confidence\":%.3f,\"age_ms\":%u,"
            "\"pose_valid\":%s,\"tx\":%.1f,\"ty\":%.1f,\"tz\":%.1f,"
            "\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f}",
            esc,
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
    }
    off += snprintf(buf + off, 4096 - off, "]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t res = httpd_resp_send(req, buf, off);
    free(buf);
    return res;
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
        camera_fb_t *fb = NULL;
        for (int retry = 0; retry < 3; retry++) {
            fb = esp_camera_fb_get();
            if (fb) break;
            xSemaphoreGive(g_cam_mutex);
            delay(50);
            xSemaphoreTake(g_cam_mutex, portMAX_DELAY);
        }
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
    snprintf(buf, sizeof(buf),
        "{\"camera\":\"%s\",\"free_heap\":%lu,\"psram_free\":%lu,\"qr_processing_ms\":%u,\"qr_fps\":%.1f,\"qr_raw\":%d,\"qr_decoded\":%d,\"qr_detections\":%d,\"decode_err\":%d,\"decode_err_flip\":%d,\"track_active\":%s,\"track_conf\":%.3f,\"track_age_ms\":%u,\"frame_id\":%u}",
        g_camera_ok ? "OK" : "FAILED",
        (unsigned long)ESP.getFreeHeap(),
        (unsigned long)ESP.getFreePsram(),
        (unsigned)g_processing_ms,
        g_qr_fps,
        g_qr_raw_count,
        g_qr_decoded_count,
        g_num_detections,
        g_last_decode_err,
        g_last_decode_err_flip,
        g_track.active ? "true" : "false",
        g_track.confidence,
        (unsigned)(g_track.active ? g_track.det.age_ms : 0U),
        (unsigned)g_frame_id);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, buf);
}

// Start two HTTP servers:
// - port 80 for JSON API + capture
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
    };

    if (httpd_start(&g_httpd, &cfg) == ESP_OK) {
        for (auto &u : api_uris) httpd_register_uri_handler(g_httpd, &u);
        Serial.println("API server:    port 80  (/capture /data /status)");
    }

    httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
    scfg.server_port = 81;
    scfg.ctrl_port = 32769;
    scfg.max_uri_handlers = 2;
    scfg.stack_size = 16384;

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

// Initialize network in STA mode — join GLITCH AP
static void initWiFi() {
    WiFi.mode(WIFI_STA);
    // Set static IP so the dashboard always knows where the stream is
    IPAddress staticIP(192, 168, 4, 100);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.config(staticIP, gateway, subnet);

    WiFi.begin(sta_ssid, sta_password);
    Serial.print("Connecting to GLITCH AP");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 60) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nConnected - IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWiFi connection failed!");
    }
}

void setup() {
    // Boot sequence:
    // 1) serial and basic IO
    // 2) synchronization primitives
    // 3) network + camera
    // 4) QR engine init
    // 5) start API/stream servers
    // 6) start dedicated QR processing task
    Serial.begin(115200);
    delay(1000);

    if (LED_FLASH_PIN >= 0) {
        pinMode(LED_FLASH_PIN, OUTPUT);
        digitalWrite(LED_FLASH_PIN, LOW);
    }

    g_data_mutex = xSemaphoreCreateMutex();
    g_cam_mutex = xSemaphoreCreateMutex();

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

    startServer(); // Re-enabled for dashboard camera stream

    if (g_camera_ok && g_qr && g_gray_buf && g_proc_buf) {
        g_scan_wake_sem = xSemaphoreCreateBinary();

        xTaskCreatePinnedToCore(
            [](void *) {
                uint32_t fps_count = 0;
                uint32_t fps_t0 = millis();
                while (true) {
                    // Sleep until scan request wakes us
                    if (g_scan_state == SCAN_SLEEP) {
                        xSemaphoreTake(g_scan_wake_sem, portMAX_DELAY);
                        // Woken — transition to ACTIVE and initialize scan state
                        portENTER_CRITICAL(&g_scan_mux);
                        g_scan_state = SCAN_ACTIVE;
                        portEXIT_CRITICAL(&g_scan_mux);
                        g_scan_start_ms = millis();
                        g_scan_frame_count = 0;
                        g_stable_count = 0;
                        memset(&g_best_detection, 0, sizeof(g_best_detection));
                        g_best_prob = -1.0f;
                        g_track.active = false;
                        g_initial_sent = false;
                        g_last_sent_confidence = -1.0f;
                        g_last_sent_yaw = 0;
                        g_last_sent_color = 0;
                        g_scan_retries = 0;
                        Serial.printf("[SCAN] Woken for task=%d\n", g_scan_task_id);
                    }

                    // Handle mid-scan restart (set by ESP-NOW callback)
                    bool need_restart = false;
                    portENTER_CRITICAL(&g_scan_mux);
                    if (g_scan_restart) {
                        g_scan_restart = false;
                        need_restart = true;
                    }
                    portEXIT_CRITICAL(&g_scan_mux);
                    if (need_restart) {
                        g_scan_start_ms = millis();
                        g_scan_frame_count = 0;
                        g_stable_count = 0;
                        g_scan_retries = 0;
                        memset(&g_best_detection, 0, sizeof(g_best_detection));
                        g_best_prob = -1.0f;
                        g_track.active = false;
                        g_initial_sent = false;
                        g_last_sent_confidence = -1.0f;
                        g_last_sent_yaw = 0;
                        g_last_sent_color = 0;
                        Serial.printf("[SCAN] Restarted for task=%d\n", g_scan_task_id);
                    }

                    process_qr_frame();
                    delay(5); // Yield CPU to stream handler

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

    // WiFi reconnect (throttled — same pattern as arm firmware)
    static unsigned long lastWifiAttempt = 0;
    static bool wasWifiConnected = false;
    bool isWifiConnected = (WiFi.status() == WL_CONNECTED);

    if (!wasWifiConnected && isWifiConnected) {
        Serial.println("[WiFi] Reconnected, re-adding ESP-NOW peer on correct channel");
        esp_now_del_peer(baseAddress);
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, baseAddress, 6);
        peerInfo.channel = WIFI_CHANNEL;
        peerInfo.encrypt = false;
        peerInfo.ifidx = WIFI_IF_STA;
        if (esp_now_add_peer(&peerInfo) == ESP_OK) {
            g_espnow_ready = true;
            Serial.println("[ESP-NOW] Base peer re-added — ready");
        }
    }
    wasWifiConnected = isWifiConnected;

    if (!isWifiConnected && millis() - lastWifiAttempt > 10000) {
        lastWifiAttempt = millis();
        Serial.println("[WiFi] Retrying connection to GLITCH AP...");
        WiFi.disconnect();
        delay(100);
        WiFi.mode(WIFI_STA);
        esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
        WiFi.begin(sta_ssid, sta_password);
        esp_wifi_set_ps(WIFI_PS_NONE);
    }

    // Emit a low-rate heartbeat for field diagnostics (every 10 seconds).
    if (millis() - last > 10000) {
        Serial.printf("[Health] heap=%lu psram=%lu wifi=%s(%s ch=%d) cam=%s qr_fps=%.1f det=%d proc=%ums espnow_tx=%lu/%lu rx=%lu/%lu\n",
            (unsigned long)ESP.getFreeHeap(),
            (unsigned long)ESP.getFreePsram(),
            (WiFi.status() == WL_CONNECTED) ? "OK" : "DOWN",
            WiFi.SSID().c_str(), WiFi.channel(),
            g_camera_ok ? "OK" : "FAIL",
            g_qr_fps,
            g_num_detections,
            (unsigned)g_processing_ms,
            (unsigned long)espNowSendOk, (unsigned long)espNowSendFail,
            (unsigned long)espNowRxValid, (unsigned long)espNowRxInvalid);
        last = millis();
    }

    // Sleep to avoid busy-waiting; qr_task keeps processing frames meanwhile.
    delay(1000);
}
