/*
 * ESP32-S3 Vision - On-device QR + Pose Estimation (SIMPLIFIED - NO RTOS)
 * Communicates with base ESP32 via UDP.
 * Serves MJPEG stream and JSON API for dashboard.
 * 
 * SIMPLIFICATIONS:
 * - Removed FreeRTOS task for QR processing
 * - QR processing now called from loop() with millis() timing
 * - Removed most mutexes (single-threaded loop)
 * - Keep minimal protection for HTTP handler data sharing
 */

#include <Arduino.h>
#include "esp_camera.h"
#include "WiFi.h"
#include "WiFiUdp.h"
#include "esp_http_server.h"
#include "img_converters.h"
#include "quirc.h"
#include "platform_detect.h"
#include <math.h>
// Removed: #include <freertos/semphr.h>

// =================== NETWORK CONFIG ===================
// Connect to the Base ESP32 AP so UDP node traffic and the camera HTTP
// dashboard endpoints share the same WiFi channel.
#define USE_AP_MODE false

static const char *ap_ssid = "ESP32S3-CAM";
static const char *ap_password = "12345678";
static const char *sta_ssid = "GLITCH";
static const char *sta_password = "12345678";

static IPAddress CAMERA_IP(192, 168, 4, 202);
static IPAddress BASE_IP(192, 168, 4, 1);
static IPAddress UDP_NETMASK(255, 255, 255, 0);
static const uint16_t GLITCH_UDP_PORT = 4210;

static WiFiUDP udp;
static bool g_udp_ready = false;

// Removed: static SemaphoreHandle_t g_udp_mutex = NULL;
// Removed: static SemaphoreHandle_t g_platform_mutex = NULL;
// Removed: static SemaphoreHandle_t g_cam_mutex = NULL;
// Keep ONLY g_data_mutex for HTTP handler safety (ESP32 HTTP server uses separate tasks)
static SemaphoreHandle_t g_data_mutex = NULL;

static bool sendUdpPacketToBase(const uint8_t *data, size_t len) {
    if (!g_udp_ready) return false;
    
    // No mutex needed - single-threaded loop()
    bool ok = udp.beginPacket(BASE_IP, GLITCH_UDP_PORT);
    if (ok) {
        udp.write(data, len);
        ok = (udp.endPacket() == 1);
    }
    
    return ok;
}

// =================== CAMERA CONFIG ===================
// ... (camera config remains the same)

// =================== QR PROCESSING STATE ===================
static unsigned long last_qr_process_time = 0;
static const unsigned long QR_PROCESS_INTERVAL = 50; // Process QR every 50ms

static void process_qr_frame() {
    // Removed: mutex protection - single-threaded
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        return;
    }
    
    uint32_t t0 = millis();
    int w = fb->width, h = fb->height;
    
    // ... (rest of QR processing code - same as before but remove mutex ops)
    
    esp_camera_fb_return(fb);
    
    // NOTE: Removed all xSemaphoreTake/xSemaphoreGive calls
    // Data sharing with HTTP handlers protected by g_data_mutex only
}

// =================== UDP RECEIVE ===================
static void processUdpRx() {
    if (!g_udp_ready) return;
    
    // No mutex needed - single-threaded
    uint8_t buf[64];
    int packetLen = 0;
    while ((packetLen = udp.parsePacket()) > 0) {
        int len = udp.read(buf, sizeof(buf));
        if (len < 1) continue;
        if (udp.remoteIP() != BASE_IP) continue;
        
        uint8_t pktType = buf[0];
        if (pktType == UDP_TYPE_SCAN_REQ && len == (int)sizeof(ScanRequest)) {
            ScanRequest req;
            memcpy(&req, buf, sizeof(req));
            // Removed: portENTER_CRITICAL/portEXIT_CRITICAL
            g_scan_task_id = req.task_id;
            g_scan_mode = req.mode;
            g_scan_requested = true;
            Serial.printf("[UDP] Scan request: task=%d mode=%d\n", req.task_id, req.mode);
        }
    }
}

// =================== SETUP ===================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // ... (camera init code)
    
    // Initialize WiFi
    initWiFi();
    
    // Initialize UDP
    if (!udp.begin(GLITCH_UDP_PORT)) {
        Serial.println("[UDP] Listener start FAILED");
    } else {
        g_udp_ready = true;
        Serial.printf("[UDP] Ready on port %u, base=%s\n", GLITCH_UDP_PORT, BASE_IP.toString().c_str());
    }
    
    // Initialize HTTP server
    startServer();
    
    // Removed: xTaskCreatePinnedToCore() for qr_task
    // QR processing now happens in loop()
    
    Serial.println("Ready (Simplified - No RTOS)");
}

// =================== MAIN LOOP ===================
void loop() {
    // Process UDP requests
    processUdpRx();
    
    // Non-blocking QR processing with timing control
    if (millis() - last_qr_process_time > QR_PROCESS_INTERVAL) {
        process_qr_frame();
        last_qr_process_time = millis();
    }
    
    // Health heartbeat
    static uint32_t last_health = 0;
    if (millis() - last_health > 10000) {
        Serial.printf("[Health] heap=%lu psram=%lu wifi=%s cam=%s udp=%s\n",
            (unsigned long)ESP.getFreeHeap(),
            (unsigned long)ESP.getFreePsram(),
            (WiFi.status() == WL_CONNECTED || USE_AP_MODE) ? "OK" : "DOWN",
            g_camera_ok ? "OK" : "FAIL",
            g_udp_ready ? "OK" : "DOWN");
        last_health = millis();
    }
    
    // Small delay to prevent watchdog issues
    delay(10);
}

// =================== HTTP HANDLERS ===================
// Keep g_data_mutex ONLY for protecting g_detections array
// ESP32 HTTP server runs handlers in separate tasks
static esp_err_t data_handler(httpd_req_t *req) {
    QRDetection dets[MAX_QR_DETECTIONS];
    int n;
    uint32_t fid, proc_ms;
    float fps;
    int raw_count, decoded_count;
    
    if (g_data_mutex) xSemaphoreTake(g_data_mutex, portMAX_DELAY);
    n = g_num_detections;
    fid = g_frame_id;
    proc_ms = g_processing_ms;
    fps = g_qr_fps;
    raw_count = g_qr_raw_count;
    decoded_count = g_qr_decoded_count;
    memcpy(dets, g_detections, sizeof(QRDetection) * n);
    if (g_data_mutex) xSemaphoreGive(g_data_mutex);
    
    // ... (rest of HTTP handler code)
}