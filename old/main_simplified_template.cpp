/*
 * ESP32-S3 Vision - SIMPLIFIED VERSION (No RTOS)
 * Uses simple state machines and millis() timing instead of FreeRTOS
 */

#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include "quirc.h"
#include "platform_detect.h"
#include <math.h>
// Removed: #include <freertos/semphr.h>

// =================== NETWORK CONFIG ===================
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

// SIMPLIFIED: Keep ONLY g_data_mutex for HTTP handler safety
// ESP32 HTTP server runs handlers in separate tasks
static SemaphoreHandle_t g_data_mutex = NULL;

// =================== TIMING CONTROL ===================
static unsigned long last_qr_process_time = 0;
static const unsigned long QR_PROCESS_INTERVAL = 50; // 50ms = 20 FPS
static unsigned long last_health_print = 0;
static const unsigned long HEALTH_INTERVAL = 10000; // 10 seconds

// =================== SIMPLIFIED UDP SEND ===================
static bool sendUdpPacketToBase(const uint8_t *data, size_t len) {
    if (!g_udp_ready) return false;
    
    // No mutex needed - called from single-threaded loop()
    bool ok = udp.beginPacket(BASE_IP, GLITCH_UDP_PORT);
    if (ok) {
        udp.write(data, len);
        ok = (udp.endPacket() == 1);
    }
    
    return ok;
}

// =================== CAMERA CONFIG ===================
// ... (Keep all camera config the same)

// =================== QR PROCESSING (SIMPLIFIED) ===================
// Removed all mutex protection - single-threaded
static void process_qr_frame() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        return;
    }
    
    // ... (QR processing code - same as before but remove all xSemaphoreTake/xSemaphoreGive)
    
    esp_camera_fb_return(fb);
}

// =================== UDP RECEIVE (SIMPLIFIED) ===================
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
        // ... (Same UDP packet handling code)
    }
}

// =================== SETUP ===================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // ... (Camera init)
    
    // Init WiFi
    initWiFi();
    
    // Init UDP
    if (!udp.begin(GLITCH_UDP_PORT)) {
        Serial.println("[UDP] Failed");
    } else {
        g_udp_ready = true;
    }
    
    // Init HTTP server
    startServer();
    
    // Create ONLY g_data_mutex for HTTP handlers
    g_data_mutex = xSemaphoreCreateMutex();
    
    Serial.println("Ready (Simplified - No RTOS)");
}

// =================== MAIN LOOP (SIMPLIFIED) ===================
void loop() {
    // Process UDP requests
    processUdpRx();
    
    // Non-blocking QR processing with timing control
    if (millis() - last_qr_process_time > QR_PROCESS_INTERVAL) {
        process_qr_frame();
        last_qr_process_time = millis();
    }
    
    // Health print
    if (millis() - last_health_print > HEALTH_INTERVAL) {
        Serial.printf("[Health] heap=%lu udp=%s\n", 
            (unsigned long)ESP.getFreeHeap(), 
            g_udp_ready ? "OK" : "DOWN");
        last_health_print = millis();
    }
    
    delay(10); // Small delay to prevent watchdog
}

// =================== HTTP HANDLERS ===================
// Keep g_data_mutex protection for data sharing with HTTP tasks
static esp_err_t data_handler(httpd_req_t *req) {
    if (g_data_mutex) xSemaphoreTake(g_data_mutex, portMAX_DELAY);
    // ... (Copy data to local buffer)
    if (g_data_mutex) xSemaphoreGive(g_data_mutex);
    
    // ... (Send JSON response)
}