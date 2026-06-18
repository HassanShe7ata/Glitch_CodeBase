#include "arm_pose_link.h"

#include <WiFiUdp.h>
#include <ctype.h>
#include <string.h>

static WiFiUDP g_arm_udp;
static IPAddress g_arm_remote_ip(0, 0, 0, 0);
static uint16_t g_arm_remote_port = 0;
static bool g_arm_link_ready = false;

void arm_pose_link_begin(const IPAddress &remote_ip, uint16_t remote_port) {
    g_arm_remote_ip = remote_ip;
    g_arm_remote_port = remote_port;
    g_arm_link_ready = (g_arm_udp.begin(0) == 1);
}

static bool wifi_ready_for_udp() {
    wifi_mode_t mode = WiFi.getMode();
    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) {
        return true;
    }
    return WiFi.status() == WL_CONNECTED;
}

bool arm_pose_link_send(const ArmPosePacket &packet) {
    if (!g_arm_link_ready || g_arm_remote_port == 0) {
        return false;
    }
    if (!wifi_ready_for_udp()) {
        return false;
    }

    if (!g_arm_udp.beginPacket(g_arm_remote_ip, g_arm_remote_port)) {
        return false;
    }

    size_t wrote = g_arm_udp.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    if (wrote != sizeof(packet)) {
        g_arm_udp.endPacket();
        return false;
    }

    return g_arm_udp.endPacket() == 1;
}

static bool starts_with_ci(const char *text, int text_len, const char *prefix) {
    int n = (int)strlen(prefix);
    if (!text || text_len < n) {
        return false;
    }

    for (int i = 0; i < n; i++) {
        char a = text[i];
        char b = prefix[i];
        if (a >= 'a' && a <= 'z') {
            a = (char)(a - 'a' + 'A');
        }
        if (b >= 'a' && b <= 'z') {
            b = (char)(b - 'a' + 'A');
        }
        if (a != b) {
            return false;
        }
    }
    return true;
}

ArmColorCode arm_pose_color_from_text(const char *text, int text_len) {
    if (!text || text_len <= 0) {
        return ARM_COLOR_UNKNOWN;
    }

    if (starts_with_ci(text, text_len, "R") || starts_with_ci(text, text_len, "RED")) {
        return ARM_COLOR_R;
    }
    if (starts_with_ci(text, text_len, "G") || starts_with_ci(text, text_len, "GREEN")) {
        return ARM_COLOR_G;
    }
    if (starts_with_ci(text, text_len, "B") || starts_with_ci(text, text_len, "BLUE")) {
        return ARM_COLOR_B;
    }

    for (int i = 0; i < text_len; i++) {
        char c = text[i];
        if (c >= 'a' && c <= 'z') {
            c = (char)(c - 'a' + 'A');
        }
        if (c == 'R') {
            return ARM_COLOR_R;
        }
        if (c == 'G') {
            return ARM_COLOR_G;
        }
        if (c == 'B') {
            return ARM_COLOR_B;
        }
    }

    return ARM_COLOR_UNKNOWN;
}

const char *arm_pose_color_to_string(ArmColorCode color) {
    switch (color) {
        case ARM_COLOR_R:
            return "R";
        case ARM_COLOR_G:
            return "G";
        case ARM_COLOR_B:
            return "B";
        default:
            return "UNKNOWN";
    }
}
