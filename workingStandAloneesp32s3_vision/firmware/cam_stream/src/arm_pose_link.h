#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <stdint.h>

enum ArmColorCode : uint8_t {
    ARM_COLOR_UNKNOWN = 0,
    ARM_COLOR_R = 1,
    ARM_COLOR_G = 2,
    ARM_COLOR_B = 3,
};

struct __attribute__((packed)) ArmPosePacket {
    float x_mm;
    float y_mm;
    float z_mm;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint8_t color;
    uint8_t pose_valid;
    uint16_t reserved;
};

void arm_pose_link_begin(const IPAddress &remote_ip, uint16_t remote_port);
bool arm_pose_link_send(const ArmPosePacket &packet);
ArmColorCode arm_pose_color_from_text(const char *text, int text_len);
const char *arm_pose_color_to_string(ArmColorCode color);
