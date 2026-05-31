#include "g_head.h"
#include "ConfigStorage.h"
#include "FormationNetwork.h"
#include "MavlinkHandler.h"
#include "Formation.h"

// 全局对象
MyConfig config;
MyStatus status;
MyLine lines[FLIGHT_NUM];

static bool isResetButtonPressed() {
    return digitalRead(SWITCH1) == LOW || digitalRead(BOARD_BUTTON_PIN) == LOW;
}

ConfigStorage storage;
FormationNetwork network(config, status, lines);
MavlinkHandler mavlink(status);
Formation formation(config, status, lines);

unsigned long lastMavHeartbeat = 0;
unsigned long lastFormationUpdate = 0;
unsigned long lastLogTime = 0;

const char* vehicleTypeName(uint8_t vehicleType) {
    switch (vehicleType) {
        case VEHICLE_TYPE_COPTER: return "Copter";
        case VEHICLE_TYPE_PLANE: return "Plane";
        default: return "Unknown";
    }
}

const char* actionReasonCodeName(uint8_t reason) {
    switch (reason) {
        case ACTION_REASON_NA: return "N/A";
        case ACTION_REASON_READY: return "READY";
        case ACTION_REASON_WAIT_LEADER: return "WAIT_LEADER";
        case ACTION_REASON_LINK_UNSTABLE: return "LINK_UNSTABLE";
        case ACTION_REASON_ARM_TEST_COOLDOWN: return "ARM_TEST_COOLDOWN";
        case ACTION_REASON_ALT_TOO_LOW: return "ALT_TOO_LOW";
        case ACTION_REASON_NOT_GUIDED: return "NOT_GUIDED";
        case ACTION_REASON_NOT_ARMED: return "NOT_ARMED";
        case ACTION_REASON_ALREADY_AIRBORNE: return "ALREADY_AIRBORNE";
        case ACTION_REASON_AUTO_DISABLED: return "AUTO_DISABLED";
        case ACTION_REASON_LEADER_NOT_READY: return "LEADER_NOT_READY";
        case ACTION_REASON_SAFETY_BLOCKED: return "SAFETY_BLOCKED";
        case ACTION_REASON_MANUAL_PLANE_ONLY: return "MANUAL_PLANE_ONLY";
        case ACTION_REASON_NOT_REQUESTED: return "NOT_REQUESTED";
        default: return "UNKNOWN_REASON";
    }
}

static float clampFloatValue(float value, float minValue, float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static float distanceMetersE7(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
    float lat_rad = lat1 * 1e-7f * PI / 180.0f;
    float dx = (lon2 - lon1) * 0.011111f * cos(lat_rad);
    float dy = (lat2 - lat1) * 0.011111f;
    return sqrtf(dx * dx + dy * dy);
}

static float bearingDegE7(int32_t fromLat, int32_t fromLon, int32_t toLat, int32_t toLon) {
    float lat_rad = fromLat * 1e-7f * PI / 180.0f;
    float dx = (toLon - fromLon) * 0.011111f * cos(lat_rad);
    float dy = (toLat - fromLat) * 0.011111f;
    float bearing = atan2f(dx, dy) * 180.0f / PI;
    if (bearing < 0.0f) bearing += 360.0f;
    return bearing;
}

static void offsetGpsMetersE7(int32_t lat, int32_t lon, float dist_m, float bearing_deg, int32_t &outLat, int32_t &outLon) {
    float bearing_rad = bearing_deg * PI / 180.0f;
    double lat_offset = (dist_m * cos(bearing_rad)) / 111111.0;
    double lon_offset = (dist_m * sin(bearing_rad)) / (111111.0 * cos(lat * 1e-7 * PI / 180.0));
    outLat = lat + (int32_t)(lat_offset * 1e7);
    outLon = lon + (int32_t)(lon_offset * 1e7);
}

void printDebugLogs() {
    Serial.println("\n--- [System Status Log] ---");
    
    // 1. 飞行器基础信息
    Serial.printf("Mode: %s | NikeName: %s\n", config.AP ? "Leader (AP)" : "Follower (STA)", config.MyNikeName);
    Serial.printf("GPS: %.7f, %.7f | Alt: %.2fm | Sats: %d\n", 
                  status.gpsData.lat / 1e7, status.gpsData.lon / 1e7, 
                  status.pos.alt / 1000.0, status.gpsData.satellites_visible);
    
    // 2. 飞控状态
    bool isArmed = status.heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
    Serial.printf("FC Type: %s | State: %s | Custom Mode: %d\n",
                  vehicleTypeName(status.vehicleType),
                  isArmed ? "ARMED" : "DISARMED",
                  status.heartbeat.custom_mode);
    Serial.printf("Battery: %.2fV\n", status.sysStatus.voltage_battery / 1000.0);

    // 3. WiFi 与 编队状态
    Serial.printf("WiFi: %s | SSID: %s\n", status.byWifi ? "Connected/Active" : "Disconnected", config.chSsid);
    if (!config.AP) {
        Serial.printf("Action Reasons: Takeoff=%s | Land=%s | Join=%s\n",
                      actionReasonCodeName(status.takeoffReason),
                      actionReasonCodeName(status.landReason),
                      actionReasonCodeName(status.joinReason));
        Serial.printf("Diag: LeaderAge=%lums | TxPkt=%lu | LeaderPkt=%lu | LastReject=%u | LastFcAckCmd=%u Result=%u\n",
                      formation.leaderLinkAgeMs(),
                      (unsigned long)status.lastSentPacketCounter,
                      (unsigned long)status.lastLeaderPacketCounter,
                      status.lastPacketRejectReason,
                      status.lastFcAckCommand,
                      status.lastFcAckResult);
    }
    
    if (!config.AP) {
        Serial.print("Formation: ");
        bool isArmed = status.heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
        bool isGuided = mavlink.isGuidedMode();

        if (mavlink.isPlane()) {
            if (formation.isLeaderLost()) {
                Serial.println("Leader Lost -> Waiting / Recovering");
            } else if (status.pos.relative_alt < LOWEST_ALTITUDE) {
                Serial.println("Plane Standby (Manual Takeoff Required)");
            } else if (isGuided && formation.isSafetyOk()) {
                int32_t tLat, tLon; float tAlt;
                if (formation.getTargetPoint(tLat, tLon, tAlt)) {
                    if (formation.leaderData.vehicleType != VEHICLE_TYPE_PLANE || formation.leaderData.groundSpeed < PLANE_LOW_SPEED_LEADER) {
                        Serial.printf("Plane Orbit Target: %.7f, %.7f, Alt: %.2fm\n", tLat/1e7, tLon/1e7, tAlt);
                    } else {
                        Serial.printf("Plane Follow Target: %.7f, %.7f, Alt: %.2fm\n", tLat/1e7, tLon/1e7, tAlt);
                    }
                } else {
                    Serial.println("Waiting for Leader Data...");
                }
            } else {
                Serial.println("Plane Waiting For Pilot GUIDED Join");
            }
        } else if (status.pos.relative_alt < LOWEST_ALTITUDE) {
            float tAlt;
            bool shouldAutoArm = formation.shouldAutoArmForTakeoff();
            if (isGuided && shouldAutoArm && !formation.shouldAutoTakeoff(tAlt)) {
                if (!isArmed) {
                    Serial.println("Auto Arming with Leader...");
                } else {
                    Serial.println("Auto Armed, Waiting Leader Takeoff Height...");
                }
            } else if (isGuided && formation.shouldAutoTakeoff(tAlt)) {
                if (!isArmed) {
                    Serial.println("Auto Arming...");
                } else {
                    Serial.printf("Auto Taking Off to %.2fm...\n", tAlt);
                }
            } else if (isGuided && isArmed && formation.shouldAutoLand()) {
                Serial.println("Auto Landing...");
            } else {
                Serial.println("Standby (On Ground / Low Alt)");
            }
        } else if (formation.isSafetyOk()) {
            int32_t tLat, tLon; float tAlt;
            if (formation.getTargetPoint(tLat, tLon, tAlt)) {
                Serial.printf("OK! Target: %.7f, %.7f, Alt: %.2fm\n", tLat/1e7, tLon/1e7, tAlt);
            } else {
                Serial.println("Waiting for Leader Data...");
            }
        } else {
            Serial.println("Safety Check FAILED (Dist > 200m or Leader lost)");
        }
    }
    Serial.println("---------------------------\n");
}

void setup() {
    Serial.begin(115200);
    
    // 初始化配置
    storage.begin();
    storage.loadConfig(config);
    
    // 初始化引脚 (ESP32-C3)
    pinMode(SWITCH1, INPUT_PULLUP);
    pinMode(BOARD_BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    
    // 初始化各模块
    // 注意：ESP32 的 Serial1 需要显式调用 begin 并传入 RX/TX 引脚
    // 在这里重新初始化一遍以防万一
    Serial1.begin(BAUD_RATE, SERIAL_8N1, RXPIN, TXPIN);
    mavlink.begin(BAUD_RATE, RXPIN, TXPIN);
    network.begin();
    formation.begin();
    
    Serial.println("Formation System Started!");
    Serial.printf("Mode: %s, SSID: %s\n", config.AP ? "Leader" : "Follower", config.chSsid);
    
    // 请求数据流 (只在刚开机时请求一次可能不够，有些飞控会忽略)
    delay(1000);
    mavlink.requestDataStreams();
}

void loop() {
    // 1. 处理按键逻辑 (长按 3 秒重置)
    static unsigned long pressStart = 0;
    if (isResetButtonPressed()) {
        if (pressStart == 0) pressStart = millis();
        if (millis() - pressStart > 3000) {
            Serial.println("Resetting Config by board button...");
            storage.resetConfig(config);
            ESP.restart();
        }
    } else {
        // 松开按键重置计时器
        pressStart = 0;
    }

    // 2. 运行模块任务
    mavlink.handle();
    network.handle();
    formation.update();

    // 3. 维护 MAVLink 心跳包 (1Hz) 并定期重新请求数据流
    if (millis() - lastMavHeartbeat > 1000) {
        mavlink.sendHeartbeat();
        
        // 每 5 秒重新向飞控请求一次数据流，防止飞控意外停止发送数据
        static int reqCounter = 0;
        if (reqCounter++ >= 5) {
            mavlink.requestDataStreams();
            reqCounter = 0;
        }
        
        lastMavHeartbeat = millis();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // 闪烁指示
    }

    // 4. 编队控制逻辑 (5Hz)
    if (millis() - lastFormationUpdate > 200) {
        lastFormationUpdate = millis();
        
        if (!config.AP) {
            static bool latchedTakeoff = false;
            static bool latchedJoin = false;
            static bool planeJoined = false;
            static bool planeHoldActionSent = false;
            static bool planeRTLActionSent = false;
            static bool copterAutoControlActive = false;
            static bool copterHoldActionSent = false;
            static bool copterRTLActionSent = false;
            static bool rcSyncOverrideActive = false;
            static uint16_t lastRcSyncValue = 0;
            static unsigned long lastRcSyncCmdTime = 0;
            static bool latchedArmTest = false;
            static bool latchedDisarmTest = false;
            static bool latchedLand = false;
            static bool leaderDisarmLandLatched = false;
            static unsigned long lastArmTestCmdTime = 0;
            static unsigned long armTestCooldownUntil = 0;
            static unsigned long latchedTakeoffUntil = 0;
            static uint8_t lastSeenCmdType = CMD_NONE;
            static uint16_t lastSeenCmdSeq = 0;
            static bool leaderArmStateKnown = false;
            static bool lastLeaderArmedState = false;
            static uint32_t lastLeaderBootSessionId = 0;
            static uint8_t lastLeaderDisarmEventSeq = 0;
            static unsigned long copterAutoJoinReadySince = 0;
            static bool copterFollowTargetValid = false;
            static int32_t lastCopterTargetLat = 0;
            static int32_t lastCopterTargetLon = 0;
            static float lastCopterTargetAlt = 0.0f;
            static unsigned long copterJoinApproachSince = 0;

            auto clearMissionLatches = [&]() {
                latchedTakeoff = false;
                latchedTakeoffUntil = 0;
                latchedJoin = false;
                latchedLand = false;
            };

            auto setCmdAck = [&](uint8_t cmdType, uint16_t cmdSeq, uint8_t ackStatus) {
                status.lastCmdAckType = cmdType;
                status.lastCmdAckSeq = cmdSeq;
                status.lastCmdAckStatus = ackStatus;
            };

            auto fcAckAccepted = [&](uint16_t mavCmd, unsigned long sentAt) {
                if (sentAt == 0) return false;
                if (status.lastFcAckTime < sentAt) return false;
                if (status.lastFcAckCommand != mavCmd) return false;
                return status.lastFcAckResult == MAV_RESULT_ACCEPTED ||
                       status.lastFcAckResult == MAV_RESULT_IN_PROGRESS;
            };

            // 锁存来自长机的实时指令，使用命令序号避免 UDP 单包丢失或重复触发造成的不确定性。
            if (formation.leaderData.activeCmdType != CMD_NONE &&
                formation.leaderData.activeCmdSeq != 0 &&
                (formation.leaderData.activeCmdType != lastSeenCmdType ||
                 formation.leaderData.activeCmdSeq != lastSeenCmdSeq)) {
                lastSeenCmdType = formation.leaderData.activeCmdType;
                lastSeenCmdSeq = formation.leaderData.activeCmdSeq;
                setCmdAck(lastSeenCmdType, lastSeenCmdSeq, CMD_ACK_RECEIVED);

                if (lastSeenCmdType == CMD_ARM_TEST) {
                    clearMissionLatches();
                    latchedArmTest = true;
                    latchedDisarmTest = false;
                    armTestCooldownUntil = millis() + 10000;
                } else if (lastSeenCmdType == CMD_DISARM_TEST) {
                    clearMissionLatches();
                    latchedDisarmTest = true;
                    latchedArmTest = false;
                    armTestCooldownUntil = millis() + 10000;
                } else if (lastSeenCmdType == CMD_LAND_ALL) {
                    latchedTakeoff = false;
                    latchedTakeoffUntil = 0;
                    latchedJoin = false;
                    latchedLand = true;
                } else if (lastSeenCmdType == CMD_TAKEOFF_ALL) {
                    latchedJoin = false;
                    latchedLand = false;
                    latchedTakeoff = true;
                    latchedTakeoffUntil = millis() + 15000;
                } else if (lastSeenCmdType == CMD_JOIN_ALL) {
                    latchedTakeoff = false;
                    latchedTakeoffUntil = 0;
                    latchedLand = false;
                    latchedJoin = true;
                }
            }

            if (latchedTakeoff && latchedTakeoffUntil != 0 && millis() > latchedTakeoffUntil) {
                latchedTakeoff = false;
                latchedTakeoffUntil = 0;
            }

            // 到达系统最低安全高度后，再清除一键起飞锁存，避免 2m/1.5m/3m 三套阈值冲突。
            if (status.pos.relative_alt >= LOWEST_ALTITUDE) {
                latchedTakeoff = false;
                latchedTakeoffUntil = 0;
            }
            // 如果高度低于 0.5 米，清除加入编队指令锁存 (认为已落地或未起飞)
            if (status.pos.relative_alt < 500) {
                latchedJoin = false;
                latchedLand = false;
                planeJoined = false;
                planeHoldActionSent = false;
                planeRTLActionSent = false;
                copterAutoControlActive = false;
                copterHoldActionSent = false;
                copterRTLActionSent = false;
                copterAutoJoinReadySince = 0;
                copterFollowTargetValid = false;
                lastCopterTargetAlt = 0.0f;
                copterJoinApproachSince = 0;
            }

            bool isArmed = status.heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
            bool isGuided = mavlink.isGuidedMode();
            unsigned long leaderAge = formation.leaderLinkAgeMs();
            bool leaderHoldLost = leaderAge > HOLD_LOST_WAIT_TIME;
            bool leaderHardLost = leaderAge > MAX_LOST_WAIT_TIME;
            bool onGroundForArmTest = status.pos.relative_alt < 800;
            bool armTestCooldownActive = millis() < armTestCooldownUntil;
            bool leaderArmed = formation.leaderData.isArmed == 1;

            if (leaderAge <= MAX_LOST_WAIT_TIME) {
                if (lastLeaderBootSessionId != formation.leaderData.bootSessionId) {
                    lastLeaderBootSessionId = formation.leaderData.bootSessionId;
                    leaderArmStateKnown = false;
                    lastLeaderArmedState = leaderArmed;
                    leaderDisarmLandLatched = false;
                    lastLeaderDisarmEventSeq = formation.leaderData.reservedCmdFlags[0];
                }
                if (formation.leaderData.reservedCmdFlags[0] != 0 &&
                    formation.leaderData.reservedCmdFlags[0] != lastLeaderDisarmEventSeq) {
                    leaderDisarmLandLatched = true;
                    lastLeaderDisarmEventSeq = formation.leaderData.reservedCmdFlags[0];
                    Serial.println("Leader disarm event packet detected! Forcing follower landing/recovery immediately...");
                }
                if (leaderArmStateKnown && lastLeaderArmedState && !leaderArmed) {
                    leaderDisarmLandLatched = true;
                    Serial.println("Leader disarmed detected! Forcing follower landing/recovery immediately...");
                }
                lastLeaderArmedState = leaderArmed;
                leaderArmStateKnown = true;
            }

            if (!isArmed || status.pos.relative_alt < 500) {
                leaderDisarmLandLatched = false;
            }

            bool leaderDisarmForcedLand = isArmed &&
                                          leaderAge <= MAX_LOST_WAIT_TIME &&
                                          leaderDisarmLandLatched;
            bool autoLandRequested = leaderDisarmForcedLand ||
                                     (!leaderHoldLost && isArmed && formation.shouldAutoLand());
            float autoTakeoffAlt = 0.0f;
            bool autoArmForTakeoffRequested = formation.shouldAutoArmForTakeoff();
            bool autoTakeoffRequested = formation.shouldAutoTakeoff(autoTakeoffAlt);
            bool manualTakeoffRequested = latchedTakeoff;
            bool rcSyncEnabled = config.rcSyncEnabled &&
                                 formation.leaderData.rcSyncEnabled &&
                                 config.rcSyncTargetChannel >= 1 &&
                                 config.rcSyncTargetChannel <= 18;
            bool rcSyncValueValid = formation.leaderData.rcSyncValue >= 800 &&
                                    formation.leaderData.rcSyncValue <= 2200;
            static uint8_t lastLoggedTakeoffReason = ACTION_REASON_NA;
            static uint8_t lastLoggedLandReason = ACTION_REASON_NA;
            static uint8_t lastLoggedJoinReason = ACTION_REASON_NA;
            static unsigned long lastReasonLogTime = 0;

            status.takeoffReason = ACTION_REASON_NA;
            status.landReason = ACTION_REASON_NA;
            status.joinReason = ACTION_REASON_NA;

            if (mavlink.isPlane()) {
                bool planeRecoveryRequested = planeJoined && leaderDisarmForcedLand;
                bool planeLandRequested = latchedLand || planeRecoveryRequested;
                status.takeoffReason = ACTION_REASON_MANUAL_PLANE_ONLY;

                if (planeRecoveryRequested) {
                    status.landReason = ACTION_REASON_READY;
                } else if (!planeLandRequested) {
                    status.landReason = ACTION_REASON_MANUAL_PLANE_ONLY;
                } else if (!isArmed) {
                    status.landReason = ACTION_REASON_NOT_ARMED;
                } else if (leaderHoldLost) {
                    status.landReason = ACTION_REASON_LINK_UNSTABLE;
                } else {
                    status.landReason = ACTION_REASON_READY;
                }

                if (!latchedJoin) {
                    status.joinReason = ACTION_REASON_NOT_REQUESTED;
                } else if (formation.leaderLinkAgeMs() > MAX_LOST_WAIT_TIME) {
                    status.joinReason = ACTION_REASON_WAIT_LEADER;
                } else if (leaderHoldLost) {
                    status.joinReason = ACTION_REASON_LINK_UNSTABLE;
                } else if (armTestCooldownActive) {
                    status.joinReason = ACTION_REASON_ARM_TEST_COOLDOWN;
                } else if (status.pos.relative_alt < LOWEST_ALTITUDE) {
                    status.joinReason = ACTION_REASON_ALT_TOO_LOW;
                } else if (!isGuided) {
                    status.joinReason = ACTION_REASON_NOT_GUIDED;
                } else if (!formation.isSafetyOk()) {
                    status.joinReason = ACTION_REASON_SAFETY_BLOCKED;
                } else {
                    status.joinReason = ACTION_REASON_READY;
                }
            } else {
                bool autoJoinRequested = formation.leaderData.autoTakeoff &&
                                         (autoArmForTakeoffRequested || autoTakeoffRequested || copterAutoControlActive);
                bool autoJoinDelayEligible = autoJoinRequested &&
                                             !latchedLand &&
                                             !leaderDisarmForcedLand &&
                                             !armTestCooldownActive &&
                                             isArmed &&
                                             status.pos.relative_alt >= LOWEST_ALTITUDE &&
                                             !leaderHoldLost &&
                                             formation.isSafetyOk();
                if (autoJoinDelayEligible) {
                    if (copterAutoJoinReadySince == 0) {
                        copterAutoJoinReadySince = millis();
                        Serial.println("Copter reached takeoff height, holding briefly before auto-joining formation...");
                    }
                } else {
                    copterAutoJoinReadySince = 0;
                }
                bool autoJoinHoverReady = autoJoinDelayEligible &&
                                          copterAutoJoinReadySince != 0 &&
                                          millis() - copterAutoJoinReadySince >= COPTER_AUTO_JOIN_HOVER_MS;
                bool joinRequested = latchedJoin || autoJoinRequested;

                if (status.pos.relative_alt >= LOWEST_ALTITUDE) {
                    status.takeoffReason = ACTION_REASON_ALREADY_AIRBORNE;
                } else if (formation.leaderLinkAgeMs() > MAX_LOST_WAIT_TIME) {
                    status.takeoffReason = ACTION_REASON_WAIT_LEADER;
                } else if (leaderHoldLost) {
                    status.takeoffReason = ACTION_REASON_LINK_UNSTABLE;
                } else if (armTestCooldownActive) {
                    status.takeoffReason = ACTION_REASON_ARM_TEST_COOLDOWN;
                } else if (manualTakeoffRequested) {
                    status.takeoffReason = isGuided ? ACTION_REASON_READY : ACTION_REASON_NOT_GUIDED;
                } else if (!formation.leaderData.autoTakeoff) {
                    status.takeoffReason = ACTION_REASON_LEADER_NOT_READY;
                } else if (!autoTakeoffRequested) {
                    status.takeoffReason = ACTION_REASON_LEADER_NOT_READY;
                } else {
                    status.takeoffReason = isGuided ? ACTION_REASON_READY : ACTION_REASON_NOT_GUIDED;
                }

                if (leaderDisarmForcedLand) {
                    status.landReason = isGuided ? ACTION_REASON_READY : ACTION_REASON_NOT_GUIDED;
                } else if (latchedLand) {
                    if (!isArmed) {
                        status.landReason = ACTION_REASON_NOT_ARMED;
                    } else if (leaderHoldLost) {
                        status.landReason = ACTION_REASON_LINK_UNSTABLE;
                    } else {
                        status.landReason = isGuided ? ACTION_REASON_READY : ACTION_REASON_NOT_GUIDED;
                    }
                } else if (formation.leaderLinkAgeMs() > MAX_LOST_WAIT_TIME) {
                    status.landReason = ACTION_REASON_WAIT_LEADER;
                } else if (leaderHoldLost) {
                    status.landReason = ACTION_REASON_LINK_UNSTABLE;
                } else if (!isArmed) {
                    status.landReason = ACTION_REASON_NOT_ARMED;
                } else if (!formation.leaderData.autoLand || !formation.shouldAutoLand()) {
                    status.landReason = ACTION_REASON_LEADER_NOT_READY;
                } else {
                    status.landReason = isGuided ? ACTION_REASON_READY : ACTION_REASON_NOT_GUIDED;
                }

                if (!joinRequested) {
                    status.joinReason = ACTION_REASON_NOT_REQUESTED;
                } else if (formation.leaderLinkAgeMs() > MAX_LOST_WAIT_TIME) {
                    status.joinReason = ACTION_REASON_WAIT_LEADER;
                } else if (leaderHoldLost) {
                    status.joinReason = ACTION_REASON_LINK_UNSTABLE;
                } else if (armTestCooldownActive) {
                    status.joinReason = ACTION_REASON_ARM_TEST_COOLDOWN;
                } else if (status.pos.relative_alt < LOWEST_ALTITUDE) {
                    status.joinReason = ACTION_REASON_ALT_TOO_LOW;
                } else if (!isArmed) {
                    status.joinReason = ACTION_REASON_NOT_ARMED;
                } else if (!formation.isSafetyOk()) {
                    status.joinReason = ACTION_REASON_SAFETY_BLOCKED;
                } else {
                    status.joinReason = isGuided ? ACTION_REASON_READY : ACTION_REASON_NOT_GUIDED;
                }
            }

            auto logReasonChange = [&](const char *actionName, uint8_t currentReason, uint8_t &lastReason) {
                if (currentReason == lastReason) return;
                if (millis() - lastReasonLogTime < 300) return;
                Serial.printf("%s state -> %s\n", actionName, actionReasonCodeName(currentReason));
                lastReason = currentReason;
                lastReasonLogTime = millis();
            };
            logReasonChange("Takeoff", status.takeoffReason, lastLoggedTakeoffReason);
            logReasonChange("Land", status.landReason, lastLoggedLandReason);
            logReasonChange("Join", status.joinReason, lastLoggedJoinReason);

            // 长机一旦上锁，立即停止向僚机继续同步 RC 覆盖，避免把危险开关状态带到空中僚机。
            if (rcSyncEnabled && !leaderHoldLost && leaderArmed && rcSyncValueValid) {
                if (!rcSyncOverrideActive ||
                    formation.leaderData.rcSyncValue != lastRcSyncValue ||
                    millis() - lastRcSyncCmdTime > 400) {
                    mavlink.sendRcChannelOverride(config.rcSyncTargetChannel, formation.leaderData.rcSyncValue);
                    lastRcSyncValue = formation.leaderData.rcSyncValue;
                    lastRcSyncCmdTime = millis();
                    rcSyncOverrideActive = true;
                }
            } else if (rcSyncOverrideActive && millis() - lastRcSyncCmdTime > 400) {
                mavlink.sendRcChannelRelease(config.rcSyncTargetChannel);
                lastRcSyncCmdTime = millis();
                rcSyncOverrideActive = false;
                lastRcSyncValue = 0;
            }

            if (latchedArmTest) {
                if (isArmed) {
                    setCmdAck(CMD_ARM_TEST, lastSeenCmdSeq, CMD_ACK_COMPLETED);
                    latchedArmTest = false;
                    Serial.println("Arm test completed: follower is ARMED.");
                } else if (fcAckAccepted(MAV_CMD_COMPONENT_ARM_DISARM, lastArmTestCmdTime)) {
                    setCmdAck(CMD_ARM_TEST, lastSeenCmdSeq, CMD_ACK_FC_ACCEPTED);
                } else if (!leaderHoldLost && onGroundForArmTest && millis() - lastArmTestCmdTime > 2000) {
                    mavlink.sendArmCommand();
                    setCmdAck(CMD_ARM_TEST, lastSeenCmdSeq, CMD_ACK_SENT_TO_FC);
                    lastArmTestCmdTime = millis();
                    Serial.println("Arm test: sending ARM command to follower...");
                }
            }

            if (latchedDisarmTest) {
                if (!isArmed) {
                    setCmdAck(CMD_DISARM_TEST, lastSeenCmdSeq, CMD_ACK_COMPLETED);
                    latchedDisarmTest = false;
                    Serial.println("Disarm test completed: follower is DISARMED.");
                } else if (fcAckAccepted(MAV_CMD_COMPONENT_ARM_DISARM, lastArmTestCmdTime)) {
                    setCmdAck(CMD_DISARM_TEST, lastSeenCmdSeq, CMD_ACK_FC_ACCEPTED);
                } else if (onGroundForArmTest && millis() - lastArmTestCmdTime > 2000) {
                    mavlink.sendDisarmCommand();
                    setCmdAck(CMD_DISARM_TEST, lastSeenCmdSeq, CMD_ACK_SENT_TO_FC);
                    lastArmTestCmdTime = millis();
                    Serial.println("Disarm test: sending DISARM command to follower...");
                }
            }

            if (mavlink.isPlane()) {
                // 固定翼僚机不再支持跟随长机自动起飞/自动降落。
                // 仅允许飞手手动起飞后，再手动切换到 GUIDED 模式加入编队。
                bool triggerJoin = latchedJoin;
                bool planeRecoveryRequested = planeJoined && leaderDisarmForcedLand;
                bool planeLandRequested = latchedLand || planeRecoveryRequested;

                if (!leaderHoldLost) {
                    planeHoldActionSent = false;
                    planeRTLActionSent = false;
                }

                if (planeLandRequested && isArmed) {
                    static unsigned long lastPlaneLandCmdTime = 0;
                    if (status.heartbeat.custom_mode == mavlink.rtlModeNumber()) {
                        if (latchedLand) setCmdAck(CMD_LAND_ALL, lastSeenCmdSeq, CMD_ACK_COMPLETED);
                    } else if (fcAckAccepted(MAV_CMD_NAV_RETURN_TO_LAUNCH, lastPlaneLandCmdTime)) {
                        if (latchedLand) setCmdAck(CMD_LAND_ALL, lastSeenCmdSeq, CMD_ACK_FC_ACCEPTED);
                    } else if (millis() - lastPlaneLandCmdTime > 1500) {
                        mavlink.sendRecoveryMode();
                        if (latchedLand) setCmdAck(CMD_LAND_ALL, lastSeenCmdSeq, CMD_ACK_SENT_TO_FC);
                        lastPlaneLandCmdTime = millis();
                        planeJoined = false;
                        Serial.println(planeRecoveryRequested ?
                                       "Leader disarmed! Plane follower switching to immediate RTL recovery..." :
                                       "Recovery command received! Plane follower switching to RTL recovery...");
                    }
                } else if (leaderHardLost) {
                    if (planeJoined && isArmed && !planeRTLActionSent) {
                        mavlink.sendRTLMode();
                        planeRTLActionSent = true;
                        planeJoined = false;
                        Serial.println("Leader lost too long! Plane follower switching to RTL...");
                    }
                } else if (leaderHoldLost) {
                    if (planeJoined && isArmed && !planeHoldActionSent) {
                        mavlink.sendLoiterMode();
                        planeHoldActionSent = true;
                        planeJoined = false;
                        Serial.println("Leader link unstable! Plane follower switching to LOITER...");
                    }
                } else if (!armTestCooldownActive && status.pos.relative_alt >= LOWEST_ALTITUDE && triggerJoin && formation.isSafetyOk()) {
                    if (isGuided) {
                        int32_t tLat, tLon;
                        float tAlt;
                        if (formation.getTargetPoint(tLat, tLon, tAlt)) {
                            mavlink.sendFollowTarget(tLat, tLon, tAlt);
                            setCmdAck(CMD_JOIN_ALL, lastSeenCmdSeq, CMD_ACK_SENT_TO_FC);
                            planeJoined = true;
                        }
                    } else {
                        planeJoined = false;
                    }
                } else {
                    planeJoined = false;
                }
            } else {
                bool autoJoinRequested = formation.leaderData.autoTakeoff &&
                                         (autoArmForTakeoffRequested || autoTakeoffRequested || copterAutoControlActive);
                bool autoJoinDelayEligible = autoJoinRequested &&
                                             !latchedLand &&
                                             !leaderDisarmForcedLand &&
                                             !armTestCooldownActive &&
                                             isArmed &&
                                             status.pos.relative_alt >= LOWEST_ALTITUDE &&
                                             !leaderHoldLost &&
                                             formation.isSafetyOk();
                if (autoJoinDelayEligible) {
                    if (copterAutoJoinReadySince == 0) {
                        copterAutoJoinReadySince = millis();
                    }
                } else {
                    copterAutoJoinReadySince = 0;
                }
                bool autoJoinHoverReady = autoJoinDelayEligible &&
                                          copterAutoJoinReadySince != 0 &&
                                          millis() - copterAutoJoinReadySince >= COPTER_AUTO_JOIN_HOVER_MS;
                bool triggerJoin = latchedJoin || autoJoinHoverReady;
                bool followCommandDesired = triggerJoin && !latchedLand && !leaderDisarmForcedLand;

                if (!followCommandDesired || !isArmed || leaderHoldLost) {
                    copterFollowTargetValid = false;
                    lastCopterTargetAlt = 0.0f;
                    copterJoinApproachSince = 0;
                }

                if (!leaderHoldLost) {
                    copterHoldActionSent = false;
                    copterRTLActionSent = false;
                }

                if (leaderDisarmForcedLand && copterAutoControlActive) {
                    copterHoldActionSent = false;
                    copterRTLActionSent = false;
                } else if (leaderHardLost) {
                    if (!copterAutoControlActive) {
                        copterHoldActionSent = false;
                        copterRTLActionSent = false;
                    } else if (isArmed && status.pos.relative_alt >= LOWEST_ALTITUDE && !copterRTLActionSent) {
                        mavlink.sendRTLMode();
                        copterRTLActionSent = true;
                        Serial.println("Leader lost too long! Copter follower switching to RTL...");
                    }
                } else if (leaderHoldLost) {
                    if (!copterAutoControlActive) {
                        copterHoldActionSent = false;
                    } else if (isArmed && status.pos.relative_alt >= LOWEST_ALTITUDE && !copterHoldActionSent) {
                        mavlink.sendLoiterMode();
                        copterHoldActionSent = true;
                        Serial.println("Leader link unstable! Copter follower switching to LOITER...");
                    }
                }

                // 起飞、加入编队、手动降落或自动降落前，都允许把多旋翼拉回 GUIDED，
                // 避免僚机因短暂失联切进 LOITER 后无法恢复执行后续动作。
                if (!leaderDisarmForcedLand &&
                    !leaderHoldLost &&
                    !armTestCooldownActive &&
                    ((manualTakeoffRequested || autoArmForTakeoffRequested || autoTakeoffRequested) ||
                     (status.pos.relative_alt >= LOWEST_ALTITUDE && (triggerJoin || autoJoinRequested)) ||
                     autoLandRequested ||
                     (isArmed && latchedLand)) &&
                    !isGuided) {
                    static unsigned long lastModeCmdTime = 0;
                    if (millis() - lastModeCmdTime > 2000) {
                        mavlink.sendGuidedMode();
                        lastModeCmdTime = millis();
                        Serial.println("Switching to GUIDED mode for copter mission recovery...");
                    }
                }

                if (leaderDisarmForcedLand && isArmed) {
                    static unsigned long lastForcedLandCmdTime = 0;
                    if (status.heartbeat.custom_mode != mavlink.recoveryModeNumber() &&
                        millis() - lastForcedLandCmdTime > 500) {
                        copterAutoControlActive = true;
                        mavlink.sendRecoveryMode();
                        lastForcedLandCmdTime = millis();
                        Serial.println("Leader disarmed! Copter follower forcing LAND immediately...");
                    }
                } else if (isGuided && !leaderHoldLost) {
                    if (isArmed && latchedLand) {
                        static unsigned long lastManualLandCmdTime = 0;
                        if (status.heartbeat.custom_mode == mavlink.recoveryModeNumber()) {
                            setCmdAck(CMD_LAND_ALL, lastSeenCmdSeq, CMD_ACK_COMPLETED);
                        } else if (fcAckAccepted(MAV_CMD_DO_SET_MODE, lastManualLandCmdTime)) {
                            setCmdAck(CMD_LAND_ALL, lastSeenCmdSeq, CMD_ACK_FC_ACCEPTED);
                        } else if (millis() - lastManualLandCmdTime > 2000) {
                            copterAutoControlActive = true;
                            mavlink.sendRecoveryMode();
                            setCmdAck(CMD_LAND_ALL, lastSeenCmdSeq, CMD_ACK_SENT_TO_FC);
                            lastManualLandCmdTime = millis();
                            Serial.println("Recovery command received! Copter follower entering LAND...");
                        }
                    } else if (autoLandRequested) {
                        // 满足自动恢复条件：多旋翼切 LAND，固定翼切 RTL。
                        static unsigned long lastLandCmdTime = 0;
                        if (status.heartbeat.custom_mode == mavlink.recoveryModeNumber()) {
                            if (latchedLand) {
                                setCmdAck(CMD_LAND_ALL, lastSeenCmdSeq, CMD_ACK_COMPLETED);
                            }
                        } else if (fcAckAccepted(MAV_CMD_DO_SET_MODE, lastLandCmdTime)) {
                            if (latchedLand) {
                                setCmdAck(CMD_LAND_ALL, lastSeenCmdSeq, CMD_ACK_FC_ACCEPTED);
                            }
                        } else if (millis() - lastLandCmdTime > 2000) { // 防拥堵
                            copterAutoControlActive = true;
                            mavlink.sendRecoveryMode();
                            if (latchedLand) {
                                setCmdAck(CMD_LAND_ALL, lastSeenCmdSeq, CMD_ACK_SENT_TO_FC);
                            }
                            lastLandCmdTime = millis();
                            Serial.println("Leader disarmed! Auto recovery triggered...");
                        }
                    } else if (!latchedLand && status.pos.relative_alt < LOWEST_ALTITUDE) {
                        // 僚机高度不足，检查是否满足起飞条件 (自动或锁存的一键指令)
                        static unsigned long lastCmdTime = 0;
                        if (latchedTakeoff && isArmed && status.pos.relative_alt >= LOWEST_ALTITUDE) {
                            setCmdAck(CMD_TAKEOFF_ALL, lastSeenCmdSeq, CMD_ACK_COMPLETED);
                        } else if (!armTestCooldownActive && millis() - lastCmdTime > 2000) { 
                            float tAlt = autoTakeoffAlt;
                            bool triggerArmOnly = autoArmForTakeoffRequested && !autoTakeoffRequested;
                            bool triggerTakeoff = manualTakeoffRequested || autoTakeoffRequested;
                            
                            if (triggerArmOnly || triggerTakeoff) {
                                copterAutoControlActive = true;
                                if (!isArmed) {
                                    if (latchedTakeoff && fcAckAccepted(MAV_CMD_COMPONENT_ARM_DISARM, lastCmdTime)) {
                                        setCmdAck(CMD_TAKEOFF_ALL, lastSeenCmdSeq, CMD_ACK_FC_ACCEPTED);
                                    } else {
                                        mavlink.sendArmCommand();
                                        if (latchedTakeoff) setCmdAck(CMD_TAKEOFF_ALL, lastSeenCmdSeq, CMD_ACK_SENT_TO_FC);
                                        lastCmdTime = millis();
                                    }
                                } else if (triggerTakeoff) {
                                    if (latchedTakeoff && fcAckAccepted(MAV_CMD_NAV_TAKEOFF, lastCmdTime)) {
                                        setCmdAck(CMD_TAKEOFF_ALL, lastSeenCmdSeq, CMD_ACK_FC_ACCEPTED);
                                    } else {
                                        // 锁存的一键起飞也统一使用系统最低安全高度，避免低于编队门槛。
                                        float finalTakeoffAlt = latchedTakeoff ? (LOWEST_ALTITUDE / 1000.0f) : tAlt;
                                        mavlink.sendTakeoff(finalTakeoffAlt);
                                        if (latchedTakeoff) setCmdAck(CMD_TAKEOFF_ALL, lastSeenCmdSeq, CMD_ACK_SENT_TO_FC);
                                        lastCmdTime = millis();
                                    }
                                }
                            }
                        }
                    } else if (!latchedLand && isArmed) {
                        // 僚机已经在空中，检查是否加入编队
                        if (triggerJoin && formation.isSafetyOk()) {
                            int32_t tLat, tLon;
                            float tAlt;
                            if (formation.getTargetPoint(tLat, tLon, tAlt)) {
                                float currentAltM = status.pos.relative_alt / 1000.0f;
                                float targetAltFloor = currentAltM - COPTER_FOLLOW_ALT_FLOOR_MARGIN_M;
                                if (targetAltFloor < (LOWEST_ALTITUDE / 1000.0f)) {
                                    targetAltFloor = LOWEST_ALTITUDE / 1000.0f;
                                }
                                if (tAlt < targetAltFloor) {
                                    tAlt = targetAltFloor;
                                }

                                if (!copterFollowTargetValid) {
                                    lastCopterTargetLat = status.pos.lat;
                                    lastCopterTargetLon = status.pos.lon;
                                    lastCopterTargetAlt = currentAltM;
                                    copterFollowTargetValid = true;
                                    copterJoinApproachSince = millis();
                                    Serial.println("Copter entering smoothed formation approach...");
                                }

                                unsigned long joinElapsed = (copterJoinApproachSince == 0) ? 0 : (millis() - copterJoinApproachSince);
                                float joinBlend = clampFloatValue((float)joinElapsed / COPTER_JOIN_APPROACH_MS, 0.0f, 1.0f);
                                float horizontalStepLimit = COPTER_FOLLOW_MAX_STEP_M * (0.35f + 0.65f * joinBlend);
                                float targetMoveDist = distanceMetersE7(lastCopterTargetLat, lastCopterTargetLon, tLat, tLon);
                                if (targetMoveDist > horizontalStepLimit) {
                                    float moveBearing = bearingDegE7(lastCopterTargetLat, lastCopterTargetLon, tLat, tLon);
                                    offsetGpsMetersE7(lastCopterTargetLat, lastCopterTargetLon, horizontalStepLimit, moveBearing, tLat, tLon);
                                }

                                float maxAltUp = COPTER_FOLLOW_MAX_ASCENT_STEP_M * (0.5f + 0.5f * joinBlend);
                                float maxAltDown = COPTER_FOLLOW_MAX_DESCENT_STEP_M;
                                float altDelta = tAlt - lastCopterTargetAlt;
                                altDelta = clampFloatValue(altDelta, -maxAltDown, maxAltUp);
                                tAlt = lastCopterTargetAlt + altDelta;
                                if (tAlt < targetAltFloor) {
                                    tAlt = targetAltFloor;
                                }

                                copterAutoControlActive = true;
                                mavlink.sendFollowTarget(tLat, tLon, tAlt);
                                lastCopterTargetLat = tLat;
                                lastCopterTargetLon = tLon;
                                lastCopterTargetAlt = tAlt;
                                if (latchedJoin) setCmdAck(CMD_JOIN_ALL, lastSeenCmdSeq, CMD_ACK_SENT_TO_FC);
                            }
                        }
                    }
                }
            }
        }
    }

    // 5. 定期输出调试日志 (每 2 秒)
    if (millis() - lastLogTime > 2000) {
        printDebugLogs();
        lastLogTime = millis();
    }
}
