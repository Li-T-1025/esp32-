#include "Formation.h"
#include <math.h>
#include <string.h>
#include <esp_system.h>
#include <esp_wifi.h>

static uint32_t fnv1aUpdate(uint32_t hash, const uint8_t *data, size_t len) {
    const uint32_t kPrime = 16777619u;
    for (size_t i = 0; i < len; ++i) {
        hash ^= data[i];
        hash *= kPrime;
    }
    return hash;
}

static uint32_t computePacketAuthTag(const FormationPacket &packet, const char *sharedSecret) {
    FormationPacket temp = packet;
    temp.authTag = 0;

    uint32_t hash = 2166136261u;
    hash = fnv1aUpdate(hash, reinterpret_cast<const uint8_t*>(&temp), sizeof(temp));
    hash = fnv1aUpdate(hash,
                       reinterpret_cast<const uint8_t*>(sharedSecret),
                       strlen(sharedSecret));
    return hash;
}

static bool isNewerPacketCounter(uint32_t current, uint32_t previous) {
    if (previous == 0) return true;
    return current > previous;
}

static uint16_t getRcChannelRaw(const mavlink_rc_channels_t &rc, uint8_t channel) {
    switch (channel) {
        case 1: return rc.chan1_raw;
        case 2: return rc.chan2_raw;
        case 3: return rc.chan3_raw;
        case 4: return rc.chan4_raw;
        case 5: return rc.chan5_raw;
        case 6: return rc.chan6_raw;
        case 7: return rc.chan7_raw;
        case 8: return rc.chan8_raw;
        case 9: return rc.chan9_raw;
        case 10: return rc.chan10_raw;
        case 11: return rc.chan11_raw;
        case 12: return rc.chan12_raw;
        case 13: return rc.chan13_raw;
        case 14: return rc.chan14_raw;
        case 15: return rc.chan15_raw;
        case 16: return rc.chan16_raw;
        case 17: return rc.chan17_raw;
        case 18: return rc.chan18_raw;
        default: return 0;
    }
}

static bool getExpectedLeaderBssid(uint8_t outBssid[6]) {
    wifi_ap_record_t apInfo;
    if (esp_wifi_sta_get_ap_info(&apInfo) != ESP_OK) return false;
    memcpy(outBssid, apInfo.bssid, 6);
    return true;
}

Formation::Formation(MyConfig &config, MyStatus &status, MyLine *lines) 
    : _config(config), _status(status), _lines(lines), lastLeaderPacketTime(0), leaderTakeoffCandidateSince(0),
      leaderLandingCandidateSince(0), lastLocalPacketCounter(0), lastLeaderPacketCounter(0),
      localBootSessionId(0), lastLeaderBootSessionId(0), lastLocalArmedKnown(false),
      lastLocalArmedState(false), localDisarmEventSeq(0) {}

void Formation::begin() {
    localBootSessionId = esp_random();
    if (localBootSessionId == 0) {
        localBootSessionId = ((uint32_t)millis() << 16) ^ (uint32_t)micros();
        if (localBootSessionId == 0) localBootSessionId = 1;
    }
    udp.begin(FORMATION_PORT);
}

void Formation::update() {
    broadcastOwnStatus();
    receivePackets();
}

void Formation::broadcastOwnStatus() {
    static unsigned long lastBroadcast = 0;
    if (millis() - lastBroadcast < 100) return; // 10Hz
    lastBroadcast = millis();

    FormationPacket packet;
    memset(&packet, 0, sizeof(packet));
    packet.sysid = _config.AP ? 0 : 1;
    packet.vehicleType = _status.vehicleType;
    if (_config.AP) {
        esp_wifi_get_mac(WIFI_IF_AP, packet.mac);
    } else {
        WiFi.macAddress(packet.mac);
    }
    IPAddress localIP = _config.AP ? WiFi.softAPIP() : WiFi.localIP();
    for (int i = 0; i < 4; i++) packet.ip[i] = localIP[i];
    strncpy(packet.nickName, _config.MyNikeName, sizeof(packet.nickName) - 1);
    packet.lat = _status.pos.lat;
    packet.lon = _status.pos.lon;
    // 重要：广播给僚机的高度改为使用 relative_alt (相对起飞点高度)，
    // 避免两架飞机气压计基准不同导致的坠机风险
    packet.alt = _status.pos.relative_alt; 
    packet.heading = _status.hudData.heading;
    packet.groundSpeed = _status.groundSpeed;
    packet.teamType = _config.byTeamType;
    packet.teamDist = (_config.uTeamDist < MIN_TEAM_DISTANCE_CM) ? MIN_TEAM_DISTANCE_CM : _config.uTeamDist;
    packet.teamHigh = _config.uTeamHigh;
    packet.autoTakeoff = _config.autoTakeoff;
    packet.autoLand = _config.autoLand;
    packet.rcSyncEnabled = (_config.AP && _config.rcSyncEnabled) ? 1 : 0;
    packet.rcSyncSourceChannel = _config.rcSyncSourceChannel;
    packet.rcSyncValue = 0;
    if (packet.rcSyncEnabled && _config.rcSyncSourceChannel >= 1 && _config.rcSyncSourceChannel <= 18) {
        packet.rcSyncValue = getRcChannelRaw(_status.rc_channels, _config.rcSyncSourceChannel);
    }
    packet.isArmed = (_status.heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? 1 : 0;
    if (_config.AP) {
        if (lastLocalArmedKnown && lastLocalArmedState && packet.isArmed == 0) {
            localDisarmEventSeq++;
            if (localDisarmEventSeq == 0) localDisarmEventSeq = 1;
        }
        lastLocalArmedState = packet.isArmed != 0;
        lastLocalArmedKnown = true;
        packet.reservedCmdFlags[0] = localDisarmEventSeq;
    }
    bool commandActive = _config.AP &&
                         _status.activeCmdType != CMD_NONE &&
                         millis() <= _status.activeCmdUntil;
    packet.activeCmdType = commandActive ? _status.activeCmdType : CMD_NONE;
    packet.activeCmdSeq = commandActive ? _status.activeCmdSeq : 0;
    packet.ackCmdType = _config.AP ? CMD_NONE : _status.lastCmdAckType;
    packet.ackCmdSeq = _config.AP ? 0 : _status.lastCmdAckSeq;
    packet.ackCmdStatus = _config.AP ? CMD_ACK_NONE : _status.lastCmdAckStatus;
    packet.gpsHealthy = isGpsHealthy() ? 1 : 0;
    packet.ekfHealthy = isEkfHealthy() ? 1 : 0;
    lastLocalPacketCounter++;
    if (lastLocalPacketCounter == 0) lastLocalPacketCounter = 1;
    packet.packetCounter = lastLocalPacketCounter;
    packet.bootSessionId = localBootSessionId;
    _status.lastSentPacketCounter = packet.packetCounter;
    packet.authTag = computePacketAuthTag(packet, _config.chPasswd);

    IPAddress broadcastIP(255, 255, 255, 255);
    if (!_config.AP) {
        // 僚机如果已连接，发送给网关（长机）
        broadcastIP = WiFi.gatewayIP();
    }

    udp.beginPacket(broadcastIP, FORMATION_PORT);
    udp.write((uint8_t*)&packet, sizeof(packet));
    udp.endPacket();
}

void Formation::updateFollowerLine(const FormationPacket &packet) {
    int freeIndex = -1;
    int matchedIndex = -1;
    int staleIndex = -1;

    for (int i = 0; i < FLIGHT_NUM; i++) {
        if (_lines[i].useage) {
            if (memcmp(_lines[i].mac, packet.mac, sizeof(packet.mac)) == 0) {
                matchedIndex = i;
                break;
            }
            if (staleIndex == -1 && (millis() - _lines[i].line_time) > MAX_LOST_WAIT_TIME) {
                staleIndex = i;
            }
        } else if (freeIndex == -1) {
            freeIndex = i;
        }
    }

    int index = (matchedIndex != -1) ? matchedIndex : ((freeIndex != -1) ? freeIndex : staleIndex);
    if (index == -1) return;
    bool sameBootSession = _lines[index].bootSessionId == packet.bootSessionId;
    if (_lines[index].useage && sameBootSession &&
        !isNewerPacketCounter(packet.packetCounter, _lines[index].lastPacketCounter)) return;

    _lines[index].useage = 1;
    memcpy(_lines[index].mac, packet.mac, sizeof(packet.mac));
    memcpy(_lines[index].ip, packet.ip, sizeof(packet.ip));
    _lines[index].vehicleType = packet.vehicleType;
    _lines[index].isArmed = packet.isArmed;
    _lines[index].rel_alt = packet.alt;
    _lines[index].ackCmdType = packet.ackCmdType;
    _lines[index].ackCmdSeq = packet.ackCmdSeq;
    _lines[index].ackCmdStatus = packet.ackCmdStatus;
    _lines[index].lastPacketCounter = packet.packetCounter;
    _lines[index].bootSessionId = packet.bootSessionId;
    strncpy(_lines[index].chNikeName, packet.nickName, sizeof(_lines[index].chNikeName) - 1);
    _lines[index].chNikeName[sizeof(_lines[index].chNikeName) - 1] = '\0';
    _lines[index].line_time = millis();
}

void Formation::receivePackets() {
    int packetSize = udp.parsePacket();
    if (packetSize == sizeof(FormationPacket)) {
        FormationPacket p;
        udp.read((uint8_t*)&p, sizeof(p));

        if (p.authTag != computePacketAuthTag(p, _config.chPasswd)) {
            static unsigned long lastAuthRejectLog = 0;
            _status.lastPacketRejectReason = PACKET_REJECT_BAD_AUTH;
            _status.lastRejectedPacketCounter = p.packetCounter;
            _status.lastPacketRejectTime = millis();
            if (millis() - lastAuthRejectLog > 2000) {
                Serial.printf("Rejected unauthenticated formation packet from %s\n",
                              udp.remoteIP().toString().c_str());
                lastAuthRejectLog = millis();
            }
            return;
        }

        if (!_config.AP && p.sysid == 0) {
            IPAddress expectedLeaderIp = WiFi.gatewayIP();
            uint8_t expectedBssid[6];
            bool haveBssid = getExpectedLeaderBssid(expectedBssid);
            bool ipMatchesGateway = (udp.remoteIP() == expectedLeaderIp);
            bool packetIpMatchesGateway = true;
            for (int i = 0; i < 4; ++i) {
                if (p.ip[i] != expectedLeaderIp[i]) {
                    packetIpMatchesGateway = false;
                    break;
                }
            }
            bool macMatchesBssid = !haveBssid || memcmp(p.mac, expectedBssid, 6) == 0;
            if (!ipMatchesGateway || !packetIpMatchesGateway || !macMatchesBssid) {
                static unsigned long lastSourceRejectLog = 0;
                _status.lastPacketRejectReason = PACKET_REJECT_BAD_SOURCE;
                _status.lastRejectedPacketCounter = p.packetCounter;
                _status.lastPacketRejectTime = millis();
                if (millis() - lastSourceRejectLog > 2000) {
                    Serial.printf("Rejected leader packet from untrusted source %s (expected gateway %s)\n",
                                  udp.remoteIP().toString().c_str(),
                                  expectedLeaderIp.toString().c_str());
                    lastSourceRejectLog = millis();
                }
                return;
            }
        }
        
        if (p.sysid == 0) { // 收到长机消息
            bool sameLeaderBootSession = lastLeaderBootSessionId == p.bootSessionId;
            if (sameLeaderBootSession && !isNewerPacketCounter(p.packetCounter, lastLeaderPacketCounter)) {
                static unsigned long lastReplayRejectLog = 0;
                _status.lastPacketRejectReason = PACKET_REJECT_REPLAY;
                _status.lastRejectedPacketCounter = p.packetCounter;
                _status.lastPacketRejectTime = millis();
                if (millis() - lastReplayRejectLog > 2000) {
                    Serial.printf("Rejected replayed leader packet #%lu from %s\n",
                                  (unsigned long)p.packetCounter,
                                  udp.remoteIP().toString().c_str());
                    lastReplayRejectLog = millis();
                }
                return;
            }
            if (!sameLeaderBootSession) {
                leaderTakeoffCandidateSince = 0;
                leaderLandingCandidateSince = 0;
                lastLeaderBootSessionId = p.bootSessionId;
            }
            lastLeaderPacketCounter = p.packetCounter;
            _status.lastLeaderPacketCounter = p.packetCounter;
            leaderData = p;
            leaderData.activeCmdType = p.activeCmdType;
            leaderData.activeCmdSeq = p.activeCmdSeq;
            lastLeaderPacketTime = millis();
            _status.lastWifiTime = millis();
        } else if (_config.AP) {
            updateFollowerLine(p);
        }
    }
}

bool Formation::shouldAutoTakeoff(float &takeoffAlt) {
    if (_config.AP || lastLeaderPacketTime == 0) return false;
    if (!isGpsHealthy() || !isEkfHealthy()) return false;
    if (leaderData.gpsHealthy == 0 || leaderData.ekfHealthy == 0) return false;
    
    // 领航机未允许自动起飞
    if (leaderData.autoTakeoff == 0) return false;
    
    // 领航机信号不能丢失
    if (millis() - lastLeaderPacketTime > MAX_LOST_WAIT_TIME) return false;

    // 长机必须已经明确解锁，并且离地达到安全高度后一段时间，僚机才允许自动起飞。
    // 这样可以避免长机刚解锁、油门误抖动或气压高度瞬变时，僚机过早起飞。
    if (leaderData.isArmed == 0) {
        leaderTakeoffCandidateSince = 0;
        return false;
    }

    // leaderData.alt 是通过网络广播传过来的相对高度，单位毫米 (mm)
    if (leaderData.alt >= LOWEST_ALTITUDE) {
        if (leaderTakeoffCandidateSince == 0) {
            leaderTakeoffCandidateSince = millis();
        }

        if (millis() - leaderTakeoffCandidateSince < LEADER_TAKEOFF_CONFIRM_MS) {
            return false;
        }

        takeoffAlt = clampTargetAlt((leaderData.alt / 1000.0f) + (leaderData.teamHigh / 100.0f));

        // 多旋翼跟随固定翼长机时，不能直接用长机当前高度做起飞目标，否则会猛冲到过高空域。
        // 这里把自动起飞阶段限制为温和爬升到 3m~4.5m，进入编队后再继续跟随爬升。
        if (_status.vehicleType == VEHICLE_TYPE_COPTER) {
            float minTakeoffAlt = LOWEST_ALTITUDE / 1000.0f;
            float maxTakeoffAlt = COPTER_AUTO_TAKEOFF_MAX_ALT / 1000.0f;
            if (takeoffAlt > maxTakeoffAlt) takeoffAlt = maxTakeoffAlt;
            if (takeoffAlt < minTakeoffAlt) takeoffAlt = minTakeoffAlt;
        }

        // 确保自动起飞的高度绝不会低于系统最低安全高度
        if (takeoffAlt < (LOWEST_ALTITUDE / 1000.0f)) {
            takeoffAlt = LOWEST_ALTITUDE / 1000.0f;
        }
        return true;
    }

    leaderTakeoffCandidateSince = 0;
    return false;
}

bool Formation::shouldAutoArmForTakeoff() {
    if (_config.AP || lastLeaderPacketTime == 0) return false;
    if (leaderData.autoTakeoff == 0) return false;
    if (millis() - lastLeaderPacketTime > MAX_LOST_WAIT_TIME) return false;
    if (!isGpsHealthy() || !isEkfHealthy()) return false;
    if (leaderData.gpsHealthy == 0 || leaderData.ekfHealthy == 0) return false;
    if (leaderData.isArmed == 0) return false;
    if (_status.pos.relative_alt >= LOWEST_ALTITUDE) return false;
    return true;
}

bool Formation::shouldAutoLand() {
    if (_config.AP || lastLeaderPacketTime == 0) return false;

    // 领航机信号丢失超过限制，此时应该交给飞控自己的失控保护（如RTL或降落），而不是由ESP32强制发指令
    if (millis() - lastLeaderPacketTime > MAX_LOST_WAIT_TIME) return false;

    // 领航机未允许自动降落
    if (leaderData.autoLand == 0) return false;

    // 固定翼长机应在“已接地并减速稳定”后就触发僚机降落，
    // 不能等飞手手动上锁，否则僚机会在空中继续悬停。
    if (leaderData.vehicleType == VEHICLE_TYPE_PLANE) {
        bool landedCandidate = leaderData.alt <= LEADER_LAND_ALT_THRESHOLD &&
                               leaderData.groundSpeed <= PLANE_LEADER_LAND_SPEED_THRESHOLD;

        if (landedCandidate) {
            if (leaderLandingCandidateSince == 0) {
                leaderLandingCandidateSince = millis();
            }

            if (millis() - leaderLandingCandidateSince >= LEADER_LAND_CONFIRM_MS) {
                return true;
            }
        } else {
            leaderLandingCandidateSince = 0;
        }

        return false;
    }

    leaderLandingCandidateSince = 0;

    return false;
}

bool Formation::isLeaderLost() {
    if (_config.AP) return false;
    if (lastLeaderPacketTime == 0) return true;
    return (millis() - lastLeaderPacketTime > MAX_LOST_WAIT_TIME);
}

unsigned long Formation::leaderLinkAgeMs() {
    if (_config.AP) return 0;
    if (lastLeaderPacketTime == 0) return MAX_LOST_WAIT_TIME + 1;
    return millis() - lastLeaderPacketTime;
}

bool Formation::isSafetyOk() {
    // 0. 自身飞控串口连接检查 (极度重要：如果 ESP32 与自己飞控的连接断开超过 2 秒，立刻熔断)
    if (_status.lastFcTime == 0 || millis() - _status.lastFcTime > 2000) return false;
    if (!isGpsHealthy()) return false;
    if (!isEkfHealthy()) return false;

    // 1. 高度检查
    // 注意：_status.pos.alt 是海拔高度，如果要在起飞前或低空测试，建议使用 relative_alt
    // _status.pos.relative_alt 的单位是毫米 (mm)
    // 假设 LOWEST_ALTITUDE 也是毫米，如果它是 3000 (即 3 米)，则只有在高度大于 3 米时才通过
    if (_status.pos.relative_alt < LOWEST_ALTITUDE) return false;
    
    // 2. 领航机信号检查
    if (!_config.AP) {
        if (lastLeaderPacketTime == 0) return false; // 刚开机还没收到过长机数据，禁止伴飞
        if (millis() - lastLeaderPacketTime > MAX_LOST_WAIT_TIME) return false;
        if (leaderData.gpsHealthy == 0 || leaderData.ekfHealthy == 0) return false;
        
        // 极度危险的 Bug 修复：如果僚机在天上，但长机还在地上没起飞，绝对不能跟随！
        // 否则僚机会收到长机高度为 0 的坐标，直接“钻地”坠毁。
        if (leaderData.alt < LOWEST_ALTITUDE) return false;
    }
    
    // 3. 距离检查 (简单的欧几里得距离近似)
    if (!_config.AP && lastLeaderPacketTime > 0) {
        // 经纬度差值(degE7) * 1e-7 (转成度) * 111111 (转成米)
        // lon 的转换需要乘以当前纬度的 cos 值才精确，这里为了安全检查做粗略计算：
        // dx = diff * 1e-7 * 111111 ≈ diff * 0.011111
        float dx = (leaderData.lon - _status.pos.lon) * 0.011111f;
        
        // 修正：计算经度差值的距离时，必须乘以当前纬度的余弦值，否则在高纬度地区误差极大！
        // 纬度需要先转换成弧度：lat * 1e-7 * PI / 180.0
        float lat_rad = _status.pos.lat * 1e-7f * PI / 180.0f;
        dx = dx * cos(lat_rad);
        
        float dy = (leaderData.lat - _status.pos.lat) * 0.011111f;
        float distSq = dx*dx + dy*dy;
        
        // MAX_TEAM_DISTANT 单位是毫米(200000 = 200米)
        float maxDist_m = MAX_TEAM_DISTANT / 1000.0f;
        if (distSq > (maxDist_m * maxDist_m)) return false; 
    }

    return true;
}

bool Formation::getTargetPoint(int32_t &targetLat, int32_t &targetLon, float &targetAlt) {
    if (_config.AP || lastLeaderPacketTime == 0) return false;

    if (shouldUsePlaneOrbitFollow()) {
        return getPlaneOrbitTarget(targetLat, targetLon, targetAlt);
    }

    float base_dist_m = leaderData.teamDist / 100.0f; // cm to m
    float leader_heading = leaderData.heading; // 0-360
    int32_t anchorLat = leaderData.lat;
    int32_t anchorLon = leaderData.lon;
    
    // 固定翼不适合直接追当前点，前推一段距离生成虚拟引导点，减小蛇形追踪。
    if (leaderData.vehicleType == VEHICLE_TYPE_PLANE) {
        float lead_dist_m = leaderData.groundSpeed * PLANE_GUIDE_LEAD_TIME;
        if (lead_dist_m < 15.0f) lead_dist_m = 15.0f;
        if (lead_dist_m > 40.0f) lead_dist_m = 40.0f;
        offsetGPS(anchorLat, anchorLon, lead_dist_m, leader_heading, anchorLat, anchorLon);
    }
    
    float bearing_deg = leader_heading + 180.0f; // 默认正后方
    float effective_dist = base_dist_m;
    int order = _config.byPlaneOrder; // 1, 2, 3...

    if (leaderData.teamType == 0) { // 直线编队
        effective_dist = base_dist_m * order;
    } 
    else if (leaderData.teamType == 1) { // V字阵型
        if (order % 2 == 0) { // 偶数号：左后方
            bearing_deg += 45.0f;
            effective_dist = base_dist_m * (order / 2);
        } else { // 奇数号：右后方
            bearing_deg -= 45.0f;
            effective_dist = base_dist_m * ((order + 1) / 2);
        }
    }
    else if (leaderData.teamType == 2) { // 一字型编队 (横向排开)
        if (order % 2 == 0) { // 偶数号：左侧
            bearing_deg = leader_heading - 90.0f;
            effective_dist = base_dist_m * (order / 2);
        } else { // 奇数号：右侧
            bearing_deg = leader_heading + 90.0f;
            effective_dist = base_dist_m * ((order + 1) / 2);
        }
    }
    else if (leaderData.teamType == 3) { // 梯形编队 (全部偏右后方)
        bearing_deg -= 45.0f;
        effective_dist = base_dist_m * order;
    }

    while (bearing_deg >= 360.0) bearing_deg -= 360.0;
    while (bearing_deg < 0.0) bearing_deg += 360.0;

    offsetGPS(anchorLat, anchorLon, effective_dist, bearing_deg, targetLat, targetLon);
    targetAlt = clampTargetAlt((leaderData.alt / 1000.0f) + (leaderData.teamHigh / 100.0f)); // mm to m, plus offset

    return true;
}

float Formation::distanceMeters(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
    float lat_rad = lat1 * 1e-7f * PI / 180.0f;
    float dx = (lon2 - lon1) * 0.011111f * cos(lat_rad);
    float dy = (lat2 - lat1) * 0.011111f;
    return sqrtf(dx * dx + dy * dy);
}

float Formation::bearingDeg(int32_t fromLat, int32_t fromLon, int32_t toLat, int32_t toLon) {
    float lat_rad = fromLat * 1e-7f * PI / 180.0f;
    float dx = (toLon - fromLon) * 0.011111f * cos(lat_rad);
    float dy = (toLat - fromLat) * 0.011111f;
    float bearing = atan2f(dx, dy) * 180.0f / PI;
    if (bearing < 0.0f) bearing += 360.0f;
    return bearing;
}

bool Formation::shouldUsePlaneOrbitFollow() const {
    if (_config.AP) return false;
    if (_status.vehicleType != VEHICLE_TYPE_PLANE) return false;

    // 固定翼跟随多旋翼或低速长机时，改用绕点伴飞策略。
    return (leaderData.vehicleType != VEHICLE_TYPE_PLANE) || (leaderData.groundSpeed < PLANE_LOW_SPEED_LEADER);
}

bool Formation::getPlaneOrbitTarget(int32_t &targetLat, int32_t &targetLon, float &targetAlt) {
    int order = _config.byPlaneOrder;
    if (order < 1) order = 1;

    float orbitRadius = PLANE_ORBIT_MIN_RADIUS + (order - 1) * PLANE_ORBIT_STEP_RADIUS;
    float radialBearing = bearingDeg(leaderData.lat, leaderData.lon, _status.pos.lat, _status.pos.lon);
    float currentDist = distanceMeters(leaderData.lat, leaderData.lon, _status.pos.lat, _status.pos.lon);

    // 如果刚加入时还靠得太近，就优先把目标点放到外圈，避免固定翼切入长机头顶。
    if (currentDist < orbitRadius * 0.6f) {
        radialBearing = leaderData.heading + ((order % 2 == 0) ? -90.0f : 90.0f);
    }

    float orbitDirection = (order % 2 == 0) ? -1.0f : 1.0f;
    float tangentBearing = radialBearing + orbitDirection * PLANE_ORBIT_ADVANCE_DEG;

    while (tangentBearing >= 360.0f) tangentBearing -= 360.0f;
    while (tangentBearing < 0.0f) tangentBearing += 360.0f;

    offsetGPS(leaderData.lat, leaderData.lon, orbitRadius, tangentBearing, targetLat, targetLon);
    targetAlt = clampTargetAlt((leaderData.alt / 1000.0f) + (leaderData.teamHigh / 100.0f));
    return true;
}

bool Formation::isGpsHealthy() const {
    if (_status.lastGpsTime == 0 || _status.lastPosTime == 0) return false;
    if (millis() - _status.lastGpsTime > SENSOR_TIMEOUT_MS) return false;
    if (millis() - _status.lastPosTime > SENSOR_TIMEOUT_MS) return false;
    if (_status.gpsData.fix_type < MIN_GPS_FIX_TYPE) return false;
    if (_status.gpsData.satellites_visible < MIN_GPS_SATELLITES) return false;
    if (_status.gpsData.eph == UINT16_MAX || _status.gpsData.eph > MAX_GPS_EPH) return false;
    if (_status.gpsData.epv == UINT16_MAX || _status.gpsData.epv > MAX_GPS_EPV) return false;
    if (_status.pos.lat == 0 || _status.pos.lon == 0) return false;
    return true;
}

bool Formation::isEkfHealthy() const {
    if (_status.lastEkfTime == 0 || (millis() - _status.lastEkfTime > SENSOR_TIMEOUT_MS)) {
        _status.ekfHealthySince = 0;
        return false;
    }

    bool attitudeOk = (_status.ekfFlags & EKF_ATTITUDE) != 0;
    bool horizOk = (_status.ekfFlags & EKF_POS_HORIZ_ABS) != 0;
    bool vertOk = ((_status.ekfFlags & EKF_POS_VERT_ABS) != 0) ||
                  ((_status.ekfFlags & EKF_POS_VERT_AGL) != 0);
    bool constPosMode = (_status.ekfFlags & EKF_CONST_POS_MODE) != 0;
    
    // 新增：方差检查
    bool varianceOk = (_status.ekf_vel_variance < EKF_MAX_VEL_VARIANCE) && 
                      (_status.ekf_pos_horiz_variance < EKF_MAX_POS_VARIANCE);

    bool currentHealthy = attitudeOk && horizOk && vertOk && !constPosMode && varianceOk;

    if (!currentHealthy) {
        _status.ekfHealthySince = 0;
        return false;
    }

    // 记录开始健康的时间
    if (_status.ekfHealthySince == 0) {
        _status.ekfHealthySince = millis();
    }

    // 只有持续健康超过 EKF_STABLE_REQUIRED_MS 才返回 true
    return (millis() - _status.ekfHealthySince >= EKF_STABLE_REQUIRED_MS);
}

float Formation::clampTargetAlt(float alt) const {
    float minAlt = LOWEST_ALTITUDE / 1000.0f;
    float maxAlt = MAX_FORMATION_TARGET_ALT / 1000.0f;
    if (alt < minAlt) return minAlt;
    if (alt > maxAlt) return maxAlt;
    return alt;
}

void Formation::offsetGPS(int32_t lat, int32_t lon, float dist_m, float bearing_deg, int32_t &outLat, int32_t &outLon) {
    float bearing_rad = bearing_deg * PI / 180.0;
    
    // 1度纬度约 111111米
    double lat_offset = (dist_m * cos(bearing_rad)) / 111111.0;
    // 1度经度约 111111 * cos(lat) 米
    double lon_offset = (dist_m * sin(bearing_rad)) / (111111.0 * cos(lat * 1e-7 * PI / 180.0));

    outLat = lat + (int32_t)(lat_offset * 1e7);
    outLon = lon + (int32_t)(lon_offset * 1e7);
}
