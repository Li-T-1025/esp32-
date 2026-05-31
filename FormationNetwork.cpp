#include "FormationNetwork.h"
#include "ConfigStorage.h"
#include <esp_wifi.h>
#include <string.h>

extern ConfigStorage storage;

static const char* wifiStatusName(wl_status_t status) {
    switch (status) {
        case WL_NO_SHIELD: return "NO_SHIELD";
        case WL_IDLE_STATUS: return "IDLE";
        case WL_NO_SSID_AVAIL: return "NO_SSID";
        case WL_SCAN_COMPLETED: return "SCAN_DONE";
        case WL_CONNECTED: return "CONNECTED";
        case WL_CONNECT_FAILED: return "CONNECT_FAILED";
        case WL_CONNECTION_LOST: return "CONNECTION_LOST";
        case WL_DISCONNECTED: return "DISCONNECTED";
        default: return "UNKNOWN";
    }
}

static int clampInt(int value, int minValue, int maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static String escapeJsonString(const String &input) {
    String out;
    out.reserve(input.length() + 8);
    for (size_t i = 0; i < input.length(); ++i) {
        char c = input[i];
        switch (c) {
            case '\\': out += "\\\\"; break;
            case '"': out += "\\\""; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if ((unsigned char)c < 0x20) {
                    char buf[7];
                    snprintf(buf, sizeof(buf), "\\u%04x", (unsigned char)c);
                    out += buf;
                } else {
                    out += c;
                }
                break;
        }
    }
    return out;
}

static String escapeHtmlString(const String &input) {
    String out;
    out.reserve(input.length() + 8);
    for (size_t i = 0; i < input.length(); ++i) {
        char c = input[i];
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            case '\'': out += "&#39;"; break;
            default: out += c; break;
        }
    }
    return out;
}

FormationNetwork::FormationNetwork(MyConfig &config, MyStatus &status, MyLine *lines) 
    : _config(config), _status(status), _lines(lines), server(80),
      lastStaReconnectAttempt(0), lastStaConnectStart(0),
      lastStaConnected(false), lastStaWifiStatus(WL_IDLE_STATUS) {}

void FormationNetwork::begin() {
    if (_config.AP) {
        setupAP();
    } else {
        setupSTA();
    }

    server.on("/", [this]() { handleRoot(); });
    server.on("/save", HTTP_POST, [this]() { handleSave(); });
    server.on("/status", [this]() { handleStatus(); });
    server.on("/config", [this]() { handleConfig(); });
    server.on("/cmd", HTTP_POST, [this]() {
        if (!_config.AP) {
            server.send(403, "application/json", "{\"success\":0,\"message\":\"只有领航机模式允许发送机队指令。\"}");
            return;
        }
        auto onlineFollowerCount = [this]() {
            int count = 0;
            for (int i = 0; i < FLIGHT_NUM; i++) {
                if (_lines[i].useage && (millis() - _lines[i].line_time) <= MAX_LOST_WAIT_TIME) {
                    count++;
                }
            }
            return count;
        };

        String message;
        uint8_t cmdType = CMD_NONE;
        String cmd = server.arg("cmd");
        if (cmd == "armtest") {
            cmdType = CMD_ARM_TEST;
            message = "已广播解锁测试指令";
        } else if (cmd == "disarmtest") {
            cmdType = CMD_DISARM_TEST;
            message = "已广播上锁测试指令";
        } else if (cmd == "land") {
            cmdType = CMD_LAND_ALL;
            message = "已广播返航/降落指令";
        } else if (cmd == "takeoff") {
            cmdType = CMD_TAKEOFF_ALL;
            message = "已广播一键起飞指令（仅多旋翼僚机执行）";
        } else if (cmd == "join") {
            cmdType = CMD_JOIN_ALL;
            message = "已广播加入编队指令";
        } else {
            server.send(400, "application/json", "{\"success\":0,\"message\":\"缺少有效指令参数。\"}");
            return;
        }

        _status.lastIssuedCmdSeq++;
        if (_status.lastIssuedCmdSeq == 0) _status.lastIssuedCmdSeq = 1;
        _status.activeCmdType = cmdType;
        _status.activeCmdSeq = _status.lastIssuedCmdSeq;
        _status.activeCmdUntil = millis() + COMMAND_BROADCAST_WINDOW_MS;

        int activeFollowers = onlineFollowerCount();
        String json = "{";
        json += "\"success\":1,";
        json += "\"cmdType\":" + String(cmdType) + ",";
        json += "\"cmdSeq\":" + String(_status.activeCmdSeq) + ",";
        json += "\"onlineFollowers\":" + String(activeFollowers) + ",";
        json += "\"message\":\"" + message + "，将持续广播约" + String(COMMAND_BROADCAST_WINDOW_MS / 1000.0f, 1) + "秒，当前在线僚机数：" + String(activeFollowers) + "。\"";
        json += "}";
        server.send(200, "application/json", json);
    });
    
    // Captive Portal 重定向：将所有未知路径请求重定向到主页
    server.onNotFound([this]() {
        server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
        server.send(302, "text/plain", "");
    });
    
    server.begin();
}

void FormationNetwork::setupAP() {
    WiFi.mode(WIFI_AP);
    IPAddress local_IP(192, 168, 1, 1);
    IPAddress gateway(192, 168, 1, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(_config.chSsid, _config.chPasswd);
    _status.byWifi = 1;

    // 启动 DNS 服务器，将所有域名 (*.com等) 都解析到 ESP32 的 IP 地址
    dnsServer.start(53, "*", local_IP);
}

void FormationNetwork::setupSTA() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(_config.chSsid, _config.chPasswd);
    lastStaReconnectAttempt = millis();
    lastStaConnectStart = lastStaReconnectAttempt;
    lastStaConnected = false;
    lastStaWifiStatus = WL_IDLE_STATUS;
    Serial.printf("Follower WiFi connecting to SSID: %s\n", _config.chSsid);
    // 僚机连接状态由主循环监控
}

void FormationNetwork::maintainSTAConnection() {
    unsigned long now = millis();
    wl_status_t wifiStatus = WiFi.status();
    bool connected = (wifiStatus == WL_CONNECTED);
    _status.byWifi = connected ? 1 : 0;

    if (wifiStatus != lastStaWifiStatus) {
        Serial.printf("Follower WiFi status -> %s (%d)\n", wifiStatusName(wifiStatus), (int)wifiStatus);
        lastStaWifiStatus = wifiStatus;
    }

    if (connected) {
        if (!lastStaConnected) {
            Serial.printf("Follower WiFi connected. IP=%s Gateway=%s RSSI=%d\n",
                          WiFi.localIP().toString().c_str(),
                          WiFi.gatewayIP().toString().c_str(),
                          WiFi.RSSI());
        }
        lastStaConnected = true;
        lastStaConnectStart = 0;
        return;
    }

    if (lastStaConnected) {
        Serial.println("WiFi link lost, follower entering reconnect mode...");
        lastStaConnected = false;
        lastStaConnectStart = now;
    }

    // 每次主动发起连接后，给 WiFi 足够的关联/认证/DHCP 时间，避免反复 disconnect 打断正常入网。
    if (lastStaConnectStart != 0 && now - lastStaConnectStart < WIFI_CONNECT_WINDOW_MS) {
        return;
    }

    if (now - lastStaReconnectAttempt >= WIFI_RECONNECT_INTERVAL) {
        WiFi.disconnect();
        WiFi.begin(_config.chSsid, _config.chPasswd);
        lastStaReconnectAttempt = now;
        lastStaConnectStart = now;
        Serial.println("Retrying WiFi connection to leader after timeout...");
    }
}

void FormationNetwork::handle() {
    if (_config.AP) {
        // 处理 DNS 请求
        dnsServer.processNextRequest();
    }

    server.handleClient();
    if (!_config.AP) {
        maintainSTAConnection();
    }
}

bool FormationNetwork::isConnected() {
    return _status.byWifi == 1;
}

void FormationNetwork::handleRoot() {
    server.send(200, "text/html", getHTML());
}

void FormationNetwork::handleSave() {
    auto copyArgToBuffer = [this](const char *argName, char *buffer, size_t bufferSize) {
        if (!server.hasArg(argName) || bufferSize == 0) return;
        String value = server.arg(argName);
        strncpy(buffer, value.c_str(), bufferSize - 1);
        buffer[bufferSize - 1] = '\0';
    };

    MyConfig newConfig = _config;
    copyArgToBuffer("nickname", newConfig.MyNikeName, sizeof(newConfig.MyNikeName));
    copyArgToBuffer("ssid", newConfig.chSsid, sizeof(newConfig.chSsid));
    if (server.hasArg("password") && server.arg("password").length() > 0) {
        copyArgToBuffer("password", newConfig.chPasswd, sizeof(newConfig.chPasswd));
    }
    if (server.hasArg("dist")) newConfig.uTeamDist = server.arg("dist").toInt();
    if (server.hasArg("high")) newConfig.uTeamHigh = server.arg("high").toInt();
    if (server.hasArg("type")) newConfig.byTeamType = server.arg("type").toInt();
    if (server.hasArg("mode")) newConfig.AP = server.arg("mode").toInt();
    if (server.hasArg("autoTakeoff")) newConfig.autoTakeoff = server.arg("autoTakeoff").toInt();
    if (server.hasArg("autoLand")) newConfig.autoLand = server.arg("autoLand").toInt();
    if (server.hasArg("order")) newConfig.byPlaneOrder = server.arg("order").toInt();
    if (server.hasArg("rcSyncEnabled")) newConfig.rcSyncEnabled = server.arg("rcSyncEnabled").toInt();
    if (server.hasArg("rcSyncSourceChannel")) newConfig.rcSyncSourceChannel = server.arg("rcSyncSourceChannel").toInt();
    if (server.hasArg("rcSyncTargetChannel")) newConfig.rcSyncTargetChannel = server.arg("rcSyncTargetChannel").toInt();
    newConfig.AP = clampInt(newConfig.AP, 0, 1);
    newConfig.autoTakeoff = clampInt(newConfig.autoTakeoff, 0, 1);
    newConfig.autoLand = clampInt(newConfig.autoLand, 0, 1);
    newConfig.rcSyncEnabled = clampInt(newConfig.rcSyncEnabled, 0, 1);
    newConfig.byTeamType = clampInt(newConfig.byTeamType, 0, 3);
    newConfig.byPlaneOrder = clampInt(newConfig.byPlaneOrder, 1, FLIGHT_NUM);
    newConfig.uTeamDist = clampInt(newConfig.uTeamDist, MIN_TEAM_DISTANCE_CM, MAX_TEAM_DISTANT / 10);
    newConfig.uTeamHigh = clampInt(newConfig.uTeamHigh, -5000, 5000);
    if (newConfig.rcSyncSourceChannel < 7 || newConfig.rcSyncSourceChannel > 12) newConfig.rcSyncSourceChannel = 7;
    if (newConfig.rcSyncTargetChannel < 7 || newConfig.rcSyncTargetChannel > 12) newConfig.rcSyncTargetChannel = 7;

    size_t ssidLen = strlen(newConfig.chSsid);
    size_t passwordLen = strlen(newConfig.chPasswd);
    if (ssidLen == 0 || ssidLen > 32) {
        server.send(400, "application/json", "{\"success\":0,\"message\":\"WiFi 名称长度必须在 1 到 32 个字符之间。\"}");
        return;
    }
    if (passwordLen < 8 || passwordLen > 63) {
        server.send(400, "application/json", "{\"success\":0,\"message\":\"WiFi 密码长度必须在 8 到 63 个字符之间。\"}");
        return;
    }

    bool modeChanged = (newConfig.AP != _config.AP);
    bool ssidChanged = strcmp(newConfig.chSsid, _config.chSsid) != 0;
    bool passwordChanged = strcmp(newConfig.chPasswd, _config.chPasswd) != 0;
    bool requiresRestart = modeChanged || ssidChanged || passwordChanged;
    bool confirmRestart = server.hasArg("confirmRestart") && server.arg("confirmRestart").toInt() == 1;

    String restartReason;
    if (requiresRestart) {
        if (modeChanged) restartReason += "切换长机/僚机模式";
        if (ssidChanged) {
            if (restartReason.length()) restartReason += "、";
            restartReason += "修改 WiFi 名称";
        }
        if (passwordChanged) {
            if (restartReason.length()) restartReason += "、";
            restartReason += "修改 WiFi 密码";
        }
    }

    if (requiresRestart && !confirmRestart) {
        String json = "{";
        json += "\"success\":1,";
        json += "\"saved\":0,";
        json += "\"needsRestart\":1,";
        json += "\"restarting\":0,";
        json += "\"message\":\"以下修改需要重启后生效：" + restartReason + "。是否确认保存并立即重启？\"";
        json += "}";
        server.send(200, "application/json", json);
        return;
    }

    _config = newConfig;
    storage.saveConfig(_config);

    if (requiresRestart) {
        String json = "{\"success\":1,\"saved\":1,\"needsRestart\":1,\"restarting\":1,\"message\":\"设置已保存，设备即将重启...\"}";
        server.send(200, "application/json", json);
        delay(500);
        ESP.restart();
        return;
    }

    String json = "{\"success\":1,\"saved\":1,\"needsRestart\":0,\"restarting\":0,\"message\":\"设置已保存并立即生效。\"}";
    server.send(200, "application/json", json);
}

void FormationNetwork::handleStatus() {
    unsigned long leaderLinkAgeMs = 0;
    if (!_config.AP) {
        leaderLinkAgeMs = (_status.lastWifiTime == 0) ? (MAX_LOST_WAIT_TIME + 1) : (millis() - _status.lastWifiTime);
    }

    String json = "{";
    json += "\"localIP\":\"" + (_config.AP ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "\",";
    json += "\"vehicleType\":" + String(_status.vehicleType) + ",";
    json += "\"lat\":" + String(_status.gpsData.lat / 1e7, 7) + ",";
    json += "\"lon\":" + String(_status.gpsData.lon / 1e7, 7) + ",";
    json += "\"alt\":" + String(_status.pos.alt / 1000.0) + ","; // 海拔高度
    json += "\"rel_alt\":" + String(_status.pos.relative_alt / 1000.0) + ","; // 相对高度
    json += "\"groundSpeed\":" + String(_status.groundSpeed, 1) + ",";
    json += "\"sat\":" + String(_status.gpsData.satellites_visible) + ",";
    json += "\"volt\":" + String(_status.sysStatus.voltage_battery / 1000.0) + ",";
    
    bool isArmed = _status.heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
    json += "\"armed\":" + String(isArmed ? 1 : 0) + ",";
    json += "\"mode\":" + String(_status.heartbeat.custom_mode) + ",";
    
    // 遥控器信号通常看 rxerrors 或者用 rssi (如果你有外接接收机RSSI)，这里以 RC 通道更新或 RSSI 字段为准
    json += "\"rssi\":" + String(_status.rc_channels.rssi) + ","; 
    // 修正：VFR_HUD 传过来的 heading 就是标准度数，无需除以 100
    json += "\"heading\":" + String(_status.hudData.heading) + ","; // 航向角 0-359
    
    json += "\"wifi\":" + String(_status.byWifi) + ",";
    json += "\"activeCmdType\":" + String(_status.activeCmdType) + ",";
    json += "\"activeCmdSeq\":" + String(_status.activeCmdSeq) + ",";
    json += "\"activeCmdBroadcasting\":" + String((_config.AP && _status.activeCmdType != CMD_NONE && millis() <= _status.activeCmdUntil) ? 1 : 0) + ",";
    json += "\"leaderLinkAgeMs\":" + String(leaderLinkAgeMs) + ",";
    json += "\"lastSentPacketCounter\":" + String(_status.lastSentPacketCounter) + ",";
    json += "\"lastLeaderPacketCounter\":" + String(_status.lastLeaderPacketCounter) + ",";
    json += "\"lastFcAckCommand\":" + String(_status.lastFcAckCommand) + ",";
    json += "\"lastFcAckResult\":" + String(_status.lastFcAckResult) + ",";
    json += "\"lastFcAckAgeMs\":" + String(_status.lastFcAckTime == 0 ? 0 : (millis() - _status.lastFcAckTime)) + ",";
    json += "\"lastPacketRejectReason\":" + String(_status.lastPacketRejectReason) + ",";
    json += "\"lastRejectedPacketCounter\":" + String(_status.lastRejectedPacketCounter) + ",";
    json += "\"lastPacketRejectAgeMs\":" + String(_status.lastPacketRejectTime == 0 ? 0 : (millis() - _status.lastPacketRejectTime)) + ",";
    json += "\"takeoffReason\":" + String(_status.takeoffReason) + ",";
    json += "\"landReason\":" + String(_status.landReason) + ",";
    json += "\"joinReason\":" + String(_status.joinReason) + ",";
    
    // 新增网络角色与状态信息
    json += "\"isAP\":" + String(_config.AP) + ",";
    if (_config.AP) {
        json += "\"staCount\":" + String(WiFi.softAPgetStationNum()) + ",";
        json += "\"gatewayIP\":\"" + WiFi.softAPIP().toString() + "\",";
        
        wifi_sta_list_t stationList;
        esp_wifi_ap_get_sta_list(&stationList);
        json += "\"staMacs\":[";
        for(int i = 0; i < stationList.num; i++) {
            char macStr[18];
            sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", 
                    stationList.sta[i].mac[0], stationList.sta[i].mac[1], stationList.sta[i].mac[2], 
                    stationList.sta[i].mac[3], stationList.sta[i].mac[4], stationList.sta[i].mac[5]);
            json += "\"" + String(macStr) + "\"";
            if(i < stationList.num - 1) json += ",";
        }
        json += "],";
        json += "\"followers\":[";
        bool firstFollower = true;
        for (int i = 0; i < FLIGHT_NUM; i++) {
            if (_lines[i].useage && (millis() - _lines[i].line_time) <= MAX_LOST_WAIT_TIME) {
                if (!firstFollower) json += ",";
                firstFollower = false;
                char ipStr[16];
                sprintf(ipStr, "%u.%u.%u.%u", _lines[i].ip[0], _lines[i].ip[1], _lines[i].ip[2], _lines[i].ip[3]);
                char macStr[18];
                sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
                        _lines[i].mac[0], _lines[i].mac[1], _lines[i].mac[2],
                        _lines[i].mac[3], _lines[i].mac[4], _lines[i].mac[5]);
                json += "{";
                json += "\"name\":\"" + escapeJsonString(String(_lines[i].chNikeName)) + "\",";
                json += "\"ip\":\"" + String(ipStr) + "\",";
                json += "\"mac\":\"" + String(macStr) + "\",";
                json += "\"vehicleType\":" + String(_lines[i].vehicleType) + ",";
                json += "\"armed\":" + String(_lines[i].isArmed) + ",";
                json += "\"rel_alt\":" + String(_lines[i].rel_alt / 1000.0f, 1) + ",";
                json += "\"ackCmdType\":" + String(_lines[i].ackCmdType) + ",";
                json += "\"ackCmdSeq\":" + String(_lines[i].ackCmdSeq) + ",";
                json += "\"ackCmdStatus\":" + String(_lines[i].ackCmdStatus) + ",";
                json += "\"packetCounter\":" + String(_lines[i].lastPacketCounter);
                json += "}";
            }
        }
        json += "]";
    } else {
        json += "\"staCount\":0,";
        json += "\"gatewayIP\":\"" + WiFi.gatewayIP().toString() + "\"";
    }
    
    json += "}";
    server.send(200, "application/json", json);
}

void FormationNetwork::handleConfig() {
    bool passwordConfigured = strlen(_config.chPasswd) >= 8;

    String json = "{";
    json += "\"nickname\":\"" + escapeJsonString(String(_config.MyNikeName)) + "\",";
    json += "\"mode\":" + String(_config.AP) + ",";
    json += "\"ssid\":\"" + escapeJsonString(String(_config.chSsid)) + "\",";
    json += "\"password\":\"\",";
    json += "\"passwordConfigured\":" + String(passwordConfigured ? 1 : 0) + ",";
    json += "\"dist\":" + String(_config.uTeamDist) + ",";
    json += "\"high\":" + String(_config.uTeamHigh) + ",";
    json += "\"type\":" + String(_config.byTeamType) + ",";
    json += "\"order\":" + String(_config.byPlaneOrder) + ",";
    json += "\"autoTakeoff\":" + String(_config.autoTakeoff) + ",";
    json += "\"autoLand\":" + String(_config.autoLand) + ",";
    json += "\"rcSyncEnabled\":" + String(_config.rcSyncEnabled) + ",";
    json += "\"rcSyncSourceChannel\":" + String(_config.rcSyncSourceChannel) + ",";
    json += "\"rcSyncTargetChannel\":" + String(_config.rcSyncTargetChannel);
    json += "}";
    server.send(200, "application/json", json);
}

String FormationNetwork::getHTML() {
    String html = "<html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:sans-serif;background:#87CEEB;padding:20px;} .card{background:#fff;padding:20px;border-radius:10px;margin-bottom:20px;box-shadow:0 2px 5px rgba(0,0,0,0.1);} h1{text-align:center;color:#fff;text-shadow:1px 1px 2px rgba(0,0,0,0.3);} h2{margin-top:0;} input,select{width:100%;padding:10px;margin:10px 0;border:1px solid #ccc;border-radius:5px;} button{width:100%;padding:10px;background:#007bff;color:#fff;border:none;border-radius:5px;cursor:pointer;}</style></head><body>";
    
    html += "<h1>带你飞无人机编队系统</h1>";
    
    html += "<div class='card'><h2>飞行器信息</h2>";
    html += "<p>纬度: <span id='lat'>--</span></p>";
    html += "<p>经度: <span id='lon'>--</span></p>";
    html += "<p>海拔高度: <span id='alt'>--</span>m</p>";
    html += "<p>相对高度: <span id='rel_alt'>--</span>m</p>";
    html += "<p>飞控机型: <span id='vehicleType'>--</span></p>";
    html += "<p>航向角: <span id='heading'>--</span>°</p>";
    html += "<p>地速: <span id='groundSpeed'>--</span>m/s</p>";
    html += "<p>卫星: <span id='sat'>--</span></p>";
    html += "<p>电压: <span id='volt'>--</span>V</p>";
    html += "<p>飞控状态: <span id='armed' style='font-weight:bold;'>--</span></p>";
    html += "<p>飞行模式: <span id='mode'>--</span></p>";
    html += "<p>遥控信号(RSSI): <span id='rssi'>--</span></p></div>";

    html += "<div class='card'><h2>网络状态</h2>";
    html += "<div id='ap_status' style='display:none;'>";
    html += "<p>当前角色: <span style='color:blue;font-weight:bold;'>领航机 (AP)</span></p>";
    html += "<p>本机 IP: <span id='apIP'>--</span></p>";
    html += "<p>已连接数量: <span id='staCount' style='font-weight:bold;color:green;'>0</span> 台</p>";
    html += "<div id='staListDiv' style='font-size:14px; color:#333; background:#e9ecef; padding:8px; border-radius:5px; margin-top:10px;'></div>";
    html += "<div id='followerInfoDiv' style='font-size:14px; color:#333; background:#f8f9fa; padding:8px; border-radius:5px; margin-top:10px;'></div></div>";
    
    html += "<div id='sta_status' style='display:none;'>";
    html += "<p>当前角色: <span style='color:purple;font-weight:bold;'>僚机 (STA)</span></p>";
    html += "<p>本机 IP: <span id='staIP'>--</span></p>";
    html += "<p>WiFi 连接状态: <span id='wifiStatus'>--</span></p>";
    html += "<p>领航机 IP: <span id='gatewayIP'>--</span></p></div></div>";
 
    html += "<div class='card'><h2>运行诊断</h2>";
    html += "<p>链路状态: <span id='diagRole'>--</span></p>";
    html += "<p>领航机链路年龄: <span id='diagLeaderAge'>--</span></p>";
    html += "<p>最近发送报文计数: <span id='diagTxCounter'>--</span></p>";
    html += "<p>最近领航机报文计数: <span id='diagLeaderCounter'>--</span></p>";
    html += "<p>最近飞控 ACK: <span id='diagFcAck'>--</span></p>";
    html += "<p>最近拒收报文: <span id='diagReject'>--</span></p>";
    html += "<p>起飞状态说明: <span id='diagTakeoffReason'>--</span></p>";
    html += "<p>降落状态说明: <span id='diagLandReason'>--</span></p>";
    html += "<p>编队状态说明: <span id='diagJoinReason'>--</span></p></div>";
  
    // 机队实时控制面板 (暂时隐藏)
    html += "<div class='card' id='realtimeControl' style='display:none;'><h2>机队实时控制</h2>";
    html += "<div style='display:flex; gap:10px; margin-bottom:10px;'>";
    html += "<button onclick='sendFleetCommand(\"armtest\", 1, \"确认向所有僚机发送解锁测试指令？请确保已拆桨或固定机体。\")' style='background:#dc3545; height:60px; font-size:18px;'>解锁测试</button>";
    html += "<button onclick='sendFleetCommand(\"disarmtest\", 1, \"确认向所有僚机发送上锁测试指令？\")' style='background:#6c757d; height:60px; font-size:18px;'>上锁测试</button>";
    html += "</div>";
    html += "<div style='display:flex; gap:10px; margin-bottom:10px;'>";
    html += "<button onclick='sendFleetCommand(\"takeoff\", 1, \"确认向所有僚机发送一键起飞指令？固定翼僚机会忽略该指令。\")' style='background:#ff9800; height:60px; font-size:18px;'>一键起飞僚机</button>";
    html += "<button onclick='sendFleetCommand(\"join\", 1, \"确认让所有僚机尝试加入编队？固定翼僚机需先由飞手手动起飞并切到GUIDED。\")' style='background:#4caf50; height:60px; font-size:18px;'>僚机加入编队</button>";
    html += "</div>";
    html += "<div style='display:flex; gap:10px; margin-bottom:10px;'>";
    html += "<button onclick='sendFleetCommand(\"land\", 1, \"确认让所有僚机执行返航/降落动作？\")' style='background:#795548; height:60px; font-size:18px;'>返航/降落所有僚机</button>";
    html += "</div>";
    html += "<p style='font-size:12px;color:#666;line-height:1.7;'>注意：仅领航机可发送这些指令，且危险操作都会二次确认。解锁测试仅建议在地面拆桨或固定机体时使用，僚机需未失联且处于低高度时才会执行。上锁测试主要用于地面测试，僚机需处于低高度并满足命令间隔后才会执行。一键起飞僚机仅多旋翼会响应，会先尝试切入 GUIDED 并自动解锁起飞；当已开启“允许自动解锁起飞”时，多旋翼僚机会在自动起飞链路中自动加入编队，固定翼会忽略。僚机加入编队时，多旋翼需已起飞且已解锁；固定翼需先手动起飞到安全高度并切到 GUIDED，同时还要通过安全检查后才会生效。返航/降落所有僚机时，多旋翼通常进入 LAND，固定翼进入 RTL 恢复链路。</p>";
    html += "<p id='fleetCmdResult' style='font-size:13px; color:#333; min-height:20px;'></p></div>";

    html += "<div class='card'><h2>我的设置</h2><form id='settingsForm'>";
    html += "昵称: <input name='nickname' value='" + escapeHtmlString(String(_config.MyNikeName)) + "'>";
    html += "模式: <select name='mode' id='modeSelect' onchange='updateLabels()'><option value='1'" + String(_config.AP ? " selected" : "") + ">领航机 (AP)</option><option value='0'" + String(!_config.AP ? " selected" : "") + ">僚机 (STA)</option></select>";
    
    html += "<div id='followerOnlySettings' style='display:none;'>";
    html += "我的编队序号: <select name='order'>";
    for(int i=1; i<=12; i++) {
        html += "<option value='" + String(i) + "'" + String(_config.byPlaneOrder == i ? " selected" : "") + ">" + String(i) + " 号机</option>";
    }
    html += "</select><p style='font-size:12px; color:#666;'>提示：序号决定了你在阵型中的位置。V字/一字阵型下，单号在右，双号在左。</p></div>";

    html += "<span id='ssidLabel'>WiFi名称:</span> <input name='ssid' value='" + escapeHtmlString(String(_config.chSsid)) + "'>";
    html += "<span id='pwdLabel'>WiFi密码:</span> <input name='password' type='text' value='" + escapeHtmlString(String(_config.chPasswd)) + "'>";
    
    html += "<div id='leaderSettings'>";
    html += "编队距离(cm): <input name='dist' type='number' min='" + String(MIN_TEAM_DISTANCE_CM) + "' value='" + String(_config.uTeamDist) + "'>";
    html += "编队高度偏移(cm): <input name='high' type='number' value='" + String(_config.uTeamHigh) + "'>";
    html += "编队类型: <select name='type'>";
    html += "<option value='0'" + String(_config.byTeamType==0 ? " selected" : "") + ">直线跟随</option>";
    html += "<option value='1'" + String(_config.byTeamType==1 ? " selected" : "") + ">V字形</option>";
    html += "<option value='2'" + String(_config.byTeamType==2 ? " selected" : "") + ">一字型</option>";
    html += "<option value='3'" + String(_config.byTeamType==3 ? " selected" : "") + ">梯形编队</option>";
    html += "</select>";
    html += "多旋翼僚机起飞策略: <select name='autoTakeoff'><option value='0'" + String(_config.autoTakeoff==0 ? " selected" : "") + ">手动</option><option value='1'" + String(_config.autoTakeoff==1 ? " selected" : "") + ">允许自动解锁起飞</option></select>";
    html += "多旋翼僚机恢复策略: <select name='autoLand'><option value='0'" + String(_config.autoLand==0 ? " selected" : "") + ">手动</option><option value='1'" + String(_config.autoLand==1 ? " selected" : "") + ">允许自动返航/降落</option></select>";
    html += "</div>";
    html += "<div id='rcSyncSettings'>";
    html += "同步辅助通道: <select name='rcSyncEnabled'><option value='0'" + String(_config.rcSyncEnabled==0 ? " selected" : "") + ">关闭</option><option value='1'" + String(_config.rcSyncEnabled==1 ? " selected" : "") + ">开启</option></select>";
    html += "<div id='leaderRcSyncSettings'>";
    html += "长机源通道: <select name='rcSyncSourceChannel'>";
    for(int i=7; i<=12; i++) {
        html += "<option value='" + String(i) + "'" + String(_config.rcSyncSourceChannel == i ? " selected" : "") + ">CH" + String(i) + "</option>";
    }
    html += "</select></div>";
    html += "<div id='followerRcSyncSettings' style='display:none;'>";
    html += "僚机目标通道: <select name='rcSyncTargetChannel'>";
    for(int i=7; i<=12; i++) {
        html += "<option value='" + String(i) + "'" + String(_config.rcSyncTargetChannel == i ? " selected" : "") + ">CH" + String(i) + "</option>";
    }
    html += "</select></div>";
    html += "<p style='font-size:12px; color:#666;'>用于同步长机遥控器上的辅助开关，仅建议使用 CH7-CH12 这类辅助通道，避免覆盖姿态与油门通道。</p>";
    html += "</div>";
    
    html += "<button type='submit' id='saveBtn'>保存设置</button>";
    html += "<p id='saveHint' style='font-size:12px; color:#666;'>编队距离、队形、序号、辅助通道同步等参数会立即生效。自动策略表示领航机是否向多旋翼僚机广播自动解锁起飞与自动返航/降落许可；开启“允许自动解锁起飞”后，多旋翼僚机会在自动起飞链路里自动加入编队。固定翼僚机仍必须手动起飞并手动切到 GUIDED 后才能加入编队。保存设置会直接生效；只有模式和 WiFi 名称/密码修改后才需要重启。危险指令仍需二次确认。</p>";
    html += "<p id='saveResult' style='font-size:13px; color:#333; min-height:20px;'></p></form></div>";

    html += "<script>";
    html += "function showFleetCmdResult(msg, color) {";
    html += "  var el = document.getElementById('fleetCmdResult');";
    html += "  if(!el) return;";
    html += "  el.innerText = msg || '';";
    html += "  el.style.color = color || '#333';";
    html += "}";
    html += "function sendFleetCommand(cmd, val, msg) {";
    html += "  if(window.confirm(msg || '确认执行该操作？') !== true) return;";
    html += "  var formData = new FormData();";
    html += "  formData.append('cmd', cmd);";
    html += "  formData.append('value', String(val));";
    html += "  fetch('/cmd', { method: 'POST', body: formData })";
    html += "    .then(function(resp){ return resp.json(); })";
    html += "    .then(function(data){ showFleetCmdResult(data.message || '指令已发送。', data.success ? '#2e7d32' : '#d32f2f'); })";
    html += "    .catch(function(){ showFleetCmdResult('指令发送失败，请稍后重试。', '#d32f2f'); });";
    html += "}";
    html += "function updateLabels() {";
    html += "  var m = document.getElementById('modeSelect').value;";
    html += "  if(m == '1') {";
    html += "    document.getElementById('ssidLabel').innerText = '设置热点名称:';";
    html += "    document.getElementById('pwdLabel').innerText = '设置热点密码:';";
    html += "    document.getElementById('leaderSettings').style.display = 'block';";
    html += "    document.getElementById('leaderRcSyncSettings').style.display = 'block';";
    html += "    document.getElementById('realtimeControl').style.display = 'block';";
    html += "    document.getElementById('followerOnlySettings').style.display = 'none';";
    html += "    document.getElementById('followerRcSyncSettings').style.display = 'none';";
    html += "  } else {";
    html += "    document.getElementById('ssidLabel').innerText = '领航机WiFi名称:';";
    html += "    document.getElementById('pwdLabel').innerText = '领航机WiFi密码:';";
    html += "    document.getElementById('leaderSettings').style.display = 'none';";
    html += "    document.getElementById('leaderRcSyncSettings').style.display = 'none';";
    html += "    document.getElementById('realtimeControl').style.display = 'none';";
    html += "    document.getElementById('followerOnlySettings').style.display = 'block';";
    html += "    document.getElementById('followerRcSyncSettings').style.display = 'block';";
    html += "  }";
    html += "}";
    html += "updateLabels();"; // 页面加载时初始化
    html += "function showSaveResult(msg, color) {";
    html += "  var el = document.getElementById('saveResult');";
    html += "  el.innerText = msg || '';";
    html += "  el.style.color = color || '#333';";
    html += "}";
    html += "function setSaveButtonLoading(loading) {";
    html += "  var btn = document.getElementById('saveBtn');";
    html += "  btn.disabled = loading;";
    html += "  btn.innerText = loading ? '保存中...' : '保存设置';";
    html += "}";
    html += "function submitSettings(confirmRestart) {";
    html += "  setSaveButtonLoading(true);";
    html += "  var form = document.getElementById('settingsForm');";
    html += "  var formData = new FormData(form);";
    html += "  if(confirmRestart) formData.append('confirmRestart', '1');";
    html += "  fetch('/save', { method: 'POST', body: formData })";
    html += "    .then(function(resp){ return resp.json(); })";
    html += "    .then(function(data){";
    html += "      if(data.needsRestart && !data.restarting) {";
    html += "        setSaveButtonLoading(false);";
    html += "        if(window.confirm(data.message || '这些修改需要重启，是否继续？')) {";
    html += "          submitSettings(true);";
    html += "        } else {";
    html += "          showSaveResult('已取消需要重启的设置保存。', '#e67e22');";
    html += "        }";
    html += "        return;";
    html += "      }";
    html += "      showSaveResult(data.message || '设置已保存。', data.restarting ? '#e67e22' : '#2e7d32');";
    html += "      if(data.restarting) {";
    html += "        setTimeout(function(){ window.location.href='/'; }, 6000);";
    html += "      } else {";
    html += "        setSaveButtonLoading(false);";
    html += "      }";
    html += "    })";
    html += "    .catch(function(){";
    html += "      showSaveResult('保存失败，请稍后重试。', '#d32f2f');";
    html += "      setSaveButtonLoading(false);";
    html += "    });";
    html += "}";
    html += "document.getElementById('settingsForm').addEventListener('submit', function(e){";
    html += "  e.preventDefault();";
    html += "  submitSettings(false);";
    html += "});";
    html += "function getVehicleTypeName(vehicleType) {";
    html += "  if(vehicleType == 2) return '固定翼 (Plane)';";
    html += "  if(vehicleType == 1) return '多旋翼 (Copter)';";
    html += "  return '未知';";
    html += "}";
    html += "function escapeHtml(str) {";
    html += "  return String(str == null ? '' : str).replace(/[&<>\"']/g, function(ch){";
    html += "    return {'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;',\"'\":'&#39;'}[ch] || ch;";
    html += "  });";
    html += "}";
    html += "function getModeName(modeNum, vehicleType) {";
    html += "  var modes = (vehicleType == 2) ? {";
    html += "    0: 'MANUAL', 1: 'CIRCLE', 2: 'STABILIZE', 3: 'TRAINING', 4: 'ACRO',";
    html += "    5: 'FBWA', 6: 'FBWB', 7: 'CRUISE', 8: 'AUTOTUNE', 10: 'AUTO',";
    html += "    11: 'RTL', 12: 'LOITER', 13: 'TAKEOFF', 14: 'AVOID_ADSB', 15: 'GUIDED',";
    html += "    24: 'THERMAL', 26: 'AUTOLAND'";
    html += "  } : {";
    html += "    0: 'STABILIZE (自稳)', 1: 'ACRO (特技)', 2: 'ALT_HOLD (定高)', 3: 'AUTO (自动)',";
    html += "    4: 'GUIDED (引导伴飞)', 5: 'LOITER (悬停)', 6: 'RTL (返航)', 7: 'CIRCLE (绕圈)',";
    html += "    9: 'LAND (降落)', 11: 'DRIFT (漂移)', 13: 'SPORT (运动)', 14: 'FLIP (翻滚)',";
    html += "    15: 'AUTOTUNE (自动调参)', 16: 'POSHOLD (定点)', 17: 'BRAKE (刹车)',";
    html += "    18: 'THROW (抛飞)', 19: 'AVOID_ADSB (避障)', 20: 'GUIDED_NOGPS', 21: 'SMART_RTL'";
    html += "  };";
    html += "  return modes[modeNum] || ('未知模式 (' + modeNum + ')');";
    html += "}";
    html += "function getCommandName(cmdType) {";
    html += "  var names = {0:'无',1:'解锁测试',2:'上锁测试',3:'返航/降落所有僚机',4:'一键起飞僚机',5:'僚机加入编队'};";
    html += "  return names[cmdType] || ('命令' + cmdType);";
    html += "}";
    html += "function getAckStatusName(status) {";
    html += "  var names = {0:'无反馈',1:'已收到',2:'已下发到飞控',3:'飞控已接受',4:'已完成'};";
    html += "  return names[status] || ('状态' + status);";
    html += "}";
    html += "function getFcAckCommandName(cmd) {";
    html += "  var names = {0:'无',22:'起飞',176:'设置模式',400:'解锁/上锁',21:'返航恢复'};";
    html += "  return names[cmd] || ('MAVCMD ' + cmd);";
    html += "}";
    html += "function getFcAckResultName(result) {";
    html += "  var names = {0:'接受',1:'临时拒绝',2:'拒绝',3:'不支持',4:'失败',5:'执行中',6:'已取消'};";
    html += "  return names[result] || ('结果' + result);";
    html += "}";
    html += "function getRejectReasonName(reason) {";
    html += "  var names = {0:'无',1:'签名校验失败',2:'旧包/回放包',3:'非当前长机来源'};";
    html += "  return names[reason] || ('原因' + reason);";
    html += "}";
    html += "function getActionReasonText(action, reason, d) {";
    html += "  if(d.isAP == 1) return '当前是领航机界面，仅用于查看僚机状态。';";
    html += "  if(action == 'takeoff') {";
    html += "    var names = {1:'条件满足，可执行起飞。',2:'尚未收到稳定的领航机报文。',3:'领航机链路不稳定，等待恢复后再起飞。',4:'解锁/上锁测试冷却中，请稍后再试。',6:'当前不在 GUIDED 模式，已等待切入。',8:'当前已在空中，无需再次起飞。',9:'当前未广播自动起飞许可，请用一键起飞或调整长机策略。',10:'长机尚未满足自动起飞条件。'};";
    html += "    if(reason == 0) return '当前机型不支持此起飞方式。';";
    html += "    return names[reason] || '当前暂未触发起飞。';";
    html += "  }";
    html += "  if(action == 'land') {";
    html += "    var names = {1:'条件满足，可执行返航/降落。',2:'尚未收到稳定的领航机报文。',3:'领航机链路不稳定，等待恢复后再执行返航/降落。',6:'当前不在 GUIDED 模式，已等待切入。',7:'当前未解锁，飞控认为无需返航/降落。',10:'长机尚未满足自动返航/降落条件。',12:'固定翼仅支持手动返航/恢复指令。'};";
    html += "    return names[reason] || '当前暂未触发降落。';";
    html += "  }";
    html += "  var names = {1:'条件满足，可加入编队。',2:'尚未收到稳定的领航机报文。',3:'领航机链路不稳定，等待恢复后再加入。',4:'解锁/上锁测试冷却中，请稍后再试。',5:'当前高度低于安全阈值，不能加入编队。',6:'当前不在 GUIDED 模式，需要先切到 GUIDED。',7:'当前未解锁，不能加入编队。',11:'安全检查未通过，可能是飞控链路、距离或长机高度不满足。',12:'固定翼需手动起飞并切到 GUIDED 后才能加入。',13:'当前没有加入编队请求。'};";
    html += "  return names[reason] || '当前暂未触发加入编队。';";
    html += "}";
    html += "function renderDiagnostics(d) {";
    html += "  document.getElementById('diagRole').innerText = (d.isAP == 1) ? '领航机在线广播中' : ((d.wifi == 1) ? '僚机已联网' : '僚机未联网');";
    html += "  document.getElementById('diagLeaderAge').innerText = (d.isAP == 1) ? '领航机模式不适用' : (d.leaderLinkAgeMs + ' ms');";
    html += "  document.getElementById('diagTxCounter').innerText = d.lastSentPacketCounter;";
    html += "  document.getElementById('diagLeaderCounter').innerText = (d.isAP == 1) ? 'AP 模式不适用' : d.lastLeaderPacketCounter;";
    html += "  if(d.lastFcAckCommand) { document.getElementById('diagFcAck').innerText = getFcAckCommandName(d.lastFcAckCommand) + ' / ' + getFcAckResultName(d.lastFcAckResult) + ' / ' + d.lastFcAckAgeMs + ' ms前'; } else { document.getElementById('diagFcAck').innerText = '暂无'; }";
    html += "  if(d.lastPacketRejectReason) { document.getElementById('diagReject').innerText = getRejectReasonName(d.lastPacketRejectReason) + ' / 包 #' + d.lastRejectedPacketCounter + ' / ' + d.lastPacketRejectAgeMs + ' ms前'; } else { document.getElementById('diagReject').innerText = '暂无'; }";
    html += "  document.getElementById('diagTakeoffReason').innerText = getActionReasonText('takeoff', d.takeoffReason, d);";
    html += "  document.getElementById('diagLandReason').innerText = getActionReasonText('land', d.landReason, d);";
    html += "  document.getElementById('diagJoinReason').innerText = getActionReasonText('join', d.joinReason, d);";
    html += "}";
    html += "function refreshStatus() {";
    html += "  var x = new XMLHttpRequest();";
    html += "  x.onreadystatechange = function() {";
    html += "    if (x.readyState == 4 && x.status == 200) {";
    html += "      try {";
    html += "        var d = JSON.parse(x.responseText);";
    html += "        document.getElementById('lat').innerText = d.lat;";
    html += "        document.getElementById('lon').innerText = d.lon;";
    html += "        document.getElementById('alt').innerText = d.alt;";
    html += "        document.getElementById('rel_alt').innerText = d.rel_alt;";
    html += "        document.getElementById('vehicleType').innerText = getVehicleTypeName(d.vehicleType);";
    html += "        document.getElementById('heading').innerText = d.heading;";
    html += "        document.getElementById('groundSpeed').innerText = d.groundSpeed;";
    html += "        document.getElementById('sat').innerText = d.sat;";
    html += "        document.getElementById('volt').innerText = d.volt;";
    html += "        document.getElementById('armed').innerText = (d.armed == 1) ? '已解锁 (ARMED)' : '锁定 (DISARMED)';";
    html += "        document.getElementById('armed').style.color = (d.armed == 1) ? 'red' : 'green';";
    html += "        document.getElementById('mode').innerText = getModeName(d.mode, d.vehicleType);";
    html += "        document.getElementById('rssi').innerText = d.rssi;";
    html += "        renderDiagnostics(d);";
    html += "        if(d.isAP == 1) {";
    html += "          document.getElementById('ap_status').style.display = 'block';";
    html += "          document.getElementById('sta_status').style.display = 'none';";
    html += "          document.getElementById('staCount').innerText = d.staCount;";
    html += "          document.getElementById('apIP').innerText = d.localIP;";
    html += "          if(d.staMacs && d.staMacs.length > 0) {";
    html += "            document.getElementById('staListDiv').innerHTML = '<strong>连接设备 MAC 列表:</strong><br>' + d.staMacs.join('<br>');";
    html += "          } else {";
    html += "            document.getElementById('staListDiv').innerHTML = '暂无设备连接';";
    html += "          }";
    html += "          if(d.followers && d.followers.length > 0) {";
    html += "            var html = '<strong>僚机详情:</strong><br>';";
    html += "            for(var i = 0; i < d.followers.length; i++) {";
    html += "              var f = d.followers[i];";
    html += "              var ackText = '';";
    html += "              if(f.ackCmdType && f.ackCmdSeq) {";
    html += "                ackText = ' | 最近命令: ' + getCommandName(f.ackCmdType) + ' #' + f.ackCmdSeq + ' - ' + getAckStatusName(f.ackCmdStatus);";
    html += "              }";
    html += "              html += escapeHtml(f.name || '未命名') + ' | ' + getVehicleTypeName(f.vehicleType) + ' | ' + (f.armed == 1 ? '已解锁' : '已上锁') + ' | 高度: ' + f.rel_alt + 'm | 包 #' + f.packetCounter + ' | IP: ' + escapeHtml(f.ip) + ' | MAC: ' + escapeHtml(f.mac) + ackText + '<br>';";
    html += "            }";
    html += "            document.getElementById('followerInfoDiv').innerHTML = html;";
    html += "          } else {";
    html += "            document.getElementById('followerInfoDiv').innerHTML = '暂无僚机详情';";
    html += "          }";
    html += "        } else {";
    html += "          document.getElementById('ap_status').style.display = 'none';";
    html += "          document.getElementById('sta_status').style.display = 'block';";
    html += "          document.getElementById('staIP').innerText = d.localIP;";
    html += "          document.getElementById('gatewayIP').innerText = d.gatewayIP;";
    html += "          document.getElementById('wifiStatus').innerText = (d.wifi == 1) ? '已连接' : '未连接/断开';";
    html += "          document.getElementById('wifiStatus').style.color = (d.wifi == 1) ? 'green' : 'red';";
    html += "        }";
    html += "      } catch(e) {}";
    html += "    }";
    html += "  };";
    html += "  x.open('GET', '/status', true);";
    html += "  x.send();";
    html += "}";
    html += "refreshStatus();";
    html += "setInterval(refreshStatus, 1000);</script>";
    html += "</body></html>";
    return html;
}
