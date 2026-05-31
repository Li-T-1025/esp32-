#include "ConfigStorage.h"
#include <WiFi.h>

ConfigStorage::ConfigStorage() {}

void ConfigStorage::begin() {
    preferences.begin("formation", false);
}

void ConfigStorage::loadConfig(MyConfig &config) {
    if (preferences.getBytes("config", &config, sizeof(MyConfig)) != sizeof(MyConfig)) {
        resetConfig(config);
        return;
    }

    bool changed = false;
    if (config.uTeamDist < MIN_TEAM_DISTANCE_CM) {
        config.uTeamDist = MIN_TEAM_DISTANCE_CM;
        changed = true;
    }
    if (config.uTeamDist > MAX_TEAM_DISTANT / 10) {
        config.uTeamDist = MAX_TEAM_DISTANT / 10;
        changed = true;
    }
    if (config.uTeamHigh < -5000) {
        config.uTeamHigh = -5000;
        changed = true;
    }
    if (config.uTeamHigh > 5000) {
        config.uTeamHigh = 5000;
        changed = true;
    }
    if (changed) saveConfig(config);
}

void ConfigStorage::saveConfig(const MyConfig &config) {
    preferences.putBytes("config", &config, sizeof(MyConfig));
}

void ConfigStorage::resetConfig(MyConfig &config) {
    memset(&config, 0, sizeof(MyConfig));
    
    config.AP = 1; // 默认长机模式
    
    // 在获取 MAC 地址前，必须先初始化 WiFi，否则 mac 数组可能是 00:00:00:00:00:00
    WiFi.mode(WIFI_MODE_STA);
    
    // 生成默认 SSID: IdeaTeamFly + MAC 后四位
    uint8_t mac[6];
    WiFi.macAddress(mac);
    sprintf(config.chSsid, "IdeaTeamFly%02X%02X", mac[4], mac[5]);
    
    strcpy(config.chPasswd, "12345678");
    strcpy(config.MyNikeName, "BishanLeader");
    
    config.uTeamDist = MIN_TEAM_DISTANCE_CM; // GPS/气压计编队默认 1m 间距
    config.uTeamHigh = 0;   // 同高
    config.byTeamType = 0;  // 默认直线/跟随
    config.byPlayerCount = 2;
    config.ControlMeByLeader = 1;
    config.autoTakeoff = 0; // 默认安全，手动解锁起飞
    config.autoLand = 0;    // 默认安全，手动降落上锁
    config.byPlaneOrder = 1; // 默认是 1 号僚机
    config.rcSyncEnabled = 0;
    config.rcSyncSourceChannel = 7;
    config.rcSyncTargetChannel = 7;
    
    saveConfig(config);
}
