#ifndef FORMATION_H
#define FORMATION_H

#include "g_head.h"
#include <WiFiUdp.h>

// UDP 广播数据包结构
#pragma pack(1)
struct FormationPacket {
    uint8_t sysid;
    uint8_t vehicleType;
    uint8_t mac[6];
    uint8_t ip[4];
    char nickName[20];
    int32_t lat;
    int32_t lon;
    int32_t alt;
    uint16_t heading; // 0-359
    float groundSpeed; // m/s
    uint8_t teamType;
    uint16_t teamDist;
    int16_t teamHigh;
    uint8_t autoTakeoff;
    uint8_t autoLand;
    uint8_t rcSyncEnabled;
    uint8_t rcSyncSourceChannel;
    uint16_t rcSyncValue;
    uint8_t isArmed; // 记录发送方的解锁状态
    uint8_t activeCmdType; // 当前广播中的实时命令类型
    uint16_t activeCmdSeq; // 当前广播中的实时命令序号
    uint8_t reservedCmdFlags[5]; // 兼容保留字段；reservedCmdFlags[0] 当前用于广播长机上锁事件序号
    uint8_t ackCmdType; // 僚机最近确认收到的命令类型
    uint16_t ackCmdSeq; // 僚机最近确认收到的命令序号
    uint8_t ackCmdStatus; // 僚机最近命令的执行状态
    uint8_t gpsHealthy;
    uint8_t ekfHealthy;
    uint32_t packetCounter; // 单调递增计数，用于拒绝旧包/回放包
    uint32_t bootSessionId; // 发送方本次启动会话 ID，用于允许设备重启后计数器重新开始
    uint32_t authTag; // 基于共享密钥的轻量签名
};
#pragma pack()

class Formation {
public:
    Formation(MyConfig &config, MyStatus &status, MyLine *lines);
    void begin();
    void update();
    
    // 获取僚机的目标点
    bool getTargetPoint(int32_t &targetLat, int32_t &targetLon, float &targetAlt);
    
    // 安全检查
    bool isSafetyOk();
    
    // 判断是否满足自动起飞条件
    bool shouldAutoTakeoff(float &takeoffAlt);

    // 判断是否满足跟随长机预解锁条件
    bool shouldAutoArmForTakeoff();
    
    // 判断是否满足自动降落条件
    bool shouldAutoLand();

    // 领航机链路是否已经失效
    bool isLeaderLost();
    unsigned long leaderLinkAgeMs();

    FormationPacket leaderData;
    void broadcastOwnStatus();

private:
    MyConfig &_config;
    MyStatus &_status;
    MyLine *_lines;
    WiFiUDP udp;
    
    unsigned long lastLeaderPacketTime;
    unsigned long leaderTakeoffCandidateSince;
    unsigned long leaderLandingCandidateSince;
    uint32_t lastLocalPacketCounter;
    uint32_t lastLeaderPacketCounter;
    uint32_t localBootSessionId;
    uint32_t lastLeaderBootSessionId;
    bool lastLocalArmedKnown;
    bool lastLocalArmedState;
    uint8_t localDisarmEventSeq;
    void receivePackets();
    void updateFollowerLine(const FormationPacket &packet);
    bool isGpsHealthy() const;
    bool isEkfHealthy() const;
    float clampTargetAlt(float alt) const;
    
    // 地理计算辅助函数
    void offsetGPS(int32_t lat, int32_t lon, float dist_m, float bearing_deg, int32_t &outLat, int32_t &outLon);
    float distanceMeters(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);
    float bearingDeg(int32_t fromLat, int32_t fromLon, int32_t toLat, int32_t toLon);
    bool shouldUsePlaneOrbitFollow() const;
    bool getPlaneOrbitTarget(int32_t &targetLat, int32_t &targetLon, float &targetAlt);
};

#endif
