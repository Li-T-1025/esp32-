#ifndef MAVLINK_HANDLER_H
#define MAVLINK_HANDLER_H

#include "g_head.h"

class MavlinkHandler {
public:
    MavlinkHandler(MyStatus &status);
    void begin(unsigned long baud, int rx, int tx);
    void handle();
    void sendHeartbeat();
    void requestDataStreams();
    
    // 发送位置控制指令 (Guided 模式)
    void sendGuidedTarget(int32_t lat, int32_t lon, float alt);
    void sendGuidedWaypoint(int32_t lat, int32_t lon, float alt);
    void sendFollowTarget(int32_t lat, int32_t lon, float alt);
    
    // 发送自动起飞指令
    void sendTakeoff(float alt);
    
    // 发送解锁指令
    void sendArmCommand();
    void sendDisarmCommand();
    
    // 发送更改飞行模式指令 (多旋翼 LAND / 固定翼 RTL)
    void sendSetMode(uint32_t custom_mode);
    void sendGuidedMode();
    void sendRecoveryMode();
    void sendLoiterMode();
    void sendRTLMode();
    void sendRcChannelOverride(uint8_t channel, uint16_t pwm);
    void sendRcChannelRelease(uint8_t channel);
    
    bool isCopter() const;
    bool isPlane() const;
    bool isGuidedMode() const;
    uint32_t guidedModeNumber() const;
    uint32_t loiterModeNumber() const;
    uint32_t rtlModeNumber() const;
    uint32_t recoveryModeNumber() const;
    uint16_t getRcChannelValue(uint8_t channel) const;

private:
    MyStatus &_status;
    HardwareSerial &mavSerial;
    mavlink_system_t mavlink_system;

    void parseMessage(mavlink_message_t &msg);
    uint8_t detectVehicleType(uint8_t heartbeatType) const;
    const char* commandName(uint16_t command);
    const char* ackResultName(uint8_t result);
    void logCommandSend(uint16_t command, const char* detail = nullptr);
    void logCommandAck(const mavlink_command_ack_t &ack, uint8_t srcSys, uint8_t srcComp);
};

#endif
