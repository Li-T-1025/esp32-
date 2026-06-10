#include "MavlinkHandler.h"

const char* MavlinkHandler::commandName(uint16_t command) {
    switch (command) {
        case MAV_CMD_COMPONENT_ARM_DISARM: return "ARM_DISARM";
        case MAV_CMD_NAV_TAKEOFF: return "NAV_TAKEOFF";
        case MAV_CMD_DO_SET_MODE: return "DO_SET_MODE";
        case MAV_CMD_NAV_LOITER_UNLIM: return "NAV_LOITER_UNLIM";
        case MAV_CMD_NAV_RETURN_TO_LAUNCH: return "NAV_RTL";
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: return "RC_CHANNELS_OVERRIDE";
        default: return "UNKNOWN_CMD";
    }
}

const char* MavlinkHandler::ackResultName(uint8_t result) {
    switch (result) {
        case MAV_RESULT_ACCEPTED: return "ACCEPTED";
        case MAV_RESULT_TEMPORARILY_REJECTED: return "TEMP_REJECTED";
        case MAV_RESULT_DENIED: return "DENIED";
        case MAV_RESULT_UNSUPPORTED: return "UNSUPPORTED";
        case MAV_RESULT_FAILED: return "FAILED";
        case MAV_RESULT_IN_PROGRESS: return "IN_PROGRESS";
        case MAV_RESULT_CANCELLED: return "CANCELLED";
        default: return "UNKNOWN_RESULT";
    }
}

void MavlinkHandler::logCommandSend(uint16_t command, const char* detail) {
    Serial.printf("[CMD->FC] %s(%u)", commandName(command), command);
    if (detail && detail[0] != '\0') {
        Serial.printf(" | %s", detail);
    }
    Serial.println();
}

void MavlinkHandler::logCommandAck(const mavlink_command_ack_t &ack, uint8_t srcSys, uint8_t srcComp) {
    Serial.printf(
        "[ACK<-FC] %s(%u) | result=%s(%u) | progress=%u | result_param2=%d | from=%u/%u\n",
        commandName(ack.command),
        ack.command,
        ackResultName(ack.result),
        ack.result,
        ack.progress,
        ack.result_param2,
        srcSys,
        srcComp
    );
}

MavlinkHandler::MavlinkHandler(MyStatus &status) : _status(status), mavSerial(Serial1) {
    mavlink_system.sysid = 255; // 伴飞计算机/地面站常用 ID，避免与飞控(ID 1)冲突
    mavlink_system.compid = MAV_COMP_ID_MISSIONPLANNER;
}

uint8_t MavlinkHandler::detectVehicleType(uint8_t heartbeatType) const {
    switch (heartbeatType) {
        case MAV_TYPE_FIXED_WING:
            return VEHICLE_TYPE_PLANE;
        case MAV_TYPE_QUADROTOR:
        case MAV_TYPE_COAXIAL:
        case MAV_TYPE_HELICOPTER:
        case MAV_TYPE_HEXAROTOR:
        case MAV_TYPE_OCTOROTOR:
        case MAV_TYPE_TRICOPTER:
        case MAV_TYPE_DODECAROTOR:
            return VEHICLE_TYPE_COPTER;
        default:
            return VEHICLE_TYPE_UNKNOWN;
    }
}

void MavlinkHandler::begin(unsigned long baud, int rx, int tx) {
    mavSerial.begin(baud, SERIAL_8N1, rx, tx);
}

void MavlinkHandler::handle() {
    mavlink_message_t msg;
    mavlink_status_t status;

    while (mavSerial.available()) {
        uint8_t c = mavSerial.read();

        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            parseMessage(msg);
        }
    }
}

void MavlinkHandler::parseMessage(mavlink_message_t &msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            mavlink_msg_heartbeat_decode(&msg, &_status.heartbeat);
            _status.vehicleType = detectVehicleType(_status.heartbeat.type);
            _status.lastFcTime = millis();
            break;
        case MAVLINK_MSG_ID_GPS_RAW_INT:
            mavlink_msg_gps_raw_int_decode(&msg, &_status.gpsData);
            _status.lastGpsTime = millis();
            break;
        case MAVLINK_MSG_ID_VFR_HUD:
            mavlink_msg_vfr_hud_decode(&msg, &_status.hudData);
            _status.groundSpeed = _status.hudData.groundspeed;
            _status.airSpeed = _status.hudData.airspeed;
            break;
        case MAVLINK_MSG_ID_ATTITUDE:
            mavlink_msg_attitude_decode(&msg, &_status.attitude);
            break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            mavlink_msg_global_position_int_decode(&msg, &_status.pos);
            _status.lastPosTime = millis();
            break;
        case MAVLINK_MSG_ID_SYS_STATUS:
            mavlink_msg_sys_status_decode(&msg, &_status.sysStatus);
            break;
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
            mavlink_rc_channels_raw_t rc_raw;
            mavlink_msg_rc_channels_raw_decode(&msg, &rc_raw);
            _status.rc_channels.rssi = rc_raw.rssi;
            break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS:
            mavlink_msg_rc_channels_decode(&msg, &_status.rc_channels);
            break;
        case MAVLINK_MSG_ID_COMMAND_ACK: {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&msg, &ack);
            _status.lastFcAckCommand = ack.command;
            _status.lastFcAckResult = ack.result;
            _status.lastFcAckTime = millis();
            logCommandAck(ack, msg.sysid, msg.compid);
            break;
        }
        case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
            mavlink_ekf_status_report_t ekf;
            mavlink_msg_ekf_status_report_decode(&msg, &ekf);
            _status.ekfFlags = ekf.flags;
            _status.ekf_vel_variance = ekf.velocity_variance;
            _status.ekf_pos_horiz_variance = ekf.pos_horiz_variance;
            _status.lastEkfTime = millis();
            break;
        }
    }
}

void MavlinkHandler::sendHeartbeat() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                               MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavSerial.write(buf, len);
}

void MavlinkHandler::requestDataStreams() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // 请求所有数据流 @ 5Hz (发给飞控系统ID 1, 组件ID 1)
    mavlink_msg_request_data_stream_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                                         1, 1, MAV_DATA_STREAM_ALL, 5, 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavSerial.write(buf, len);

    auto requestMessageInterval = [&](uint32_t messageId, uint32_t intervalUs) {
        mavlink_msg_command_long_pack(
            mavlink_system.sysid, mavlink_system.compid, &msg,
            1, 1,
            MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            messageId,
            intervalUs,
            0, 0, 0, 0, 0
        );
        uint16_t cmdLen = mavlink_msg_to_send_buffer(buf, &msg);
        mavSerial.write(buf, cmdLen);
    };

    requestMessageInterval(MAVLINK_MSG_ID_GPS_RAW_INT, 200000);
    requestMessageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000);
    requestMessageInterval(MAVLINK_MSG_ID_EKF_STATUS_REPORT, 200000);
}

void MavlinkHandler::sendGuidedTarget(int32_t lat, int32_t lon, float alt) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // MAVLink 期望的高度单位是米 (m)
    // alt 的单位在 Formation.cpp 计算完传过来时已经是米 (m) 了，所以不需要再除以 1000
    
    // 使用 SET_POSITION_TARGET_GLOBAL_INT 发送目标点
    // 坐标系改为 MAV_FRAME_GLOBAL_RELATIVE_ALT_INT (相对于起飞点高度)，因为我们现在广播和接收的都是 relative_alt
    mavlink_msg_set_position_target_global_int_pack(
        mavlink_system.sysid, mavlink_system.compid, &msg,
        millis(), 1, 0, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000, // 仅位置有效 (X, Y, Z)
        lat, lon, alt, // 这里的 alt 单位必须是米
        0, 0, 0, // 速度无效
        0, 0, 0, // 加速度无效
        0, 0     // 偏航无效
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavSerial.write(buf, len);
}

void MavlinkHandler::sendGuidedWaypoint(int32_t lat, int32_t lon, float alt) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // ArduPlane 的 guided goto 建议通过 MISSION_ITEM_INT + current=2 下发。
    mavlink_msg_mission_item_int_pack(
        mavlink_system.sysid, mavlink_system.compid, &msg,
        1, 1, 0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT,
        2, 0,
        0, 0, 0, 0,
        lat, lon, alt,
        MAV_MISSION_TYPE_MISSION
    );

    char detail[96];
    snprintf(detail, sizeof(detail), "guided_wp lat=%ld lon=%ld alt=%.2fm",
             (long)lat, (long)lon, alt);
    logCommandSend(MAV_CMD_NAV_WAYPOINT, detail);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavSerial.write(buf, len);
}

void MavlinkHandler::sendFollowTarget(int32_t lat, int32_t lon, float alt) {
    if (isPlane()) {
        sendGuidedWaypoint(lat, lon, alt);
    } else {
        sendGuidedTarget(lat, lon, alt);
    }
}

void MavlinkHandler::sendArmCommand() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // 发送 MAV_CMD_COMPONENT_ARM_DISARM (400) 指令
    // param 1: 1.0 (Arm), 0.0 (Disarm)
    // param 2: 0.0，走飞控正常安全检查
    mavlink_msg_command_long_pack(
        mavlink_system.sysid, mavlink_system.compid, &msg,
        1, 1, // target system 1 (飞控), target component 1
        MAV_CMD_COMPONENT_ARM_DISARM,
        0, // confirmation
        1.0f, // param 1: 1.0 代表解锁
        0.0f, // param 2: 正常解锁，不强制跳过安全检查
        0, 0, 0, 0, 0 // param 3-7 留空
    );

    logCommandSend(MAV_CMD_COMPONENT_ARM_DISARM, "param1=1 arm");
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavSerial.write(buf, len);
}

void MavlinkHandler::sendDisarmCommand() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(
        mavlink_system.sysid, mavlink_system.compid, &msg,
        1, 1,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0.0f, // param 1: 0.0 代表上锁
        0.0f, // param 2: 正常上锁，不使用强制参数
        0, 0, 0, 0, 0
    );

    logCommandSend(MAV_CMD_COMPONENT_ARM_DISARM, "param1=0 disarm");
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavSerial.write(buf, len);
}

void MavlinkHandler::sendTakeoff(float alt) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // 发送 MAV_CMD_NAV_TAKEOFF (22) 指令
    // 修正：ArduPilot 4.5+ 推荐在 GUIDED 模式下，param 5, 6 设为 0 或 NaN 代表当前经纬度
    // param 7 对应起飞的目标相对高度 (米)
    mavlink_msg_command_long_pack(
        mavlink_system.sysid, mavlink_system.compid, &msg,
        1, 1, // target system 1, target component 1
        MAV_CMD_NAV_TAKEOFF,
        0, // confirmation
        0, 0, 0, 0, 0, 0, // param 1-6
        alt // param 7: 起飞高度
    );

    char detail[64];
    snprintf(detail, sizeof(detail), "target_rel_alt=%.2fm", alt);
    logCommandSend(MAV_CMD_NAV_TAKEOFF, detail);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavSerial.write(buf, len);
    
    // 删除了原有的 sendGuidedTarget 调用，避免在地面触发模式冲突
}

void MavlinkHandler::sendSetMode(uint32_t custom_mode) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // 发送 MAV_CMD_DO_SET_MODE (176) 指令
    // 在 ArduCopter 中，设置 custom_mode 时，base_mode 必须包含 MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1)
    mavlink_msg_command_long_pack(
        mavlink_system.sysid, mavlink_system.compid, &msg,
        1, 1, // target system 1, target component 1
        MAV_CMD_DO_SET_MODE,
        0, // confirmation
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // param 1: base_mode 标识
        custom_mode, // param 2: custom_mode 模式编号 (ArduCopter LAND=9)
        0, 0, 0, 0, 0 // param 3-7
    );

    char detail[64];
    snprintf(detail, sizeof(detail), "custom_mode=%lu", (unsigned long)custom_mode);
    logCommandSend(MAV_CMD_DO_SET_MODE, detail);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavSerial.write(buf, len);
}

void MavlinkHandler::sendGuidedMode() {
    sendSetMode(guidedModeNumber());
}

void MavlinkHandler::sendRecoveryMode() {
    if (isPlane()) {
        sendRTLMode();
    } else {
        sendSetMode(recoveryModeNumber());
    }
}

void MavlinkHandler::sendLoiterMode() {
    sendSetMode(loiterModeNumber());
}

void MavlinkHandler::sendRTLMode() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(
        mavlink_system.sysid, mavlink_system.compid, &msg,
        1, 1,
        MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,
        0, 0, 0, 0, 0, 0, 0
    );
    logCommandSend(MAV_CMD_NAV_RETURN_TO_LAUNCH, "rtl");
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavSerial.write(buf, len);
}

uint16_t MavlinkHandler::getRcChannelValue(uint8_t channel) const {
    switch (channel) {
        case 1: return _status.rc_channels.chan1_raw;
        case 2: return _status.rc_channels.chan2_raw;
        case 3: return _status.rc_channels.chan3_raw;
        case 4: return _status.rc_channels.chan4_raw;
        case 5: return _status.rc_channels.chan5_raw;
        case 6: return _status.rc_channels.chan6_raw;
        case 7: return _status.rc_channels.chan7_raw;
        case 8: return _status.rc_channels.chan8_raw;
        case 9: return _status.rc_channels.chan9_raw;
        case 10: return _status.rc_channels.chan10_raw;
        case 11: return _status.rc_channels.chan11_raw;
        case 12: return _status.rc_channels.chan12_raw;
        case 13: return _status.rc_channels.chan13_raw;
        case 14: return _status.rc_channels.chan14_raw;
        case 15: return _status.rc_channels.chan15_raw;
        case 16: return _status.rc_channels.chan16_raw;
        case 17: return _status.rc_channels.chan17_raw;
        case 18: return _status.rc_channels.chan18_raw;
        default: return 0;
    }
}

void MavlinkHandler::sendRcChannelOverride(uint8_t channel, uint16_t pwm) {
    if (channel < 1 || channel > 18) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t channels[18];
    for (int i = 0; i < 18; i++) {
        channels[i] = (i < 8) ? UINT16_MAX : 0;
    }
    channels[channel - 1] = pwm;

    mavlink_msg_rc_channels_override_pack(
        mavlink_system.sysid, mavlink_system.compid, &msg,
        1, 1,
        channels[0], channels[1], channels[2], channels[3], channels[4], channels[5], channels[6], channels[7],
        channels[8], channels[9], channels[10], channels[11], channels[12], channels[13], channels[14], channels[15],
        channels[16], channels[17]
    );

    char detail[64];
    snprintf(detail, sizeof(detail), "channel=%u pwm=%u", channel, pwm);
    logCommandSend(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, detail);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavSerial.write(buf, len);
}

void MavlinkHandler::sendRcChannelRelease(uint8_t channel) {
    if (channel < 1 || channel > 18) return;

    uint16_t releaseValue = (channel <= 8) ? 0 : (UINT16_MAX - 1);
    sendRcChannelOverride(channel, releaseValue);
}

bool MavlinkHandler::isCopter() const {
    return _status.vehicleType == VEHICLE_TYPE_COPTER;
}

bool MavlinkHandler::isPlane() const {
    return _status.vehicleType == VEHICLE_TYPE_PLANE;
}

bool MavlinkHandler::isGuidedMode() const {
    return _status.heartbeat.custom_mode == guidedModeNumber();
}

uint32_t MavlinkHandler::guidedModeNumber() const {
    return isPlane() ? 15 : 4;
}

uint32_t MavlinkHandler::loiterModeNumber() const {
    return isPlane() ? 12 : 5;
}

uint32_t MavlinkHandler::rtlModeNumber() const {
    return isPlane() ? 11 : 6;
}

uint32_t MavlinkHandler::recoveryModeNumber() const {
    return isPlane() ? rtlModeNumber() : 9;
}
