#ifndef _G_HEAD
#define _G_HEAD
#include <stdint.h>
#include "MAVLink\all\MAVLink.h"
#include <WiFi.h>

// -----------------------------------------------------------
// SHOW_DEBUG_INFO  打开飞控所有的信息通过串口显示在电脑屏幕上
// DEBUG_MODE 调试模式
// BUTTON_MODE 打开按钮功能
// -----------------------------------------------------------

// #define SHOW_DEBUG_INFO
// #define DEBUG_MODE
// #define BUTTON_MODE

#define MAX_UINT64_T  0xFFFFFFFFFFFFFFFF    // uint64_t 的最大值
#define FLIGHT_NUM    9                     // 编队飞机数量,最大9架
#define FORMATION_PORT     5060
// 定义圆周率
#define PI 3.14159265358979323846
#define TOLERANCE 1e-6
#define BAUD_RATE 115200  // 飞控 UART 波特率

#ifdef ARDUINO_ESP32S3_DEV
#define RXPIN 5          // GPIO 5 => RX for Serial1 or Serial1 接飞控的tx1 测试uart1和uart6能正确连接，其他端口无法连接
#define TXPIN 4          // GPIO 4 => TX for Serial1 or Serial1 接飞控的rx1
#define SWITCH1 15       // 重置开关
#define BOARD_BUTTON_PIN 0  // 板载 BOOT 按键
#define LED_PIN 1       // LED
#define LED_PIN2 2      // LED
#define DATA_PIN 10      // ws2812
#define NUM_LEDS 8       // 定义LED灯带的数量
#endif

#ifdef ARDUINO_ESP32C3_DEV
#define RXPIN 5          // GPIO 5 => RX for Serial1 or Serial1 接飞控的tx1 测试uart1和uart6能正确连接，其他端口无法连接
#define TXPIN 4          // GPIO 4 => TX for Serial1 or Serial1 接飞控的rx1
#define SWITCH1 3       // 重置开关
#define BOARD_BUTTON_PIN 9  // 板载 BOOT 按键
#define LED_PIN 12       // LED
#define LED_PIN2 13      // LED
#define DATA_PIN 8      // ws2812
#define NUM_LEDS 1       // 定义LED灯带的数量
#endif

#define MAX_TX_POWER 20
#define MAX_TEAM_DISTANT 200000      // 最大飞行编队距离，如果超过这个距离，系统拒绝编队飞行,200米
#define MIN_TEAM_DISTANCE_CM 100      // GPS/气压计编队的最小水平间距，1米
#define LOWEST_ALTITUDE  3000       // 允许编队最低高度，单位(毫米),3米
#define MAX_FORMATION_TARGET_ALT 120000 // 编队目标相对高度上限，单位(毫米),120米
#define MIN_GPS_FIX_TYPE 3           // 最低要求 3D Fix
#define MIN_GPS_SATELLITES 10        // 最低卫星数量
#define MAX_GPS_EPH 150              // GPS HDOP 阈值，MAVLink eph 通常为 HDOP*100，150=1.5
#define MAX_GPS_EPV 250              // GPS VDOP 阈值，MAVLink epv 通常为 VDOP*100，250=2.5
#define SENSOR_TIMEOUT_MS 2000       // 关键飞控传感器数据最大允许年龄
#define EKF_MAX_VEL_VARIANCE 0.5f    // EKF 速度估计允许的最大方差
#define EKF_MAX_POS_VARIANCE 0.5f    // EKF 水平位置估计允许的最大方差
#define EKF_STABLE_REQUIRED_MS 5000  // EKF 需持续健康 5 秒才允许进入编队
#define MAX_LOST_WAIT_TIME  12000   // 失联后最大等待时间，单位(毫秒),12秒
#define HOLD_LOST_WAIT_TIME 3000    // 中短时失联后进入安全过渡动作，单位(毫秒),3秒
#define WIFI_RECONNECT_INTERVAL 3000 // WiFi 主动重连间隔，单位(毫秒),3秒
#define WIFI_CONNECT_WINDOW_MS 12000 // WiFi 每次发起连接后的等待窗口，单位(毫秒),12秒
#define COPTER_AUTO_TAKEOFF_MAX_ALT 4500 // 多旋翼自动起飞的最大温和起飞高度，单位(毫米),4.5米
#define COPTER_AUTO_JOIN_HOVER_MS 8000 // 多旋翼自动起飞后进入编队前的稳定悬停时间，单位(毫秒),8秒
#define COPTER_JOIN_APPROACH_MS 5000 // 多旋翼进入编队初期的渐进收敛时间，单位(毫秒),5秒
#define COPTER_FOLLOW_MAX_STEP_M 0.8f // 多旋翼每次更新允许的最大水平目标位移，单位(m)
#define COPTER_FOLLOW_MAX_ASCENT_STEP_M 0.35f // 多旋翼每次更新允许的最大目标升高量，单位(m)
#define COPTER_FOLLOW_MAX_DESCENT_STEP_M 0.15f // 多旋翼每次更新允许的最大目标下降量，单位(m)
#define COPTER_FOLLOW_ALT_FLOOR_MARGIN_M 0.5f // 多旋翼跟随过程中目标高度不低于当前高度下方的安全余量，单位(m)
#define COPTER_EMERGENCY_DISARM_ALT_MM 250 // 多旋翼异常贴地/翻覆时允许触发停桨兜底的高度阈值，单位(毫米),0.25米
#define COPTER_EMERGENCY_DISARM_GS_MPS 1.5f // 多旋翼异常贴地/翻覆时的低地速阈值，单位(m/s)
#define COPTER_EMERGENCY_DISARM_ATT_RAD 1.05f // 多旋翼异常贴地/翻覆时的姿态阈值，单位(弧度),约60度
#define COPTER_EMERGENCY_DISARM_HOLD_MS 1500 // 多旋翼异常贴地/翻覆状态需持续的确认时间，单位(毫秒),1.5秒
#define LEADER_TAKEOFF_CONFIRM_MS 2000 // 长机离地后自动起飞确认时间，单位(毫秒),2秒
#define LEADER_LAND_ALT_THRESHOLD 1200 // 判定长机已接地的高度阈值，单位(毫米),1.2米
#define PLANE_LEADER_LAND_SPEED_THRESHOLD 6.0f // 固定翼长机落地确认速度阈值，单位 m/s
#define LEADER_LAND_CONFIRM_MS 1500 // 长机落地状态需持续确认时间，单位(毫秒),1.5秒
#define COMMAND_BROADCAST_WINDOW_MS 2000 // 实时控制命令的持续广播时间，单位(毫秒),2秒
#define PLANE_GUIDE_LEAD_TIME 1.5f  // 固定翼引导前视时间，单位秒
#define PLANE_LOW_SPEED_LEADER 5.0f // 低速长机阈值，单位 m/s
#define PLANE_ORBIT_MIN_RADIUS 35.0f // 固定翼绕点伴飞的最小半径，单位 m
#define PLANE_ORBIT_STEP_RADIUS 12.0f // 多架固定翼分层绕飞半径步进，单位 m
#define PLANE_ORBIT_ADVANCE_DEG 35.0f // 固定翼绕飞时沿圆周前视角，单位 deg

enum VehicleType : uint8_t {
    VEHICLE_TYPE_UNKNOWN = 0,
    VEHICLE_TYPE_COPTER = 1,
    VEHICLE_TYPE_PLANE = 2
};

enum RealtimeCommandType : uint8_t {
    CMD_NONE = 0,
    CMD_ARM_TEST = 1,
    CMD_DISARM_TEST = 2,
    CMD_LAND_ALL = 3,
    CMD_TAKEOFF_ALL = 4,
    CMD_JOIN_ALL = 5
};

enum RealtimeCommandAckStatus : uint8_t {
    CMD_ACK_NONE = 0,
    CMD_ACK_RECEIVED = 1,
    CMD_ACK_SENT_TO_FC = 2,
    CMD_ACK_FC_ACCEPTED = 3,
    CMD_ACK_COMPLETED = 4
};

enum PacketRejectReason : uint8_t {
    PACKET_REJECT_NONE = 0,
    PACKET_REJECT_BAD_AUTH = 1,
    PACKET_REJECT_REPLAY = 2,
    PACKET_REJECT_BAD_SOURCE = 3
};

enum ActionReasonCode : uint8_t {
    ACTION_REASON_NA = 0,
    ACTION_REASON_READY = 1,
    ACTION_REASON_WAIT_LEADER = 2,
    ACTION_REASON_LINK_UNSTABLE = 3,
    ACTION_REASON_ARM_TEST_COOLDOWN = 4,
    ACTION_REASON_ALT_TOO_LOW = 5,
    ACTION_REASON_NOT_GUIDED = 6,
    ACTION_REASON_NOT_ARMED = 7,
    ACTION_REASON_ALREADY_AIRBORNE = 8,
    ACTION_REASON_AUTO_DISABLED = 9,
    ACTION_REASON_LEADER_NOT_READY = 10,
    ACTION_REASON_SAFETY_BLOCKED = 11,
    ACTION_REASON_MANUAL_PLANE_ONLY = 12,
    ACTION_REASON_NOT_REQUESTED = 13
};

// 启用 1 字节对齐
#pragma pack(1)
//---------------------------------------------------------------------------
typedef struct tag_MyConfig
{
    uint8_t AP;            // 长机
    char chSsid[50];       // 编队长机使用的ssid
    char chPasswd[50];     // 编队长机使用的密码
    uint8_t byPlayerCount; // 玩家飞机数量,目前仅支持10架编队
    char MyNikeName[20];   // 玩家飞机的名称
    uint16_t uTeamDist;    // 编队距离
    int uTeamHigh;         // 编队高度
    uint8_t byTeamType;    // 编队类型
    uint8_t ControlMeByLeader;  // 允许我被长机遥控 
    uint8_t autoTakeoff;   // 长机是否广播允许多旋翼僚机自动解锁起飞
    uint8_t autoLand;      // 长机是否广播允许多旋翼僚机自动返航/降落
    uint8_t byPlaneOrder;  // 僚机在编队中的序号 (1, 2, 3...)
    uint8_t rcSyncEnabled; // 是否启用单通道同步
    uint8_t rcSyncSourceChannel; // 长机读取的遥控通道号
    uint8_t rcSyncTargetChannel; // 僚机写入飞控的目标通道号
    uint8_t reservedRealtimeCmdFlags[5]; // 兼容旧配置布局，当前逻辑不再使用
} MyConfig;
//---------------------------------------------------------------------------
typedef struct tag_MyStatus
{
    mavlink_gps_raw_int_t gpsData; // gps信息
    mavlink_vfr_hud_t hudData;     // 飞行姿态
    mavlink_attitude_t attitude;   // 飞机姿态
    mavlink_heartbeat_t heartbeat; // 心跳包
    mavlink_global_position_int_t pos;  //全球位置信息
    mavlink_system_time_t system_time;  //系统时间
    mavlink_sys_status_t sysStatus;
    mavlink_autopilot_version_t autopilot_version;  //飞控版本
    mavlink_rc_channels_t rc_channels;  // 遥控器通道信息
    uint8_t byPlaneOrder;          // 编队飞机序号，长机0 僚机1 僚机2
    uint8_t byWifi;                // 飞机之间的WIFI状态 0:断网 1:联网
    uint8_t vehicleType;           // 当前飞控机型：未知/多旋翼/固定翼
    uint8_t mac[6];                // 网卡地址
    float groundSpeed;             // 地速，单位 m/s
    float airSpeed;                // 空速，单位 m/s
    uint8_t activeCmdType;         // 长机当前正在广播的实时命令类型
    uint16_t activeCmdSeq;         // 长机当前实时命令序号
    unsigned long activeCmdUntil;  // 长机当前实时命令持续广播截止时间
    uint16_t lastIssuedCmdSeq;     // 长机最近发出的命令序号
    uint8_t lastCmdAckType;        // 僚机最近确认收到的命令类型
    uint16_t lastCmdAckSeq;        // 僚机最近确认收到的命令序号
    uint8_t lastCmdAckStatus;      // 僚机最近命令的执行状态
    uint16_t lastFcAckCommand;     // 飞控最近确认的 MAVLink 命令号
    uint8_t lastFcAckResult;       // 飞控最近一次 COMMAND_ACK 结果
    unsigned long lastFcAckTime;   // 飞控最近一次 COMMAND_ACK 时间
    uint32_t lastSentPacketCounter; // 本机最近一次发出的编队报文计数
    uint32_t lastLeaderPacketCounter; // 最近一次接受的领航机报文计数
    uint8_t lastPacketRejectReason; // 最近一次拒收报文原因
    uint32_t lastRejectedPacketCounter; // 最近一次拒收报文的计数
    unsigned long lastPacketRejectTime; // 最近一次拒收报文时间
    uint8_t takeoffReason;          // 当前起飞未生效/可生效原因
    uint8_t landReason;             // 当前降落未生效/可生效原因
    uint8_t joinReason;             // 当前加入编队未生效/可生效原因
    unsigned long lastFcTime;      // 最后收到飞控发来数据的时间 
    unsigned long lastWifiTime;    // 最后收到wifi的时间
    unsigned long lastGpsTime;      // 最后收到 GPS_RAW_INT 的时间
    unsigned long lastPosTime;      // 最后收到 GLOBAL_POSITION_INT 的时间
    unsigned long lastEkfTime;      // 最后收到 EKF 状态的时间
    uint16_t ekfFlags;              // EKF_STATUS_REPORT flags
    float ekf_vel_variance;         // EKF 速度方差
    float ekf_pos_horiz_variance;   // EKF 水平位置方差
    unsigned long ekfHealthySince;  // EKF 开始健康的时间点
} MyStatus;
//---------------------------------------------------------------------------
//飞机编队信息
typedef struct tag_MyLine
{
  uint8_t useage;       // 是否使用
  uint8_t mac[6];       // 编队飞机的网卡地址
  uint8_t ip[4];        // 编队飞机的局域网IP
  uint8_t vehicleType;  // 编队飞机机型
  uint8_t isArmed;      // 编队飞机当前是否解锁
  int32_t rel_alt;      // 编队飞机相对高度，单位毫米
  uint8_t ackCmdType;   // 最近一次收到的实时命令类型
  uint16_t ackCmdSeq;   // 最近一次收到的实时命令序号
  uint8_t ackCmdStatus; // 最近一次命令的执行状态
  uint32_t lastPacketCounter; // 最近一次收到的报文计数
  uint32_t bootSessionId;     // 发送方本次启动会话 ID，用于识别重启后的计数器回绕
  char chNikeName[20];  // 飞机操作者的小名
  unsigned long line_time;   // 最后一次收到定位消息的时间戳
}MyLine;
// 恢复默认对齐
#pragma pack()

#endif
