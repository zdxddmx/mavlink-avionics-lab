/*
 * mavlink.h — MAVLink v2 航电链路库（解包 / 打包 / 链路状态机）
 *
 * 面向无人机航电岗的「飞控系统软件」与「飞控与载荷接口软件」：
 *   · 帧编解码：STX + LEN + 兼容标志 + 序号 + 系统/组件 ID + 24 位消息 ID + 载荷 + 校验
 *   · 校验：CRC-16/MCRF4XX，并且要**逐字节累积 CRC_EXTRA**（MAVLink 特有的坑）
 *   · 流式解析：逐字节喂入，能处理粘包、半包、垃圾数据后重新同步
 *   · 链路状态机：心跳超时判定、解锁/上锁、任务模式、命令应答匹配
 *
 * 纯 C99，零第三方依赖。所有细节都可用单元测试覆盖，不需要真机。
 *
 * 注意：本项目是**按 MAVLink v2 规范自行实现的收发与链路管理**，
 *      不依赖官方 mavlink 生成器；消息集只覆盖航电链路常用的几条。
 */
#ifndef MAVLINK_H
#define MAVLINK_H

#include <stdint.h>
#include <stddef.h>

/* --------------------------------------------------------------- 协议常量 */
#define MAV_STX_V2        0xFD
#define MAV_STX_V1        0xFE
#define MAV_HEADER_V2     10      /* STX..MSGID（不含 STX 计 9，含 STX 计 10） */
#define MAV_HEADER_V1     6
#define MAV_MAX_PAYLOAD   255
#define MAV_MAX_FRAME     (MAV_HEADER_V2 + MAV_MAX_PAYLOAD + 2)

/* 常用消息 ID */
#define MAV_MSG_HEARTBEAT       0
#define MAV_MSG_SYS_STATUS      1
#define MAV_MSG_PING            4
#define MAV_MSG_ATTITUDE        30
#define MAV_MSG_GLOBAL_POSITION 33
#define MAV_MSG_GPS_RAW_INT     24
#define MAV_MSG_COMMAND_LONG    76
#define MAV_MSG_COMMAND_ACK     77
#define MAV_MSG_STATUSTEXT      253

/* 消息的 CRC_EXTRA（MAVLink 规范表；错一个字节整条链路就通不了） */
uint8_t mav_crc_extra(uint32_t msgid);

/* --------------------------------------------------------------- 错误码 */
typedef enum {
    MAV_OK = 0,
    MAV_ERR_PARAM = -1,
    MAV_ERR_NOMEM = -2,
    MAV_ERR_CRC = -3,
    MAV_ERR_UNSUPPORTED = -4,
    MAV_ERR_STATE = -5,
} mav_status_t;

const char *mav_status_str(mav_status_t s);

/* ------------------------------------------------------------------ 帧 */
typedef struct {
    uint8_t  magic;             /* 0xFD(v2) / 0xFE(v1) */
    uint8_t  len;
    uint8_t  incompat;
    uint8_t  compat;
    uint8_t  seq;
    uint8_t  sysid;
    uint8_t  compid;
    uint32_t msgid;             /* v2 为 24 位 */
    uint8_t  payload[MAV_MAX_PAYLOAD];
    uint16_t checksum;          /* 线上收到的校验值 */
} MavFrame;

/* 计算一帧的校验（含 CRC_EXTRA）。返回 false 表示 msgid 未知。 */
int mav_checksum(const MavFrame *f, uint16_t *out);

/* 打包成字节流。返回写入长度，<0 为错误。 */
int mav_encode(const MavFrame *f, uint8_t *out, size_t cap);

/* --------------------------------------------------------- 流式解析器 */
typedef struct {
    uint8_t  buf[MAV_MAX_FRAME];
    size_t   len;               /* 已缓存字节数 */
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t resyncs;           /* 因数据非法而丢弃字节重新同步的次数 */
    uint32_t dropped_bytes;
} MavParser;

void mav_parser_init(MavParser *p);
/*
 * 逐字节喂入。每喂一个字节返回：
 *    1  = 刚解析出一整帧（结果写入 out）
 *    0  = 还没凑齐
 *   -1  = 该帧校验/格式错误，已丢弃并重新同步
 * 设计成"一字节一调用"是为了贴合串口中断/DMA 空闲中断的真实使用方式。
 */
int mav_parse_byte(MavParser *p, uint8_t b, MavFrame *out);
/* 便捷：喂一整块缓冲，返回解析出的帧数 */
int mav_parse_buf(MavParser *p, const uint8_t *data, size_t n,
                  MavFrame *out, size_t max_frames);

/* --------------------------------------------------------- 载荷编解码 */
/* HEARTBEAT：type/autopilot/base_mode/custom_mode/system_status/mavlink_version */
typedef struct {
    uint8_t  type;
    uint8_t  autopilot;
    uint8_t  base_mode;
    uint32_t custom_mode;
    uint8_t  system_status;
    uint8_t  mavlink_version;
} MavHeartbeat;

int mav_pack_heartbeat(const MavHeartbeat *hb, MavFrame *out,
                       uint8_t sysid, uint8_t compid);
int mav_unpack_heartbeat(const MavFrame *f, MavHeartbeat *out);

/* COMMAND_LONG 的载荷（按 MAVLink 字段顺序，注意是"小端 + 字段重排"的线格式） */
typedef struct {
    uint16_t command;
    uint8_t  target_system;
    uint8_t  target_component;
    uint8_t  confirmation;
    float    param1, param2, param3, param4, param5, param6, param7;
} MavCommandLong;

int mav_pack_command_long(const MavCommandLong *c, MavFrame *out,
                          uint8_t sysid, uint8_t compid);
int mav_unpack_command_long(const MavFrame *f, MavCommandLong *out);

/* 常用命令 ID */
#define MAV_CMD_NAV_TAKEOFF      22
#define MAV_CMD_NAV_LAND         21
#define MAV_CMD_NAV_RETURN       20
#define MAV_CMD_COMPONENT_ARM    400
#define MAV_CMD_DO_SET_MODE      176

/* --------------------------------------------------------- 链路状态机 */
typedef enum {
    MAV_LINK_DOWN = 0,          /* 从未收到心跳 */
    MAV_LINK_UP,                /* 心跳正常 */
    MAV_LINK_TIMEOUT,           /* 心跳超时 */
} MavLinkState;

typedef struct {
    MavLinkState state;
    uint8_t  sysid;             /* 对端系统 ID */
    uint8_t  compid;
    uint8_t  peer_type;         /* 对端 HEARTBEAT.type */
    uint32_t last_hb_ms;        /* 上次心跳时间戳（毫秒） */
    uint32_t hb_count;
    uint32_t hb_missed;
    uint32_t timeout_ms;        /* 心跳超时阈值 */
    uint8_t  armed;             /* 是否已解锁 */
    uint8_t  mode;              /* 简化模式：0 手动 1 定高 2 任务 3 返航 */
    uint32_t cmd_sent;
    uint32_t cmd_acked;
    uint16_t last_ack_cmd;      /* 最近一次被应答的命令 */
    uint8_t  last_ack_result;
} MavLink;

void mav_link_init(MavLink *l, uint32_t timeout_ms);
/* 收到任意帧时调用；内部据此维护心跳与状态。now_ms 为单调毫秒时间戳 */
void mav_link_on_frame(MavLink *l, const MavFrame *f, uint32_t now_ms);
/* 周期性调用：判定心跳超时 */
void mav_link_tick(MavLink *l, uint32_t now_ms);
/* 发起解锁/上锁；返回打包好的帧 */
int  mav_link_arm(MavLink *l, int arm, MavFrame *out,
                  uint8_t sysid, uint8_t compid);
/* 设置模式 */
int  mav_link_set_mode(MavLink *l, uint8_t mode, MavFrame *out,
                       uint8_t sysid, uint8_t compid);
/* 是否允许起飞：链路在线 + 已解锁 */
int  mav_link_ready(const MavLink *l);

#endif /* MAVLINK_H */
