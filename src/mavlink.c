/* mavlink.c — MAVLink v2 编解码与航电链路状态机实现 */
#include "mavlink.h"
#include <string.h>

const char *mav_status_str(mav_status_t s)
{
    switch (s) {
    case MAV_OK:              return "OK";
    case MAV_ERR_PARAM:       return "PARAM";
    case MAV_ERR_NOMEM:       return "NOMEM";
    case MAV_ERR_CRC:         return "CRC";
    case MAV_ERR_UNSUPPORTED: return "UNSUPPORTED";
    case MAV_ERR_STATE:       return "STATE";
    }
    return "?";
}

/* =============================================================== CRC
 * MAVLink 用的是 CRC-16/MCRF4XX：初值 0xFFFF，多项式 0x1021 反射（0x8408）。
 * 两处容易错：
 *   1) 反射多项式要用 0x8408，不是 0x1021
 *   2) 校验要覆盖「除 STX 和校验本身以外的所有字节」，最后**再累积 CRC_EXTRA**
 */
static uint16_t crc_accumulate(uint8_t b, uint16_t crc)
{
    uint8_t tmp = b ^ (uint8_t)(crc & 0xFF);
    tmp ^= (uint8_t)(tmp << 4);
    return (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ (tmp >> 4));
}

static uint16_t crc_calc(const uint8_t *buf, size_t n, uint16_t crc)
{
    for (size_t i = 0; i < n; i++) crc = crc_accumulate(buf[i], crc);
    return crc;
}

uint8_t mav_crc_extra(uint32_t msgid)
{
    switch (msgid) {
    case MAV_MSG_HEARTBEAT:       return 50;
    case MAV_MSG_SYS_STATUS:      return 124;
    case MAV_MSG_PING:            return 237;
    case MAV_MSG_GPS_RAW_INT:     return 24;
    case MAV_MSG_ATTITUDE:        return 39;
    case MAV_MSG_GLOBAL_POSITION: return 104;
    case MAV_MSG_COMMAND_LONG:    return 152;
    case MAV_MSG_COMMAND_ACK:     return 143;
    case MAV_MSG_STATUSTEXT:      return 83;
    default:                      return 0xFF;   /* 0xFF 表示未知 */
    }
}

int mav_checksum(const MavFrame *f, uint16_t *out)
{
    if (!f || !out) return MAV_ERR_PARAM;
    const uint8_t extra = mav_crc_extra(f->msgid);
    if (extra == 0xFF) return MAV_ERR_UNSUPPORTED;

    uint8_t hdr[9];
    size_t n = 0;
    if (f->magic == MAV_STX_V1) {
        hdr[n++] = f->len;
        hdr[n++] = f->seq;
        hdr[n++] = f->sysid;
        hdr[n++] = f->compid;
        hdr[n++] = (uint8_t)(f->msgid & 0xFF);
    } else {
        hdr[n++] = f->len;
        hdr[n++] = f->incompat;
        hdr[n++] = f->compat;
        hdr[n++] = f->seq;
        hdr[n++] = f->sysid;
        hdr[n++] = f->compid;
        hdr[n++] = (uint8_t)(f->msgid & 0xFF);
        hdr[n++] = (uint8_t)((f->msgid >> 8) & 0xFF);
        hdr[n++] = (uint8_t)((f->msgid >> 16) & 0xFF);
    }
    uint16_t crc = 0xFFFF;
    crc = crc_calc(hdr, n, crc);
    crc = crc_calc(f->payload, f->len, crc);
    crc = crc_accumulate(extra, crc);          /* 关键：把 CRC_EXTRA 也累积进去 */
    *out = crc;
    return MAV_OK;
}

int mav_encode(const MavFrame *f, uint8_t *out, size_t cap)
{
    if (!f || !out) return MAV_ERR_PARAM;
    /* f->len 是 uint8_t，天然不会超过 MAV_MAX_PAYLOAD（255），无需再判 */
    const int hdr = (f->magic == MAV_STX_V1) ? MAV_HEADER_V1 : MAV_HEADER_V2;
    const size_t total = (size_t)hdr + f->len + 2;
    if (cap < total) return MAV_ERR_NOMEM;

    uint16_t crc = 0;
    if (mav_checksum(f, &crc) != MAV_OK) return MAV_ERR_UNSUPPORTED;

    size_t i = 0;
    out[i++] = f->magic;
    out[i++] = f->len;
    if (f->magic == MAV_STX_V1) {
        out[i++] = f->seq;
        out[i++] = f->sysid;
        out[i++] = f->compid;
        out[i++] = (uint8_t)(f->msgid & 0xFF);
    } else {
        out[i++] = f->incompat;
        out[i++] = f->compat;
        out[i++] = f->seq;
        out[i++] = f->sysid;
        out[i++] = f->compid;
        out[i++] = (uint8_t)(f->msgid & 0xFF);
        out[i++] = (uint8_t)((f->msgid >> 8) & 0xFF);
        out[i++] = (uint8_t)((f->msgid >> 16) & 0xFF);
    }
    memcpy(out + i, f->payload, f->len);
    i += f->len;
    out[i++] = (uint8_t)(crc & 0xFF);          /* 小端 */
    out[i++] = (uint8_t)((crc >> 8) & 0xFF);
    return (int)i;
}

/* ========================================================= 流式解析器 */
void mav_parser_init(MavParser *p)
{
    memset(p, 0, sizeof(*p));
}

/* 从缓冲区头部丢弃 n 字节 */
static void drop_front(MavParser *p, size_t n)
{
    if (n >= p->len) {
        p->dropped_bytes += (uint32_t)p->len;
        p->len = 0;
        return;
    }
    memmove(p->buf, p->buf + n, p->len - n);
    p->len -= n;
    p->dropped_bytes += (uint32_t)n;
}

/* 尝试从缓冲区解析出一帧；返回 1 成功 0 不够 -1 该位置非法（已丢 1 字节） */
static int try_parse(MavParser *p, MavFrame *out)
{
    while (p->len > 0) {
        /* 1) 找 STX：不是 STX 就丢掉，这就是"重新同步" */
        if (p->buf[0] != MAV_STX_V2 && p->buf[0] != MAV_STX_V1) {
            drop_front(p, 1);
            p->resyncs++;
            continue;
        }
        const int is_v2 = (p->buf[0] == MAV_STX_V2);
        const size_t hdr = is_v2 ? MAV_HEADER_V2 : MAV_HEADER_V1;
        if (p->len < hdr) return 0;                       /* 头都没收齐 */
        const uint8_t len = p->buf[1];
        const size_t total = hdr + len + 2;
        if (p->len < total) return 0;                     /* 载荷还没收齐 */

        MavFrame f;
        memset(&f, 0, sizeof(f));
        f.magic  = p->buf[0];
        f.len    = len;
        f.seq    = p->buf[is_v2 ? 4 : 2];
        f.sysid  = p->buf[is_v2 ? 5 : 3];
        f.compid = p->buf[is_v2 ? 6 : 4];
        if (is_v2) {
            f.incompat = p->buf[2];
            f.compat   = p->buf[3];
            f.msgid = (uint32_t)p->buf[7] | ((uint32_t)p->buf[8] << 8) |
                      ((uint32_t)p->buf[9] << 16);
        } else {
            f.msgid = p->buf[5];
        }
        memcpy(f.payload, p->buf + hdr, len);
        f.checksum = (uint16_t)p->buf[hdr + len] |
                     ((uint16_t)p->buf[hdr + len + 1] << 8);

        uint16_t want = 0;
        const int cr = mav_checksum(&f, &want);
        if (cr != MAV_OK) {
            /* 未支持的消息：按"整帧丢弃"处理，并计一次重同步 */
            drop_front(p, 1);
            p->resyncs++;
            continue;
        }
        if (want != f.checksum) {
            p->crc_errors++;
            /* 关键设计：CRC 错时**丢弃整帧**（total 字节），而不是只丢 1 字节。
             *
             * 一开始我写的是"只丢 1 字节再重试"，理由是想防止载荷里含 0xFD
             * 时把后续真帧一起丢掉 —— 但这个理由站不住：
             * 长度字段已经精确给出该帧有多长，丢 total 字节不会多丢。
             * 反过来，"只丢 1 字节"会造成更严重的后果：
             * 丢掉 STX 后解析器会一路扫到载荷里的 0xFD，把它当成新帧头，
             * 再读到下一个字节 0xFD 当作长度 253 -> 于是**死等一个永远凑不齐的
             * 巨型帧**，后面的真帧再也解不出来。
             * MAVLink 官方解析器的做法也是丢弃整帧。 */
            drop_front(p, total);
            p->resyncs++;
            continue;
        }
        drop_front(p, total);
        p->frames_ok++;
        if (out) *out = f;
        return 1;
    }
    return 0;
}

int mav_parse_byte(MavParser *p, uint8_t b, MavFrame *out)
{
    if (!p) return -1;
    if (p->len >= sizeof(p->buf)) {          /* 缓冲满：丢掉最旧的 1 字节 */
        drop_front(p, 1);
        p->resyncs++;
    }
    p->buf[p->len++] = b;
    return try_parse(p, out);
}

int mav_parse_buf(MavParser *p, const uint8_t *data, size_t n,
                  MavFrame *out, size_t max_frames)
{
    if (!p || !data) return MAV_ERR_PARAM;
    int got = 0;
    for (size_t i = 0; i < n; i++) {
        MavFrame f;
        const int r = mav_parse_byte(p, data[i], &f);
        if (r == 1) {
            if (out && (size_t)got < max_frames) out[got] = f;
            got++;
        }
    }
    return got;
}

/* ======================================================= 载荷编解码
 * MAVLink 的线格式是「按字段宽度从大到小重排」的，不是源码里的声明顺序，
 * 这里按规范排：HEARTBEAT 线上顺序 = custom_mode(4) type autopilot base_mode
 * system_status mavlink_version
 */
static void put_u32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }
static void put_u16(uint8_t *p, uint16_t v) { memcpy(p, &v, 2); }
static void put_f32(uint8_t *p, float v)    { memcpy(p, &v, 4); }
static uint32_t get_u32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static uint16_t get_u16(const uint8_t *p) { uint16_t v; memcpy(&v, p, 2); return v; }
static float    get_f32(const uint8_t *p) { float v; memcpy(&v, p, 4); return v; }

static void frame_base(MavFrame *f, uint32_t msgid, uint8_t len,
                       uint8_t sysid, uint8_t compid)
{
    memset(f, 0, sizeof(*f));
    f->magic = MAV_STX_V2;
    f->len = len;
    f->msgid = msgid;
    f->sysid = sysid;
    f->compid = compid;
    f->seq = 0;
}

int mav_pack_heartbeat(const MavHeartbeat *hb, MavFrame *out,
                       uint8_t sysid, uint8_t compid)
{
    if (!hb || !out) return MAV_ERR_PARAM;
    frame_base(out, MAV_MSG_HEARTBEAT, 9, sysid, compid);
    put_u32(out->payload + 0, hb->custom_mode);
    out->payload[4] = hb->type;
    out->payload[5] = hb->autopilot;
    out->payload[6] = hb->base_mode;
    out->payload[7] = hb->system_status;
    out->payload[8] = hb->mavlink_version;
    return MAV_OK;
}

int mav_unpack_heartbeat(const MavFrame *f, MavHeartbeat *out)
{
    if (!f || !out) return MAV_ERR_PARAM;
    if (f->msgid != MAV_MSG_HEARTBEAT || f->len < 9) return MAV_ERR_PARAM;
    out->custom_mode    = get_u32(f->payload + 0);
    out->type           = f->payload[4];
    out->autopilot      = f->payload[5];
    out->base_mode      = f->payload[6];
    out->system_status  = f->payload[7];
    out->mavlink_version = f->payload[8];
    return MAV_OK;
}

int mav_pack_command_long(const MavCommandLong *c, MavFrame *out,
                          uint8_t sysid, uint8_t compid)
{
    if (!c || !out) return MAV_ERR_PARAM;
    frame_base(out, MAV_MSG_COMMAND_LONG, 33, sysid, compid);
    /* 线上顺序：7 个 float -> command(2) -> target_sys -> target_comp -> confirmation */
    put_f32(out->payload + 0,  c->param1);
    put_f32(out->payload + 4,  c->param2);
    put_f32(out->payload + 8,  c->param3);
    put_f32(out->payload + 12, c->param4);
    put_f32(out->payload + 16, c->param5);
    put_f32(out->payload + 20, c->param6);
    put_f32(out->payload + 24, c->param7);
    put_u16(out->payload + 28, c->command);
    out->payload[30] = c->target_system;
    out->payload[31] = c->target_component;
    out->payload[32] = c->confirmation;
    return MAV_OK;
}

int mav_unpack_command_long(const MavFrame *f, MavCommandLong *out)
{
    if (!f || !out) return MAV_ERR_PARAM;
    if (f->msgid != MAV_MSG_COMMAND_LONG || f->len < 33) return MAV_ERR_PARAM;
    out->param1 = get_f32(f->payload + 0);
    out->param2 = get_f32(f->payload + 4);
    out->param3 = get_f32(f->payload + 8);
    out->param4 = get_f32(f->payload + 12);
    out->param5 = get_f32(f->payload + 16);
    out->param6 = get_f32(f->payload + 20);
    out->param7 = get_f32(f->payload + 24);
    out->command = get_u16(f->payload + 28);
    out->target_system = f->payload[30];
    out->target_component = f->payload[31];
    out->confirmation = f->payload[32];
    return MAV_OK;
}

/* ======================================================= 链路状态机 */
void mav_link_init(MavLink *l, uint32_t timeout_ms)
{
    memset(l, 0, sizeof(*l));
    l->state = MAV_LINK_DOWN;
    l->timeout_ms = timeout_ms ? timeout_ms : 3000;
}

void mav_link_on_frame(MavLink *l, const MavFrame *f, uint32_t now_ms)
{
    if (!l || !f) return;

    if (f->msgid == MAV_MSG_HEARTBEAT) {
        MavHeartbeat hb;
        if (mav_unpack_heartbeat(f, &hb) == MAV_OK) {
            if (l->state != MAV_LINK_UP) {
                l->state = MAV_LINK_UP;
                l->sysid = f->sysid;
                l->compid = f->compid;
            }
            l->peer_type = hb.type;
            l->last_hb_ms = now_ms;
            l->hb_count++;
            l->hb_missed = 0;
            /* base_mode 的 bit7 是 SAFETY_ARMED */
            l->armed = (hb.base_mode & 0x80) ? 1 : 0;
        }
        return;
    }

    if (f->msgid == MAV_MSG_COMMAND_ACK && f->len >= 3) {
        l->last_ack_cmd = get_u16(f->payload + 0);
        l->last_ack_result = f->payload[2];
        l->cmd_acked++;
        return;
    }
}

void mav_link_tick(MavLink *l, uint32_t now_ms)
{
    if (!l) return;
    if (l->state != MAV_LINK_UP) return;
    if (now_ms - l->last_hb_ms > l->timeout_ms) {
        l->state = MAV_LINK_TIMEOUT;
        l->hb_missed++;
        l->armed = 0;                 /* 失联必须视为已上锁，避免"以为还在飞" */
    }
}

int mav_link_arm(MavLink *l, int arm, MavFrame *out, uint8_t sysid, uint8_t compid)
{
    if (!l || !out) return MAV_ERR_PARAM;
    if (l->state != MAV_LINK_UP) return MAV_ERR_STATE;   /* 链路不通不允许解锁 */
    MavCommandLong c;
    memset(&c, 0, sizeof(c));
    c.command = MAV_CMD_COMPONENT_ARM;
    c.target_system = l->sysid;
    c.target_component = l->compid;
    c.param1 = arm ? 1.0f : 0.0f;
    c.confirmation = 0;
    const int r = mav_pack_command_long(&c, out, sysid, compid);
    if (r == MAV_OK) l->cmd_sent++;
    return r;
}

int mav_link_set_mode(MavLink *l, uint8_t mode, MavFrame *out,
                      uint8_t sysid, uint8_t compid)
{
    if (!l || !out) return MAV_ERR_PARAM;
    if (l->state != MAV_LINK_UP) return MAV_ERR_STATE;
    if (mode > 3) return MAV_ERR_PARAM;
    MavCommandLong c;
    memset(&c, 0, sizeof(c));
    c.command = MAV_CMD_DO_SET_MODE;
    c.target_system = l->sysid;
    c.target_component = l->compid;
    c.param1 = 1.0f;              /* MAV_MODE_FLAG_CUSTOM_MODE_ENABLED */
    c.param2 = (float)mode;
    const int r = mav_pack_command_long(&c, out, sysid, compid);
    if (r == MAV_OK) {
        l->cmd_sent++;
        l->mode = mode;
    }
    return r;
}

int mav_link_ready(const MavLink *l)
{
    if (!l) return 0;
    return (l->state == MAV_LINK_UP) && l->armed;
}
