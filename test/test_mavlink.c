/* test_mavlink.c — MAVLink v2 航电链路库单元测试（纯 C99）
 *
 *  T1 crc        CRC_EXTRA 表、校验含 EXTRA、未知消息拒绝
 *  T2 encode     打包/解包往返、缓冲不足、v1/v2 帧头差异
 *  T3 parse      逐字节喂入、粘包（一帧接一帧）、半包、噪音重同步、
 *                CRC 错后只丢 1 字节（载荷里含 0xFD 也不能丢真帧）
 *  T4 payload    HEARTBEAT / COMMAND_LONG 的线格式字段重排
 *  T5 link       心跳在线/超时、失联自动上锁、解锁需链路在线、模式设置、命令应答
 */
#include "mavlink.h"
#include <stdio.h>
#include <string.h>

static int passed = 0, failed = 0;

#define CHECK(cond, ...) do {                        \
    if (cond) { passed++; printf("  [PASS] "); }      \
    else      { failed++; printf("  [FAIL] "); }      \
    printf(__VA_ARGS__); printf("\n");                \
} while (0)

/* ------------------------------------------------------------------ T1 */
static void t1_crc(void)
{
    printf("T1 CRC 与 CRC_EXTRA\n");

    CHECK(mav_crc_extra(MAV_MSG_HEARTBEAT) == 50, "HEARTBEAT 的 CRC_EXTRA = %u",
          mav_crc_extra(MAV_MSG_HEARTBEAT));
    CHECK(mav_crc_extra(MAV_MSG_COMMAND_LONG) == 152, "COMMAND_LONG 的 CRC_EXTRA = %u",
          mav_crc_extra(MAV_MSG_COMMAND_LONG));
    CHECK(mav_crc_extra(MAV_MSG_STATUSTEXT) == 83, "STATUSTEXT 的 CRC_EXTRA = %u",
          mav_crc_extra(MAV_MSG_STATUSTEXT));
    CHECK(mav_crc_extra(0x123456) == 0xFF, "未知消息 -> 0xFF（表示无表项）");

    /* 未知消息不允许打包（否则 CRC 无从算起） */
    MavFrame f;
    memset(&f, 0, sizeof(f));
    f.magic = MAV_STX_V2;
    f.msgid = 0x123456;
    uint16_t c = 0;
    CHECK(mav_checksum(&f, &c) == MAV_ERR_UNSUPPORTED,
          "未知消息算校验 -> UNSUPPORTED");
    CHECK(mav_checksum(NULL, &c) == MAV_ERR_PARAM, "空指针 -> PARAM");

    /* 校验必须把 CRC_EXTRA 累积进去：只改 msgid 的 EXTRA 就会变 */
    MavFrame a, b;
    memset(&a, 0, sizeof(a)); memset(&b, 0, sizeof(b));
    a.magic = b.magic = MAV_STX_V2;
    a.msgid = MAV_MSG_HEARTBEAT;
    b.msgid = MAV_MSG_ATTITUDE;
    a.len = b.len = 9;
    uint16_t ca = 0, cb = 0;
    mav_checksum(&a, &ca);
    mav_checksum(&b, &cb);
    CHECK(ca != cb, "不同消息校验不同（EXTRA 参与）：%04X vs %04X", ca, cb);

    /* 同内容必须稳定 */
    uint16_t c2 = 0;
    mav_checksum(&a, &c2);
    CHECK(ca == c2, "同内容校验稳定 = %04X", ca);
}

/* ------------------------------------------------------------------ T2 */
static void t2_encode(void)
{
    printf("T2 打包与帧头\n");

    MavHeartbeat hb;
    memset(&hb, 0, sizeof(hb));
    hb.type = 2; hb.autopilot = 3; hb.base_mode = 0x80;
    hb.custom_mode = 0x0A0B0C0D; hb.system_status = 4; hb.mavlink_version = 3;

    MavFrame f;
    CHECK(mav_pack_heartbeat(&hb, &f, 1, 1) == MAV_OK, "打包 HEARTBEAT 成功");
    CHECK(f.magic == MAV_STX_V2, "帧头 STX = 0x%02X（v2）", f.magic);
    CHECK(f.len == 9, "载荷长度 = %u", f.len);
    CHECK(f.sysid == 1 && f.compid == 1, "sysid/compid = %u/%u", f.sysid, f.compid);

    uint8_t buf[MAV_MAX_FRAME];
    const int n = mav_encode(&f, buf, sizeof(buf));
    CHECK(n == MAV_HEADER_V2 + 9 + 2, "编码长度 = %d（10+9+2）", n);
    CHECK(buf[0] == 0xFD, "首字节 = 0x%02X", buf[0]);
    CHECK(buf[1] == 9, "第二字节 = 载荷长度 %u", buf[1]);

    /* 缓冲不足 */
    CHECK(mav_encode(&f, buf, 5) == MAV_ERR_NOMEM, "缓冲不足 -> NOMEM");
    CHECK(mav_encode(NULL, buf, sizeof(buf)) == MAV_ERR_PARAM, "空帧 -> PARAM");

    /* 载荷超长 */
    MavFrame big = f;
    big.len = 200;
    CHECK(mav_encode(&big, buf, sizeof(buf)) > 0, "255 以内载荷可编码");
    big.len = 255;
    CHECK(mav_encode(&big, buf, sizeof(buf)) > 0, "255 字节上限可编码");

    /* v1 帧头只有 1 字节 msgid */
    MavFrame v1 = f;
    v1.magic = MAV_STX_V1;
    v1.msgid = MAV_MSG_HEARTBEAT;
    const int n1 = mav_encode(&v1, buf, sizeof(buf));
    CHECK(n1 == MAV_HEADER_V1 + 9 + 2, "v1 编码长度 = %d（6+9+2）", n1);
    CHECK(buf[0] == 0xFE, "v1 首字节 = 0x%02X", buf[0]);
}

/* ------------------------------------------------------------------ T3 */
static void t3_parse(void)
{
    printf("T3 流式解析\n");

    MavHeartbeat hb;
    memset(&hb, 0, sizeof(hb));
    hb.type = 2; hb.autopilot = 3; hb.base_mode = 0x80;
    hb.custom_mode = 0x11223344; hb.system_status = 4; hb.mavlink_version = 3;
    MavFrame f;
    mav_pack_heartbeat(&hb, &f, 1, 1);
    uint8_t buf[MAV_MAX_FRAME];
    const int n = mav_encode(&f, buf, sizeof(buf));

    /* 逐字节喂入：只有最后一个字节才应返回 1 */
    MavParser p;
    mav_parser_init(&p);
    int hits = 0, last_idx = -1;
    for (int i = 0; i < n; i++) {
        MavFrame got;
        if (mav_parse_byte(&p, buf[i], &got) == 1) { hits++; last_idx = i; }
    }
    CHECK(hits == 1 && last_idx == n - 1,
          "逐字节解析：%d 帧，在最后 1 字节凑齐（idx=%d/%d）", hits, last_idx, n - 1);
    CHECK(p.frames_ok == 1, "解析计数 = %u", p.frames_ok);

    /* 粘包：两帧连着发 */
    mav_parser_init(&p);
    uint8_t two[2 * MAV_MAX_FRAME];
    memcpy(two, buf, n);
    memcpy(two + n, buf, n);
    MavFrame out[4];
    const int k = mav_parse_buf(&p, two, (size_t)n * 2, out, 4);
    CHECK(k == 2, "粘包解析出 %d 帧", k);
    CHECK(out[0].msgid == MAV_MSG_HEARTBEAT && out[1].msgid == MAV_MSG_HEARTBEAT,
          "两帧消息 ID 都正确");

    /* 半包：只喂一半，不应出帧；补齐后出帧 */
    mav_parser_init(&p);
    MavFrame got;
    int r = 0;
    for (int i = 0; i < n / 2; i++) r += (mav_parse_byte(&p, buf[i], &got) == 1);
    CHECK(r == 0, "半包不出帧（已喂 %d/%d 字节）", n / 2, n);
    for (int i = n / 2; i < n; i++) r += (mav_parse_byte(&p, buf[i], &got) == 1);
    CHECK(r == 1, "补齐后出 1 帧");

    /* 噪音 + 真帧：应能重新同步并解出真帧 */
    mav_parser_init(&p);
    uint8_t noisy[64 + MAV_MAX_FRAME];
    memset(noisy, 0x5A, 64);                  /* 64 字节垃圾 */
    memcpy(noisy + 64, buf, n);
    const int k2 = mav_parse_buf(&p, noisy, 64 + (size_t)n, out, 4);
    CHECK(k2 == 1, "带 64 字节噪音仍解出 %d 帧", k2);
    CHECK(p.resyncs >= 64, "重同步计数 = %u（≥64）", p.resyncs);

    /* CRC 错：改一个载荷字节 */
    mav_parser_init(&p);
    uint8_t bad[MAV_MAX_FRAME];
    memcpy(bad, buf, n);
    bad[MAV_HEADER_V2 + 2] ^= 0xFF;           /* 改载荷 */
    const int k3 = mav_parse_buf(&p, bad, (size_t)n, out, 4);
    CHECK(k3 == 0, "CRC 错 -> 不出帧（出了 %d 帧）", k3);
    CHECK(p.crc_errors >= 1, "CRC 错误计数 = %u", p.crc_errors);

    /* 关键用例：载荷里含 0xFD 时，坏帧之后紧跟的好帧仍必须解出来。
     * 如果实现是"整帧丢弃"，这里会连带丢掉后面的真帧。 */
    mav_parser_init(&p);
    MavFrame mp;
    memset(&mp, 0, sizeof(mp));
    mp.magic = MAV_STX_V2;
    mp.msgid = MAV_MSG_STATUSTEXT;
    mp.len = 8;
    memset(mp.payload, 0xFD, 8);              /* 载荷里塞满 0xFD */
    uint8_t fbuf[MAV_MAX_FRAME];
    const int fn = mav_encode(&mp, fbuf, sizeof(fbuf));
    CHECK(fn > 0, "构造含 0xFD 载荷的帧成功（%d 字节）", fn);

    uint8_t mix[2 * MAV_MAX_FRAME];
    memcpy(mix, fbuf, fn);
    mix[MAV_HEADER_V2 + 3] ^= 0x01;           /* 破坏第一帧的载荷 -> CRC 错 */
    memcpy(mix + fn, buf, n);                 /* 紧跟一帧完好的 HEARTBEAT */
    const int k4 = mav_parse_buf(&p, mix, (size_t)fn + n, out, 4);
    CHECK(k4 == 1 && out[0].msgid == MAV_MSG_HEARTBEAT,
          "坏帧（载荷含 0xFD）后仍解出紧跟的好帧：%d 帧, msgid=%u",
          k4, k4 > 0 ? out[0].msgid : 999);

    /* 缓冲溢出保护：连续喂大量垃圾不应崩、不应越界 */
    mav_parser_init(&p);
    for (int i = 0; i < 5000; i++) (void)mav_parse_byte(&p, (uint8_t)(i * 7), NULL);
    CHECK(p.len <= MAV_MAX_FRAME, "长时间垃圾输入后缓冲长度 = %zu（不越界）", p.len);
}

/* ------------------------------------------------------------------ T4 */
static void t4_payload(void)
{
    printf("T4 载荷线格式\n");

    /* HEARTBEAT 的线上顺序是 custom_mode(4) 在前，不是结构体声明顺序 */
    MavHeartbeat hb, hb2;
    memset(&hb, 0, sizeof(hb));
    hb.custom_mode = 0xDEADBEEF;
    hb.type = 2; hb.autopilot = 3; hb.base_mode = 0x81;
    hb.system_status = 4; hb.mavlink_version = 3;

    MavFrame f;
    mav_pack_heartbeat(&hb, &f, 1, 1);
    CHECK(f.payload[0] == 0xEF && f.payload[3] == 0xDE,
          "custom_mode 小端在最前：%02X %02X %02X %02X",
          f.payload[0], f.payload[1], f.payload[2], f.payload[3]);
    CHECK(f.payload[4] == 2 && f.payload[5] == 3,
          "type/autopilot 紧随其后：%u/%u", f.payload[4], f.payload[5]);

    CHECK(mav_unpack_heartbeat(&f, &hb2) == MAV_OK, "解包 HEARTBEAT 成功");
    CHECK(hb2.custom_mode == 0xDEADBEEF && hb2.base_mode == 0x81 &&
          hb2.mavlink_version == 3, "往返一致：mode=%08X base=%02X ver=%u",
          hb2.custom_mode, hb2.base_mode, hb2.mavlink_version);

    CHECK(mav_unpack_heartbeat(NULL, &hb2) == MAV_ERR_PARAM, "空帧 -> PARAM");
    MavFrame wrong = f;
    wrong.msgid = MAV_MSG_ATTITUDE;
    CHECK(mav_unpack_heartbeat(&wrong, &hb2) == MAV_ERR_PARAM,
          "消息 ID 不符 -> PARAM");

    /* COMMAND_LONG：7 个 float 在前，然后 command(2) + target + confirmation */
    MavCommandLong c, c2;
    memset(&c, 0, sizeof(c));
    c.command = MAV_CMD_COMPONENT_ARM;
    c.target_system = 1; c.target_component = 1; c.confirmation = 0;
    c.param1 = 1.0f; c.param2 = 2.5f; c.param7 = 7.25f;

    MavFrame cf;
    CHECK(mav_pack_command_long(&c, &cf, 255, 190) == MAV_OK, "打包 COMMAND_LONG");
    CHECK(cf.len == 33, "载荷长度 = %u（7*4+2+1+1+1）", cf.len);
    CHECK(mav_unpack_command_long(&cf, &c2) == MAV_OK, "解包 COMMAND_LONG");
    CHECK(c2.command == MAV_CMD_COMPONENT_ARM, "command = %u", c2.command);
    CHECK(c2.param1 == 1.0f && c2.param2 == 2.5f && c2.param7 == 7.25f,
          "float 往返一致：%.2f %.2f %.2f", c2.param1, c2.param2, c2.param7);
    CHECK(c2.target_system == 1 && cf.sysid == 255 && cf.compid == 190,
          "目标/源 ID 正确：tgt=%u src=%u/%u", c2.target_system, cf.sysid, cf.compid);
}

/* ------------------------------------------------------------------ T5 */
static void t5_link(void)
{
    printf("T5 链路状态机\n");

    MavLink l;
    mav_link_init(&l, 3000);
    CHECK(l.state == MAV_LINK_DOWN, "初始状态 DOWN");
    CHECK(l.timeout_ms == 3000, "超时阈值 = %u ms", l.timeout_ms);
    CHECK(mav_link_ready(&l) == 0, "未上线时不可起飞");

    /* 未上线不允许解锁 */
    MavFrame out;
    CHECK(mav_link_arm(&l, 1, &out, 255, 190) == MAV_ERR_STATE,
          "链路未上线时解锁 -> STATE（拒绝）");

    /* 收到心跳 -> 上线 */
    MavHeartbeat hb;
    memset(&hb, 0, sizeof(hb));
    hb.type = 2; hb.autopilot = 3; hb.base_mode = 0x00;
    hb.mavlink_version = 3;
    MavFrame f;
    mav_pack_heartbeat(&hb, &f, 1, 1);
    mav_link_on_frame(&l, &f, 1000);
    CHECK(l.state == MAV_LINK_UP, "收到心跳 -> UP");
    CHECK(l.sysid == 1 && l.compid == 1, "记录对端 ID：%u/%u", l.sysid, l.compid);
    CHECK(l.hb_count == 1 && l.armed == 0, "心跳计数 = %u，未解锁", l.hb_count);
    CHECK(mav_link_ready(&l) == 0, "在线但未解锁 -> 仍不可起飞");

    /* 解锁命令 */
    CHECK(mav_link_arm(&l, 1, &out, 255, 190) == MAV_OK, "在线时解锁命令打包成功");
    CHECK(out.msgid == MAV_MSG_COMMAND_LONG, "发出的是 COMMAND_LONG");
    MavCommandLong cl;
    mav_unpack_command_long(&out, &cl);
    CHECK(cl.command == MAV_CMD_COMPONENT_ARM && cl.param1 == 1.0f,
          "解锁命令 = %u，param1 = %.1f", cl.command, cl.param1);
    CHECK(l.cmd_sent == 1, "已发命令计数 = %u", l.cmd_sent);

    /* 对端回报已解锁（base_mode bit7） */
    hb.base_mode = 0x80;
    mav_pack_heartbeat(&hb, &f, 1, 1);
    mav_link_on_frame(&l, &f, 1100);
    CHECK(l.armed == 1, "收到 ARMED 心跳后 armed = %u", l.armed);
    CHECK(mav_link_ready(&l) == 1, "在线 + 解锁 -> 允许起飞");

    /* 命令应答 */
    MavFrame ack;
    memset(&ack, 0, sizeof(ack));
    ack.magic = MAV_STX_V2;
    ack.msgid = MAV_MSG_COMMAND_ACK;
    ack.len = 3;
    ack.payload[0] = (uint8_t)(MAV_CMD_COMPONENT_ARM & 0xFF);
    ack.payload[1] = (uint8_t)((MAV_CMD_COMPONENT_ARM >> 8) & 0xFF);
    ack.payload[2] = 0;                        /* MAV_RESULT_ACCEPTED */
    mav_link_on_frame(&l, &ack, 1200);
    CHECK(l.cmd_acked == 1 && l.last_ack_cmd == MAV_CMD_COMPONENT_ARM &&
          l.last_ack_result == 0, "应答匹配：cmd=%u result=%u",
          l.last_ack_cmd, l.last_ack_result);

    /* 心跳正常时 tick 不应超时 */
    mav_link_tick(&l, 3000);
    CHECK(l.state == MAV_LINK_UP, "3.0 s 时仍在 UP（阈值 3000 ms）");

    /* 心跳超时 -> TIMEOUT 且自动上锁 */
    mav_link_tick(&l, 1201 + 3001);
    CHECK(l.state == MAV_LINK_TIMEOUT, "超时后状态 = TIMEOUT");
    CHECK(l.armed == 0, "失联自动上锁（armed=%u）—— 不能默认飞机还在飞", l.armed);
    CHECK(mav_link_ready(&l) == 0, "失联后不可起飞");
    CHECK(mav_link_arm(&l, 1, &out, 255, 190) == MAV_ERR_STATE,
          "失联时解锁被拒绝");

    /* 心跳恢复 -> 回到 UP */
    mav_pack_heartbeat(&hb, &f, 1, 1);
    mav_link_on_frame(&l, &f, 9000);
    CHECK(l.state == MAV_LINK_UP, "心跳恢复 -> 回到 UP");

    /* 模式设置 */
    memset(&hb, 0, sizeof(hb));
    hb.mavlink_version = 3;
    mav_pack_heartbeat(&hb, &f, 1, 1);
    mav_link_on_frame(&l, &f, 9100);
    CHECK(mav_link_set_mode(&l, 2, &out, 255, 190) == MAV_OK, "设置任务模式成功");
    CHECK(l.mode == 2, "模式记录 = %u", l.mode);
    mav_unpack_command_long(&out, &cl);
    CHECK(cl.command == MAV_CMD_DO_SET_MODE && cl.param2 == 2.0f,
          "模式命令 = %u，param2 = %.1f", cl.command, cl.param2);
    CHECK(mav_link_set_mode(&l, 9, &out, 255, 190) == MAV_ERR_PARAM,
          "非法模式 9 -> PARAM");

    /* 边界 */
    mav_link_init(&l, 0);
    CHECK(l.timeout_ms == 3000, "超时传 0 时回落默认 3000 ms");
    mav_link_on_frame(NULL, &f, 0);
    mav_link_tick(NULL, 0);
    CHECK(1, "空指针调用不崩");
    CHECK(mav_link_ready(NULL) == 0, "ready(NULL) = 0");
}

int main(void)
{
    printf("==== MAVLink v2 航电链路库 单元测试 ====\n\n");
    t1_crc();
    t2_encode();
    t3_parse();
    t4_payload();
    t5_link();
    printf("\n==== 结果：%d passed, %d failed ====\n", passed, failed);
    return failed ? 1 : 0;
}
