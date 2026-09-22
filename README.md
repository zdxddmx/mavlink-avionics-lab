# mavlink-avionics-lab — MAVLink v2 航电链路库

> 面向**无人机航电**方向的嵌入式软件：把 **飞控通信链路**（MAVLink v2 帧编解码、
> 流式解析、校验）与**链路状态管理**（心跳监控、解锁/上锁、模式切换、命令应答匹配）
> 做成可测、可仿真的 C 库。
>
> 纯 **C99**，零第三方依赖 —— **不需要真机、不需要 PX4/ArduPilot 环境即可完整验证**。

![c](https://img.shields.io/badge/C-C99-blue)
![deps](https://img.shields.io/badge/dependencies-none-green)
![tests](https://img.shields.io/badge/tests-74%20passed-brightgreen)

---

## 1. 为什么做这个

航电岗的日常工作绕不开两件事：**飞控怎么和地面站/载荷说话**，以及**怎么判断链路还好不好**。
这两件事的底层就是 MAVLink 帧格式与链路状态机：

| 岗位关注点 | 本项目对应实现 |
|---|---|
| 帧编解码 | v1/v2 两种帧头、24 位消息 ID、载荷长度校验、小端校验和 |
| **CRC_EXTRA** | MAVLink 特有机制：校验要**逐字节累积 CRC_EXTRA**，错一个字节整条链路不通 |
| 串口流式解析 | **逐字节喂入**（贴合串口中断/DMA 空闲中断的真实用法），处理粘包、半包、噪音重同步 |
| 飞控与载荷接口 | `HEARTBEAT` / `COMMAND_LONG` / `COMMAND_ACK` / `STATUSTEXT` 的**线格式字段重排** |
| 链路健康判断 | 心跳超时判定、**失联自动上锁**、解锁前置条件、命令应答匹配 |
| 安全性 | 链路不通时**拒绝**发解锁命令；失联后不允许起飞 |

---

## 2. 快速开始

```bash
make test          # 74 项单元测试
# 手动：
gcc -std=c99 -O2 -Wall -Wextra -Wpedantic -Iinclude \
    src/mavlink.c test/test_mavlink.c -o build/mavtest && ./build/mavtest
```

最小用例：

```c
#include "mavlink.h"

MavLink link;
mav_link_init(&link, 3000);              /* 3 秒心跳超时 */

MavParser p;
mav_parser_init(&p);

/* 串口每收到一个字节就喂进来（中断/DMA 空闲中断里这样写） */
MavFrame f;
if (mav_parse_byte(&p, byte, &f) == 1) {
    mav_link_on_frame(&link, &f, now_ms());
}

mav_link_tick(&link, now_ms());          /* 周期调用，判心跳超时 */

if (mav_link_ready(&link)) {             /* 在线 + 已解锁 才允许起飞 */
    MavFrame cmd;
    mav_link_arm(&link, 1, &cmd, 255, 190);
    uint8_t buf[MAV_MAX_FRAME];
    int n = mav_encode(&cmd, buf, sizeof(buf));
    uart_send(buf, n);
}
```

---

## 3. 目录结构

```
include/mavlink.h   帧结构、CRC、解析器、载荷编解码、链路状态机接口
src/mavlink.c       全部实现（单文件，方便直接搬进固件工程）
test/test_mavlink.c 74 项单元测试
```

---

## 4. 实测结果

```
==== 结果：74 passed, 0 failed ====
```

覆盖：

- **CRC**：CRC_EXTRA 表值（HEARTBEAT=50 / COMMAND_LONG=152 / STATUSTEXT=83）、
  未知消息拒绝、EXTRA 参与校验、同内容校验稳定
- **打包**：v2（10+len+2）与 v1（6+len+2）帧头差异、缓冲不足返回 NOMEM、
  255 字节上限
- **流式解析**：逐字节喂入**只在最后一个字节成帧**、**粘包**（两帧连发解出 2 帧）、
  **半包**不出帧、**64 字节噪音后重新同步**、CRC 错不出帧、长时间垃圾输入不越界
- **载荷线格式**：HEARTBEAT 的 `custom_mode` 小端在前、COMMAND_LONG 的
  7 个 float 在前的**字段重排**、float 往返一致
- **链路状态机**：DOWN→UP→TIMEOUT→UP 全迁移、**失联自动上锁**、
  链路不通时解锁被拒、命令应答匹配、模式设置与非法值拒绝、空指针不崩

---

## 5. 开发中踩到并修掉的问题

| # | 问题 | 现象 | 根因与修复 |
|---|---|---|---|
| 1 | **CRC 错时"只丢 1 字节"会导致解析器卡死** | 坏帧（载荷含 `0xFD`）之后的**好帧永远解不出来** | 丢掉 STX 后解析器会一路扫到载荷里的 `0xFD`，把它当新帧头，再读下一个 `0xFD` 当作长度 **253** → **死等一个永远凑不齐的巨型帧**。改成 **CRC 错时丢弃整帧**（长度字段已精确给出帧长，不会多丢）——这也是 MAVLink 官方解析器的做法 |
| 2 | 测试把 `mav_encode` 的返回值当状态码 | "255 以内载荷可编码" 断言失败 | `mav_encode` 返回的是**写入长度**（正数），不是 `MAV_OK`(0)，测试断言写错 |
| 3 | 手工构造 COMMAND_ACK 时只填了低字节 | 应答匹配出 `cmd=144`（应为 400） | `command` 是 uint16，高字节没填。`400 = 0x0190`，只写 `0x90` 就是 144 —— 测试的构造错误 |
| 4 | 编译告警：`f->len > MAV_MAX_PAYLOAD` 恒为假 | `-Wtype-limits` | `len` 是 `uint8_t`，上限 255 恰好等于 `MAV_MAX_PAYLOAD`，该判断多余，删掉 |
| 5 | 测试里 CJK 说明文字用了英文双引号 | 编译报 `expected ')'` | 把引号换成中文书名号式表述 |

> 第 1 条是**实现里的真实设计错误**，由单元测试抓出来 —— 而且它只在
> "载荷里恰好含 `0xFD`" 时暴露，正是那种**只在现场偶发、实验室难复现**的问题。

---

## 6. 边界说明（重要）

- 本项目是**按 MAVLink v2 规范自行实现的收发与链路管理**，
  **不依赖官方 mavlink 生成器**，**消息集只覆盖航电链路常用的 9 条**
  （HEARTBEAT / SYS_STATUS / PING / ATTITUDE / GLOBAL_POSITION / GPS_RAW_INT /
  COMMAND_LONG / COMMAND_ACK / STATUSTEXT）
- **未实现 MAVLink 2 签名**（`incompat` 标志位保留但未做签名校验）
- **未做**：消息流速率控制、参数协议（PARAM_*）完整实现、任务协议（MISSION_*）、
  日志下载协议、`ftp` 协议
- **未接入真实飞控**：没有跑过 PX4/ArduPilot 的 SITL，也**没有实飞**
- 链路状态机是**简化模型**（4 档模式、单对端），不是完整 ArduPilot 模式体系

**下一步可做**：接 PX4 SITL 跑通端到端；补 MAVLink 2 签名；加 PARAM_* 与 MISSION_*；
做消息流限速与丢包统计。

---

## 7. 许可

MIT
