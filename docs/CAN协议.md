# RobStride RS04 CAN 通信协议

> 依据：RobStride 官方手册 + Seeed `RobStride_Control` 参考实现（Python `robstride_dynamics`）
> 适用：RS00 / RS01 / RS02 / RS03 / **RS04** / RS05 / RS06（本文以 RS04 为例）

---

## 1. 物理层

| 项目 | 参数 |
|---|---|
| 总线 | CAN 2.0B |
| 帧格式 | **扩展帧（29-bit ID）** |
| 波特率 | **1 Mbps** |
| 终端电阻 | 总线两端各 120Ω |
| 控制频率 | 建议 100–200 Hz，1 kHz 也可（注意总线负载） |

> 关键：**必须用扩展帧**。标准帧电机不会响应。

---

## 2. CAN ID 结构

29 位 ID 拆成三段：

```
bit 28..24   通信类型 (5 bit)
bit 23..8    数据区 2 / extra_data (16 bit)
bit 7..0     目标电机 ID (8 bit)
```

```c
ext_id = (comm_type << 24) | (extra_data << 8) | motor_id;
```

- `comm_type`：见下表
- `extra_data`：不同命令含义不同（MIT 模式下是**前馈力矩**）
- `motor_id`：电机当前 CAN ID（出厂默认 1，可改）

主机也有一个 `host_id`（建议 0xFD 或 0xFF），放在请求的 `extra_data` 里；电机回复时，回复 ID 的低 8 位是 `host_id`，`extra_data` 的低 8 位才是**电机 ID**。

> 常见坑：回复帧里 `id & 0xFF` 是主机 ID，不是电机 ID；判断是不是自己的电机要看 `(extra_data & 0xFF)`。

---

## 3. 通信类型（comm_type）

| 值 | 名称 | 方向 | 说明 |
|---|---|---|---|
| 0 | 获取设备 ID | 主机→电机 | 返回 UUID |
| **1** | **运控控制（MIT）** | 主机→电机 | 位置/速度/KP/KD + 前馈力矩 |
| **2** | **运控状态反馈** | 电机→主机 | 位置/速度/力矩/温度 |
| **3** | **使能** | 主机→电机 | 上电后必须先使能 |
| **4** | **停止** | 主机→电机 | 关闭输出 |
| 6 | 设置零点 | 主机→电机 | `data[0]=0x01` |
| 7 | 设置 CAN ID | 主机→电机 | extra_data 填新 ID |
| **17** | **读参数** | 主机→电机 | 读单个参数 |
| **18** | **写参数** | 主机→电机 | 写单个参数 |
| 21 | 故障反馈 | 电机→主机 | 8 字节故障码 |
| 22 | 保存参数 | 主机→电机 | 掉电保存 |
| 23 | 设置波特率 | 主机→电机 | |
| 24 | 主动上报 | 电机→主机 | |
| 25 | 设置协议 | 主机→电机 | |

---

## 4. MIT 控制帧（comm_type = 1）

### 4.1 帧结构

```
ID  = (1 << 24) | (torque_u16 << 8) | motor_id
DLC = 8
Data（8 字节，大端）:
  [0:2]  position_u16
  [2:4]  velocity_u16
  [4:6]  kp_u16
  [6:8]  kd_u16
```

**注意：前馈力矩不在数据区，而在 ID 的 `extra_data` 里。**

### 4.2 编码（浮点 → 整数）

```c
p_u16  = clip((p  / P_MAX  + 1.0) * 0x7FFF, 0, 0xFFFF);
v_u16  = clip((v  / V_MAX  + 1.0) * 0x7FFF, 0, 0xFFFF);
kp_u16 = clip( kp / KP_MAX        * 0xFFFF, 0, 0xFFFF);
kd_u16 = clip( kd / KD_MAX        * 0xFFFF, 0, 0xFFFF);
t_u16  = clip((t  / T_MAX  + 1.0) * 0x7FFF, 0, 0xFFFF);
```

### 4.3 RS04 量程常数

| 常数 | 值 | 含义 |
|---|---|---|
| `P_MAX` | `4π ≈ 12.566` | 位置范围 ±4π rad（约 ±2 圈） |
| `V_MAX` | `15.0` | 速度范围 ±15 rad/s |
| `T_MAX` | `120.0` | 力矩范围 ±120 Nm |
| `KP_MAX` | `5000.0` | KP 范围 0–5000 |
| `KD_MAX` | `100.0` | KD 范围 0–100 |

### 4.4 示例（C）

```c
static inline uint16_t f2u(float x, float max) {
    float v = (x / max + 1.0f) * 32767.0f;
    if (v < 0.0f) v = 0.0f;
    if (v > 65535.0f) v = 65535.0f;
    return (uint16_t)v;
}

void rs04_mit(uint8_t id, float p, float v, float kp, float kd, float t) {
    uint32_t ext = (1u << 24) | ((uint32_t)f2u(t, 120.0f) << 8) | id;
    uint8_t d[8];
    uint16_t pu  = f2u(p,  12.5663706f);
    uint16_t vu  = f2u(v,  15.0f);
    uint16_t kpu = f2u(kp, 5000.0f);   // 注意：kp/kd 用 0xFFFF 缩放
    uint16_t kdu = f2u(kd, 100.0f);
    d[0] = pu >> 8;  d[1] = pu & 0xFF;
    d[2] = vu >> 8;  d[3] = vu & 0xFF;
    d[4] = kpu >> 8; d[5] = kpu & 0xFF;
    d[6] = kdu >> 8; d[7] = kdu & 0xFF;
    can_send_ext(ext, d, 8);
}
```

> 坑：kp/kd 用 `0xFFFF` 缩放，位置/速度/力矩用 `0x7FFF`。混用会导致增益差 2 倍。

---

## 5. 状态反馈帧（comm_type = 2）

```
ID  : comm_type=2，extra_data 内含状态位和电机 ID
DLC = 8
Data（8 字节，大端）:
  [0:2]  position_u16
  [2:4]  velocity_u16
  [4:6]  torque_u16
  [6:8]  temperature_u16
```

### 5.1 解码

```c
p  = (p_u16 / 32767.0f - 1.0f) * 12.5663706f;  // rad
v  = (v_u16 / 32767.0f - 1.0f) * 15.0f;        // rad/s
t  = (t_u16 / 32767.0f - 1.0f) * 120.0f;       // Nm
Tc =  t_u16_temp * 0.1f;                        // °C
```

> 力矩在反馈帧里是**无符号 16 位**，用 `u/32767 - 1` 还原到 [-1, 1]。

### 5.2 状态位（在回复 ID 的 extra_data 里）

| bit | 含义 |
|---|---|
| 0–7 | 电机 ID |
| 8 | 欠压 |
| 9 | 过流 |
| 10 | 过温 |
| 11 | 磁编故障 |
| 12 | 堵转 |
| 13 | 未标定 |

---

## 6. 使能 / 停止

```
使能：ID = (3 << 24) | (host_id << 8) | motor_id，Data = 8 字节 0x00
停止：ID = (4 << 24) | (host_id << 8) | motor_id，Data = 8 字节 0x00
```

上电后**必须先使能**，否则 MIT 帧不产生输出（电机会回状态帧，但不动）。

---

## 7. 参数读写

参数读写使用**小端**（与 MIT 数据区的大端相反）。

### 7.1 读参数（comm_type = 17）

```
Data（8 字节）:
  [0:2] param_id   (uint16, 小端)
  [2:4] 0x0000
  [4:8] 0x00000000
```

回复：comm_type=17，`Data[4:8]` 是参数值（小端，按参数类型解释）。

### 7.2 写参数（comm_type = 18）

```
Data（8 字节）:
  [0:2] param_id   (uint16, 小端)
  [2:4] 0x0000
  [4:8] value      (小端，4 字节；不足补 0)
```

### 7.3 常用参数 ID

| 参数 | ID | 类型 | 说明 |
|---|---|---|---|
| `run_mode` | `0x7005` | int8 | 0=MIT，1=位置，2=速度 |
| `loc_ref` | `0x7016` | float | 位置模式目标 |
| `spd_ref` | `0x700A` | float | 速度模式目标 |
| `limit_torque` | `0x700B` | float | 力矩限制 |
| `limit_spd` | `0x7017` | float | 速度限制 |
| `loc_kp` | `0x701E` | float | 位置环 KP |
| `spd_kp` | `0x701F` | float | 速度环 KP |
| `spd_ki` | `0x7020` | float | 速度环 KI |
| `mechPos` | `0x3016` | float | 实测位置 |
| `mechVel` | `0x3017` | float | 实测速度 |
| `torque_fdb` | `0x302C` | float | 实测力矩 |
| `mechOffset` | `0x2005` | float | 机械零偏 |
| `zero_sta` | `0x7029` | uint8 | 零点状态 |

### 7.4 保存参数（comm_type = 22）

```
Data = 8 字节 0x00
```

> 写完 ID / 波特率 / 零偏后必须发一次保存，否则掉电丢失。

---

## 8. 上电初始化顺序（推荐）

```
1. CAN 初始化：1 Mbps，正常模式，扩展帧
2. 发「获取设备 ID」确认电机在线
3. 发「使能」
4. 循环：
     - 发 MIT 帧（100–200 Hz）
     - 收状态帧，解析位置/速度/力矩/温度
     - 检查状态位和故障帧
5. 退出前发「停止」
```

---

## 9. 常见坑

| 现象 | 原因 | 排查 |
|---|---|---|
| 电机完全没反应 | 用了标准帧 | 必须扩展帧 |
| 收到帧但不动 | 没使能 | 先发 comm_type=3 |
| 位置方向反了 | 零偏/方向配置 | 改 `mechOffset` 或用软件取反 |
| 抖动/啸叫 | KP 太大、KD 太小 | 先 KP=0 纯阻尼，再慢慢加 KP |
| 力矩符号反 | `t_u16` 编码错 | 检查 `(t/T_MAX+1)*0x7FFF` |
| 增益差 2 倍 | kp/kd 用了 0x7FFF | 应为 0xFFFF |
| 上电乱动 | 未设零点 | 先 `set_zero` 再使能 |
| 总线错误帧多 | 没加 120Ω 终端电阻 | 两端各一个 |

---

## 10. 参考资料

- RobStride 官方手册（随电机提供）
- Seeed `RobStride_Control`：https://github.com/Seeed-Projects/RobStride_Control
- 本文协议细节以 `python/robstride_dynamics/protocol.py`、`bus.py`、`table.py` 为准
