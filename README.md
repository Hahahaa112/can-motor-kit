# CAN 多关节电机驱动与调试系统

> 基于 STM32 + FreeRTOS + CAN 的四关节电机控制系统（开发中）

## 项目简介

面向四足 / 轮足机器人关节的底层驱动与调试工具链：

- **STM32 侧**：CAN 应用层协议、FreeRTOS 实时任务、电机闭环控制
- **上位机侧**：Python 实时曲线、参数下发、数据录制

## 功能进度

- [ ] CAN 驱动（发送队列 + 接收环形缓冲）
- [ ] 应用层协议（ID 分配 / 8 字节打包 / CRC / 超时重传）
- [ ] FreeRTOS 任务划分（CAN 收发 / 1kHz 控制 / 日志）
- [ ] 单电机位置环 + 速度环
- [ ] 4 电机同步控制
- [ ] Python 上位机（实时曲线 + 参数下发）
- [ ] 总线负载与抖动测试报告

## 硬件

| 部件 | 型号 | 备注 |
|---|---|---|
| 主控 | STM32F407 | 待实验室确认 |
| 电机 | RobStride RS04 | 120 Nm / 15 rad/s / CAN 1 Mbps |
| USB-CAN | 待确认 | |
| 逻辑分析仪 | 待确认 | |

## 目录结构

```
can-motor-kit/
├── firmware/     STM32 固件
├── host/         Python 上位机
├── scripts/      编译、烧录、抓包脚本
├── docs/         协议文档、测试报告、开发日志
└── README.md
```

## 快速开始

```bash
git clone git@github.com:Hahahaa112/can-motor-kit.git
cd can-motor-kit
```

（编译与运行步骤待补充）

**先跑上位机 demo（Linux + SocketCAN）**

```bash
sudo ip link set can0 up type can bitrate 1000000
pip install python-can
python host/robstride.py --interface socketcan --channel can0 --id 1
```

协议细节（CAN ID 结构、MIT 帧编码、参数读写）见 [docs/CAN协议.md](docs/CAN协议.md)。

## 性能指标

| 指标 | 目标 | 实测 |
|---|---|---|
| 控制周期 | 1 kHz | — |
| 周期抖动 | < 100 us | — |
| 4 电机同步误差 | < 1 ms | — |
| 总线负载（500 kbps） | < 50% | — |

## 开发日志

见 [docs/开发日志.md](docs/开发日志.md)。

## License

MIT
