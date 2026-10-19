# ESP32-C3 Wi-Fi Motion

**基于 ESP32-C3 的wifi模块实现运动检测**

利用 Wi-Fi 信道状态信息（CSI）的变化判断周围是否有移动（或者板子自身是否在移动）。板子完成采集和计算，建立本地http服务器，浏览器显示结果可视化

> An experimental ESP32-C3 Wi-Fi CSI motion detector with an on-device web dashboard. No camera or dedicated motion sensor required.

## 能做什么

- 采集 Wi-Fi CSI，计算幅度变化、相位变化、幅度分布熵和 FFT 频带能量特征。
- 融合特征，经中值滤波和 EWMA 平滑，输出运动 / 静止状态。
- 启动后用约 15 秒静止数据标定阈值，再通过滞回和连续帧计数减少状态抖动。
- 提供活动曲线、子载波差分热力图和 JSON API，页面每 500 ms 刷新。
- 内置网页 OTA；保留可选的 AHT21 温湿度、ENS160 空气质量读取功能。

这是实验项目。检测效果受路由器、流量、摆放位置、环境变化影响，目前没有准确率或距离的系统测试。

## 硬件与环境

- ESP32-C3 开发板
- 一个可以连接、允许 ICMP ping 的 2.4 GHz Wi-Fi 路由器。
- 电脑上安装 ESP-IDF。项目原开发环境是 **ESP-IDF v6.0.2**；其他版本尚未验证。
- 可选：AHT21（地址 `0x38`）、ENS160（地址 `0x53`），SDA 接 GPIO0，SCL 接 GPIO1，并与开发板共地，使用适配模块要求的供电与电平。没有这些传感器也可以运行 CSI 检测。

ESP-IDF 安装见[乐鑫官方入门文档](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/get-started/index.html)。CSI 背景见[官方 Wi-Fi CSI 说明](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-guides/wifi-driver/wifi-vendor-features.html#wi-fi-channel-state-information)。

## 快速开始

在已经激活 ESP-IDF 环境的终端里：

```sh
git clone https://github.com/clef233/esp32c3-wifi-motion.git
cd esp32c3-wifi-motion
```

复制 Wi-Fi 配置模板（PowerShell）：

```powershell
Copy-Item main/wifi_config.example.h main/wifi_config.h
```

Linux / macOS：

```sh
cp main/wifi_config.example.h main/wifi_config.h
```

编辑 `main/wifi_config.h`，填入自己的 SSID 和密码。该文件已被 Git 忽略。默认使用 DHCP 获取地址，并以 100 ms 间隔 ping 网关，产生用于采集 CSI 的响应流量；如果网关不响应，可把 `CSI_PING_TARGET` 改为同一局域网内可达且响应 ping 的 IPv4 地址。

```sh
idf.py set-target esp32c3
idf.py build
idf.py -p COM5 flash monitor
```

把 `COM5` 换成你的串口；Linux 常见为 `/dev/ttyACM0` 或 `/dev/ttyUSB0`。`sdkconfig.defaults` 已启用 CSI 和双 OTA 分区。退出串口监视器按 `Ctrl+]`。

串口出现 `Connected! IP address: ...` 后，让电脑 / 手机连接同一局域网，并打开：

| 路径 | 内容 |
| --- | --- |
| `http://<设备IP>/` | 设备地址、芯片温度、可选环境传感器、CSI 帧数 |
| `http://<设备IP>/csi` | 实时运动状态、活动曲线和热力图 |
| `http://<设备IP>/api/csi` | 最新 CSI 幅度与运动特征 JSON |
| `http://<设备IP>/api/sensors` | 传感器 JSON |
| `http://<设备IP>/ota` | 上传应用固件并重启 |

设备刚启动时，保持周围静止约 15 秒，等页面从“预热中”切换后再尝试走动。阈值在这段时间自动计算；当位置或者环境发生变化时应该重新计算阈值。

## 检测流程

```text
ESP32-C3 ping 路由器 → 接收 Wi-Fi 帧的 CSI
  → 提取子载波幅度 / 相位，计算相邻帧差异
  → 融合幅度差、相位差、幅度分布熵和 FFT 频带能量比
  → 8 帧中值滤波 + EWMA
  → 静止基线均值 + 4 倍标准差确定阈值
  → 滞回 + 连续帧确认 → 网页显示运动 / 静止
```

算法集中在 `components/csi_capture/csi_capture.c`。主要参数：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `WARMUP_MS` | 15000 | 标定时长，且至少需要 20 个样本 |
| `ACTIVITY_HISTORY_LEN` | 8 | 中值滤波窗口 |
| `EWMA_ALPHA` | 0.25 | 平滑系数 |
| `THRESHOLD_SIGMA` | 4.0 | 基线标准差倍数 |
| `HYSTERESIS_LOW_RATIO` | 0.70 | 退出运动状态的相对阈值 |
| `MOTION_ENTER_FRAMES` / `MOTION_EXIT_FRAMES` | 2 / 5 | 状态切换连续帧数 |
| `FFT_WINDOW` | 64 | FFT 样本窗口 |

FFT 特征按收到的帧计算，采样间隔不保证均匀；这里的频带能量比是启发式特征，没有标定为物理频率。API 中 `confidence` 是由阈值计算的相对分数，不是概率。

## OTA 与使用范围

首次需要串口烧录。后续可在 `/ota` 上传 `build/esp32c3-wifi-motion.bin`，不要上传 bootloader、分区表或合并固件。单个 OTA 应用分区大小为 `0xF0000`（960 KiB）。

当前 HTTP 页面和 OTA 接口没有登录认证或 HTTPS，应只在可信局域网内使用，不要映射到公网。Wi-Fi 密码会进入本地固件，发布固件前也要使用示例配置。请在有权使用的网络和空间里进行实验。

## 文件结构

```text
main/
  main.c                     # 网页、API、OTA、可选传感器与启动流程
  wifi_config.example.h      # 可公开的配置模板
components/
  csi_capture/               # CSI 接收、特征提取、运动判断
  wifi_web/                  # Wi-Fi 连接与 HTTP 服务
partitions.csv               # 双 OTA 分区
sdkconfig.defaults           # 可复现的项目默认配置
```

编译检查见 GitHub Actions

## 排查

- **一直没有 CSI 数据**：确认连上 2.4 GHz 网络、CSI 配置已启用，网关 / 指定目标能够响应 ping；检查串口日志和首页帧数。
- **标定后一直显示运动 / 不灵敏**：保持设备和路由器固定，静止重启标定；换摆放位置，再调整阈值或特征权重。
- **环境传感器读取失败**：未连接可选传感器时属于正常现象，不影响 CSI 功能。

欢迎提交 issue，记录开发板、ESP-IDF 版本、路由器、摆放方式和观察结果。

## 许可证

[MIT](LICENSE)
