# ESP32-C3 毫米波雷达解析器 (BLE版本)

## 功能概述

- 解析HLK-LD6002雷达的有无人检测和相位数据
- 低通滤波处理总相位（去除心跳干扰）
- 20秒窗口计算呼吸速率，每8秒更新
- 通过BLE低功耗蓝牙发送数据（5Hz）

## 硬件连接

```
HLK-LD6002          ESP32-C3
   VCC  ───────────  3.3V
   GND  ───────────  GND
   TX   ───────────  GPIO4
   RX   ───────────  GPIO5
```

## VSCode编译上传

### 1. 安装PlatformIO

在VSCode扩展商店搜索并安装 "PlatformIO IDE"

### 2. 打开项目

File → Open Folder → 选择 `radar_ble` 文件夹

### 3. 编译上传

- 点击VSCode底部状态栏的 **√** (Build) 编译
- 点击 **→** (Upload) 上传到ESP32-C3
- 点击 **🔌** (Serial Monitor) 打开串口监视器

## Windows连接方法

### 方法一：使用 Bluetooth LE Lab（推荐）

这是Microsoft Store免费应用，界面友好。

1. **配对设备**
   - 打开Windows设置 → 蓝牙和设备
   - 点击"添加设备" → "蓝牙"
   - 找到并配对 `ESP32-Radar`

2. **安装应用**
   - 打开Microsoft Store
   - 搜索 "Bluetooth LE Lab"
   - 安装（免费）

3. **查看数据**
   - 打开Bluetooth LE Lab
   - 点击 "Start" 搜索设备
   - 选择 `ESP32-Radar`
   - 找到服务 `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
   - 点击特征 `6E400003...`（TX）
   - 启用 "Notify" 即可看到实时数据

### 方法二：使用 Serial Bluetooth Terminal

Microsoft Store也有类似的BLE串口终端应用。

### 方法三：USB串口监视器（调试用）

ESP32-C3通过USB连接电脑时，会同时通过USB CDC输出数据。
直接在VSCode的PlatformIO串口监视器中查看即可。

## 输出数据格式

JSON格式，每200ms一条（5Hz）：

```json
{"h":1,"p":0.85,"r":15.5}
```

| 字段 | 含义 | 范围 |
|------|------|------|
| h | 有无人 | 0=无人, 1=有人 |
| p | 滤波后相位 | 浮点数 |
| r | 呼吸速率(BPM) | 6-30，0=无效 |

## 功耗优化说明

| 参数 | 设置 | 说明 |
|------|------|------|
| BLE发送频率 | 5Hz | 每200ms发送一次 |
| BLE功率 | +3dBm | 中等功率，平衡距离和功耗 |
| 连接间隔 | 7.5-15ms | BLE5标准低功耗参数 |

如需进一步降低功耗，可修改 `main.cpp` 中：
```cpp
#define BLE_SEND_INTERVAL_MS 500  // 改为500ms，2Hz
```

## 滤波参数调整

在 `main.cpp` 中修改：

```cpp
#define FILTER_ALPHA 0.15f  // 低通滤波系数
                            // 0.05 = 非常平滑（响应慢）
                            // 0.30 = 较快响应（可能有心跳残留）
```

## 文件结构

```
radar_ble/
├── src/
│   └── main.cpp        # 主程序
├── platformio.ini      # PlatformIO配置
└── README.md           # 本文档
```

## 故障排除

### BLE搜索不到设备
- 确认ESP32-C3已上电并运行（USB串口有输出）
- 重启ESP32-C3
- 在Windows蓝牙设置中删除旧配对记录后重试

### 数据不更新
- 确认雷达模组已连接并上电
- 检查GPIO引脚连接是否正确
- 串口监视器中应能看到同样的数据输出

### 呼吸速率为0
- 需要有人在雷达检测范围内
- 需要至少5秒数据才开始计算
- 剧烈运动会导致数据无效

## BLE服务UUID参考

| 服务/特征 | UUID |
|-----------|------|
| Nordic UART Service | 6E400001-B5A3-F393-E0A9-E50E24DCCA9E |
| RX (写入) | 6E400002-B5A3-F393-E0A9-E50E24DCCA9E |
| TX (通知) | 6E400003-B5A3-F393-E0A9-E50E24DCCA9E |
