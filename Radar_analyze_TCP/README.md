# ESP32-C3 毫米波雷达解析器（WiFi TCP版本）

## 硬件连接

```
HLK-LD6002 雷达          ESP32-C3
    VCC  ─────────────  3.3V
    GND  ─────────────  GND
    TX   ─────────────  GPIO4 (RADAR_RX_PIN)
    RX   ─────────────  GPIO5 (RADAR_TX_PIN)
```

## 使用前配置

打开 `src/main.cpp`，修改第17~18行：

```cpp
#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
```

如引脚不同，同步修改第21~22行。

## VSCode 编译上传

1. 安装 PlatformIO IDE 扩展
2. File → Open Folder → 选择 `radar_wifi` 目录
3. 底部状态栏：✓ 编译 / → 上传 / 插头图标 打开串口监视器
4. 串口监视器会打印 ESP32 的 IP 地址

## 电脑端连接

ESP32 作为 TCP Server，端口 8080。

**使用 Windows 内置工具（无需安装）：**

打开 PowerShell，输入：
```powershell
$client = New-Object System.Net.Sockets.TcpClient
$client.Connect("192.168.x.x", 8080)   # 替换为串口监视器显示的IP
$stream = $client.GetStream()
$reader = New-Object System.IO.StreamReader($stream)
while ($true) { Write-Host $reader.ReadLine() }
```

**使用串口监视器（USB调试，无需TCP）：**

ESP32 通过 USB 串口同步输出相同数据，在 VSCode PlatformIO 串口监视器中直接查看。

## 输出数据格式

所有数据为每行一条 JSON，换行符结尾。

**相位数据（10Hz）：**
```json
{"t":"p","h":1,"p":0.85}
```

**呼吸速率更新（每8秒）：**
```json
{"t":"r","h":1,"p":0.85,"r":15.5}
```

| 字段 | 含义 |
|------|------|
| t    | 消息类型：p=相位, r=呼吸更新 |
| h    | 有无人：1=有人, 0=无人 |
| p    | 滤波后总相位（浮点数） |
| r    | 呼吸速率 BPM（0=无效） |

## 参数调整参考

| 参数 | 默认值 | 说明 |
|------|--------|------|
| FILTER_ALPHA | 0.15 | 低通滤波系数，0.05更平滑，0.3响应更快 |
| TCP_SEND_HZ | 10 | 相位数据发送频率（Hz） |
| PHASE_SAMPLE_RATE | 20 | 雷达实际输出相位频率，需实测确认 |
| TCP_PORT | 8080 | TCP服务器端口 |

## 内存占用估算

- 相位缓冲区：400 × 2字节 = 800字节（int16定点数）
- FreeRTOS 两个任务栈：4096 × 2 = 8KB
- WiFi栈：约40KB（ESP32-C3 ROM管理）
