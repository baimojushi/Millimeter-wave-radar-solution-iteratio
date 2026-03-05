# ESP32-C3 毫米波雷达解析器（WiFi AP 热点版本）

无需路由器，无需互联网，ESP32 自建热点，电脑直连。

## 硬件连接

```
HLK-LD6002 雷达          ESP32-C3
    VCC  ─────────────  3.3V
    GND  ─────────────  GND
    TX   ─────────────  GPIO4 (RADAR_RX_PIN)
    RX   ─────────────  GPIO5 (RADAR_TX_PIN)
```

## 配置参数（src/main.cpp 第17~19行）

```cpp
#define AP_SSID     "ESP32-Radar"   // 热点名称，可自定义
#define AP_PASSWORD "12345678"      // 热点密码，至少8位；留空则无密码
#define TCP_PORT    8080
```

## 使用步骤

1. 编译上传后，打开 VSCode 串口监视器，看到如下输出：

```
=== ESP32-C3 Radar Parser (WiFi AP Mode) ===
AP Ready. SSID: ESP32-Radar
Password: 12345678
ESP32 IP: 192.168.4.1
TCP Port: 8080
```

2. 电脑打开 WiFi 设置，连接热点 `ESP32-Radar`

3. 连接 TCP（IP 固定为 192.168.4.1）：

**PowerShell（无需安装任何软件）：**
```powershell
$client = New-Object System.Net.Sockets.TcpClient
$client.Connect("192.168.4.1", 8080)
$stream = $client.GetStream()
$reader = New-Object System.IO.StreamReader($stream)
while ($true) { Write-Host $reader.ReadLine() }--
```

## 输出数据格式

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
| t    | 消息类型：p=相位，r=呼吸更新 |
| h    | 有无人：1=有人，0=无人 |
| p    | 滤波后总相位 |
| r    | 呼吸速率 BPM（0=无效） |

## 文件结构

```
radar_wifi_ap/
├── platformio.ini
├── src/
│   └── main.cpp
└── README.md
```
