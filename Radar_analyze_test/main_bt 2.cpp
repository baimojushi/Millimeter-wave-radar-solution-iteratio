/**
 * ESP32 毫米波雷达解析器 v3
 * 算法：差分自相关 + 窄带带通 + Hilbert 瞬时幅度（CV 置信度）
 *
 * ─────────────── Bug 修复记录（本次测试发现）───────────────────────
 *
 * Bug A【关键】雷达帧率误判 → ACF lag 范围完全错误
 *   症状：br 输出大量 46.7、50bpm（钳位上限），实际呼吸信号不可能被检测
 *   根因：
 *     上一次测试手动提取的 730 样本数据恰好是 10Hz，
 *     但雷达实际通过 UART 以 50Hz 发送 TF_TYPE_PHASE 帧。
 *     代码使用 fs=10.0f 做 lag→bpm 换算，误差 5 倍。
 *   验证：
 *     实测 br 组中位宽度 = 10 个 JSON 输出行（@ send 100ms = 1s）
 *     等价 ACF 更新间隔 = 50 样本 / 50Hz = 1s ✓
 *     lag=13 @ fs=10.0 → br=46.2bpm；lag=13 @ 50Hz → 实际 231bpm（身体振动）
 *     lag=53 @ fs=10.0 → br=11.3bpm；lag=53 @ 50Hz → 实际 56.6bpm（高频运动）
 *   修复：
 *     fs = 50.0f
 *     ACF_LAG_MIN = 60   (= 50bpm 上限：T=1.2s×50Hz=60样本，半周期取整)
 *     ACF_LAG_MAX = 333  (= 9bpm 下限：T=6.67s×50Hz=333样本)
 *     ACF_WIN     = 700  (= 14s 窗口，至少包含 2 个完整 9bpm 周期)
 *     ACF_UPDATE_N = 250 (= 5s 更新一次：250/50Hz)
 *
 * Bug B【关键】Hilbert trim 计算错误 → amp/cv 永远为 0
 *   症状：所有 JSON 输出 amp=0.000, cv=0.000
 *   根因：
 *     代码：int trim = HIL_BP_TAP;  // = 63
 *     有效范围：inst[63 : 128-63] = inst[63:65] = 2 个样本
 *     cnt_a < 4 → early return → amp/cv 不更新
 *   修复：
 *     int trim = HIL_BP_TAP / 2;  // = 31
 *     有效范围（新 WIN=700）：inst[31 : 700-31] = 638 个样本 ✓
 *
 * Bug C【次要】带通中心频率参数未随 fs 修正
 *   窄带 BP 带宽 ±0.08Hz 在 50Hz 采样率下仍然有效（不影响频率精度）
 *   但 HIL_BP_TAP=63 对应截止频率应重新评估：
 *     过渡带宽 ≈ 3.3/(Ntap×Ts) = 3.3/(63×0.02) = 2.62Hz → 远大于呼吸带宽，足够
 *     无需增加 tap 数
 *
 * ─────────────── 干扰测试结果解析 ─────────────────────────────────
 *   测试时长：108.8s，1088 个 JSON 行
 *   雷达伪影（|Δraw|>2rad）：11 次，密度约每 10s 一次
 *   raw 峰峰幅度：平均 2.8rad/10样本窗口；最大段 12.14rad（有人大幅运动）
 *   修复前 br 分析：
 *     32% 的时间 br=50.0（钳位）→ 对应 lag=12（最短），高频运动主导
 *     剩余值（11-48bpm）均为身体运动在 fs=10 错误换算下的假读数
 *     conf_acf 最高 0.666（t=41s），但对应的是运动周期而非呼吸
 *   修复后预期：
 *     ACF lag=60~333 @ 50Hz 对应真实 9~50bpm
 *     Hilbert CV < 0.5 时输出有效 br，否则静默
 *     运动伪影期间 CV 升高 → 自动抑制错误输出
 *
 * ─────────────── 完整管线 ─────────────────────────────────────────
 *  原始相位 raw (50Hz)
 *    │
 *    ├─ 轻量去噪 LPF(α=0.69, fc≈23Hz @ 50Hz → 等效去高频噪声)
 *    │
 *    ├─ 差分 d[n] + 归一化自相关 Rdd(k), k=60..333
 *    │    argmax + 抛物线插值 → br_bpm, conf_acf
 *    │
 *    └─ 窄带FIR带通（中心=br/60Hz，±0.08Hz，63tap）
 *           → Hilbert FIR（63tap）→ 瞬时幅度
 *           → CV → conf_hil
 *
 *  JSON 输出（10Hz）：
 *  {"raw":1.23,"br":12.5,"conf_acf":0.35,"amp":1.2,"cv":0.45}
 *    raw=0 且 br=0 → 无人 / 信号无效
 *    br>0 && conf_acf>0.08 && cv<0.8 → 可信
 */

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth is not enabled!"
#endif

// ==================== 用户配置 ====================
#define BT_DEVICE_NAME  "ESP32-Radar-BT"
#define RADAR_RX_PIN    16
#define RADAR_TX_PIN    4
#define RADAR_BAUD      1382400UL
#define SEND_MS         100      // 10Hz JSON 输出

// ==================== 滤波参数 ====================
#define LP_DENOISE_A    0.69f    // 去噪LPF

// ── ACF（已修正为50Hz帧率）──────────────────────────
#define RADAR_FS        50.0f   // 雷达帧率（Hz）★ Bug A 修复
#define ACF_WIN         700     // 窗口 = 14s @ 50Hz（包含≥2个9bpm周期）
#define ACF_LAG_MIN     60      // 对应 50bpm: T=1.2s=60样本 ★ Bug A 修复
#define ACF_LAG_MAX     333     // 对应 9bpm:  T=6.67s=333样本 ★ Bug A 修复
#define ACF_UPDATE_N    250     // 每250样本=5s更新一次 ★ Bug A 修复
#define ACF_CONF_MIN    0.08f   // Rdd峰最低可信阈值

// ── Hilbert 置信度 ───────────────────────────────────
#define HIL_BP_TAP      63      // 带通FIR tap数（延迟31样本=0.62s @ 50Hz）
#define HIL_H_TAP       63      // Hilbert FIR tap数

// ── 伪影检测 ────────────────────────────────────────
#define ART_DIFF        2.0f    // LPF差分阈值（rad/sample）
#define ART_LOCKOUT     20      // 伪影后锁定样本数

// ==================== TinyFrame ====================
#define TF_SOF          0x01
#define TF_TYPE_HUMAN   0x0F09
#define TF_TYPE_PHASE   0x0A13

typedef enum {
    ST_SOF, ST_ID_H, ST_ID_L, ST_LEN_H, ST_LEN_L,
    ST_TYPE_H, ST_TYPE_L, ST_HEAD_CKSUM, ST_DATA, ST_DATA_CKSUM
} TF_State;

// ==================== 全局 ====================
static BluetoothSerial BT;
static volatile bool   spp_ready = false;

static TF_State tf_state = ST_SOF;
static uint16_t tf_id, tf_len, tf_type;
static uint8_t  tf_data[64] = {};
static uint16_t tf_data_idx = 0;
static uint8_t  xor_acc     = 0;

static volatile float v_raw      = 0.0f;
static volatile float v_br       = 0.0f;
static volatile float v_conf_acf = 0.0f;
static volatile float v_amp      = 0.0f;
static volatile float v_cv       = 0.0f;
static volatile bool  v_human    = false;

// 降噪LPF
static float lp_y = 0.0f, lp_prev = 0.0f;
static bool  lp_init = false, lp_prev_init = false;

// ACF 环形缓冲
static float*    acf_buf        = nullptr;   // 在setup()中动态分配 (700×4=2.8KB)
static uint16_t  acf_wr         = 0;
static uint16_t  acf_cnt        = 0;
static uint16_t  acf_since_upd  = 0;

// Hilbert 带通系数
static float     bp_h[HIL_BP_TAP]  = {};
static float     hil_h[HIL_H_TAP]  = {};
static bool      hil_ready = false;
static float     last_f0   = -1.0f;

// 伪影
static uint16_t art_lockout = 0;

// ==================== SPP ====================
static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t*) {
    if      (event == ESP_SPP_SRV_OPEN_EVT) spp_ready = true;
    else if (event == ESP_SPP_CLOSE_EVT)    spp_ready = false;
}

// ==================== 工具 ====================
static float bytes_to_float(const uint8_t* b) {
    uint32_t u = (uint32_t)b[0] | ((uint32_t)b[1]<<8)
               | ((uint32_t)b[2]<<16) | ((uint32_t)b[3]<<24);
    float f; memcpy(&f, &u, 4); return f;
}
static void tf_reset() { tf_state = ST_SOF; tf_data_idx = 0; xor_acc = 0; }
static float hann(int i, int N) {
    return 0.5f - 0.5f * cosf((float)M_PI * 2.0f * i / (N - 1));
}

// ==================== 构建带通FIR（随估计频率动态更新）====================
static void build_bp(float f0_hz) {
    if (fabsf(f0_hz - last_f0) < 0.005f && hil_ready) return;
    last_f0 = f0_hz;
    float fl = f0_hz - 0.08f; if (fl < 9.0f/60.0f)  fl = 9.0f/60.0f;
    float fh = f0_hz + 0.08f; if (fh > 50.0f/60.0f) fh = 50.0f/60.0f;
    int half = HIL_BP_TAP / 2;
    for (int i = 0; i < HIL_BP_TAP; i++) {
        int n = i - half;
        float v = (n == 0) ? 2.0f*(fh-fl)/RADAR_FS
                           : (sinf(2.0f*(float)M_PI*fh/RADAR_FS*n)
                             -sinf(2.0f*(float)M_PI*fl/RADAR_FS*n))
                             / ((float)M_PI*n);
        bp_h[i] = v * hann(i, HIL_BP_TAP);
    }
    // Hilbert 系数（固定）
    int hh = HIL_H_TAP / 2;
    for (int i = 0; i < HIL_H_TAP; i++) {
        int n = i - hh;
        float v = (n != 0 && n%2 == 1) ? 2.0f/((float)M_PI*n) : 0.0f;
        hil_h[i] = v * hann(i, HIL_H_TAP);
    }
    hil_ready = true;
}

// ==================== ACF + Hilbert 联合计算 ====================
// 使用静态/动态分配避免栈溢出（ACF_WIN=700 时差分数组=699×4=2.8KB）
static float* s_diff   = nullptr;   // 在setup中分配
static float* s_bp_out = nullptr;
static float* s_hil_out = nullptr;
static float* s_lp_lin  = nullptr;

static void update_rate() {
    if (!s_diff) return;

    // ── 差分自相关 ────────────────────────────────────
    const int M = ACF_WIN - 1;
    for (int i = 0; i < M; i++) {
        float cur  = acf_buf[(acf_wr + i + 1) % ACF_WIN];
        float prev = acf_buf[(acf_wr + i)     % ACF_WIN];
        s_diff[i]  = cur - prev;
    }
    float r0 = 0.0f;
    for (int i = 0; i < M; i++) r0 += s_diff[i] * s_diff[i];
    if (r0 < 1e-6f) { v_br = 0; v_conf_acf = 0; return; }

    float best_r = -1.0f; int best_k = ACF_LAG_MIN;
    // 注意：ACF_LAG_MAX=333，全遍历 333×700≈233k MAC，@240MHz≈1ms，可接受
    for (int k = ACF_LAG_MIN; k <= ACF_LAG_MAX; k++) {
        float rk = 0.0f;
        int n = M - k;
        for (int i = 0; i < n; i++) rk += s_diff[i] * s_diff[i+k];
        float rkn = rk / r0;
        if (rkn > best_r) { best_r = rkn; best_k = k; }
    }
    v_conf_acf = best_r;
    if (best_r < ACF_CONF_MIN) { v_br = 0; return; }

    // 抛物线插值（需要缓存R[best_k-1..best_k+1]，重算3点即可）
    float y1 = best_r;
    float y0 = y1, y2 = y1;
    if (best_k > ACF_LAG_MIN) {
        float rk=0; int n=M-(best_k-1);
        for(int i=0;i<n;i++) rk+=s_diff[i]*s_diff[i+best_k-1];
        y0 = rk/r0;
    }
    if (best_k < ACF_LAG_MAX) {
        float rk=0; int n=M-(best_k+1);
        for(int i=0;i<n;i++) rk+=s_diff[i]*s_diff[i+best_k+1];
        y2 = rk/r0;
    }
    float pk  = (float)best_k;
    float den = y0 - 2.0f*y1 + y2;
    if (fabsf(den) > 1e-6f) pk += 0.5f*(y0-y2)/den;

    float br_hz  = RADAR_FS / pk;          // ★ Bug A 修复：使用 RADAR_FS
    float br_bpm = br_hz * 60.0f;
    v_br = (br_bpm >= 9.0f && br_bpm <= 50.0f) ? br_bpm : 0.0f;
    if (v_br == 0.0f) return;

    // ── 窄带BP → Hilbert → CV ──────────────────────────
    build_bp(br_hz);
    if (!hil_ready) return;

    // 线性化环形缓冲
    for (int i = 0; i < ACF_WIN; i++)
        s_lp_lin[i] = acf_buf[(acf_wr + i) % ACF_WIN];

    // 带通滤波
    int half_bp = HIL_BP_TAP / 2;
    for (int i = 0; i < ACF_WIN; i++) {
        float s = 0.0f;
        for (int j = 0; j < HIL_BP_TAP; j++) {
            int idx = i + j - half_bp;
            float xv = (idx < 0) ? s_lp_lin[0]
                      : (idx >= ACF_WIN) ? s_lp_lin[ACF_WIN-1]
                      : s_lp_lin[idx];
            s += bp_h[j] * xv;
        }
        s_bp_out[i] = s;
    }

    // Hilbert
    int half_h = HIL_H_TAP / 2;
    for (int i = 0; i < ACF_WIN; i++) {
        float s = 0.0f;
        for (int j = 0; j < HIL_H_TAP; j++) {
            int idx = i + j - half_h;
            float xv = (idx < 0) ? s_bp_out[0]
                      : (idx >= ACF_WIN) ? s_bp_out[ACF_WIN-1]
                      : s_bp_out[idx];
            s += hil_h[j] * xv;
        }
        s_hil_out[i] = s;
    }

    // ★ Bug B 修复：trim = HIL_BP_TAP / 2（原为 HIL_BP_TAP）
    int trim = HIL_BP_TAP / 2;   // = 31，而不是 63
    float sum_a = 0.0f, sum_a2 = 0.0f;
    int cnt_a = 0;
    for (int i = trim; i < ACF_WIN - trim; i++) {
        float a = sqrtf(s_bp_out[i]*s_bp_out[i] + s_hil_out[i]*s_hil_out[i]);
        sum_a  += a;
        sum_a2 += a * a;
        cnt_a++;
    }
    if (cnt_a < 4) return;
    float mean_a = sum_a / cnt_a;
    float var_a  = fmaxf(0.0f, sum_a2/cnt_a - mean_a*mean_a);
    v_amp = mean_a;
    v_cv  = sqrtf(var_a) / (mean_a + 1e-6f);
}

// ==================== 样本处理 ====================
static void process_sample(float raw) {
    if (!lp_init) { lp_y = raw; lp_init = true; lp_prev = raw; lp_prev_init = true; return; }
    lp_y = LP_DENOISE_A * raw + (1.0f - LP_DENOISE_A) * lp_y;

    if (lp_prev_init && fabsf(lp_y - lp_prev) > ART_DIFF) {
        acf_cnt = acf_wr = acf_since_upd = 0;
        art_lockout = ART_LOCKOUT;
        v_br = 0.0f; v_conf_acf = 0.0f; v_amp = 0.0f; v_cv = 0.0f;
        lp_prev = lp_y; return;
    }
    lp_prev = lp_y;

    if (art_lockout > 0) { art_lockout--; return; }

    if (acf_buf) {
        acf_buf[acf_wr] = lp_y;
        acf_wr = (acf_wr + 1) % ACF_WIN;
        if (acf_cnt < ACF_WIN) acf_cnt++;
        acf_since_upd++;

        if (acf_cnt >= ACF_WIN && acf_since_upd >= ACF_UPDATE_N) {
            acf_since_upd = 0;
            update_rate();
        }
    }
}

// ==================== TF帧处理 ====================
static void on_frame() {
    switch (tf_type) {
        case TF_TYPE_HUMAN: {
            if (tf_len >= 2) {
                bool det = (tf_data[0] | ((uint16_t)tf_data[1]<<8)) == 0x0001;
                if (!det && v_human) {
                    lp_init = false; lp_prev_init = false; hil_ready = false;
                    acf_cnt = acf_wr = acf_since_upd = art_lockout = 0;
                    v_br = v_conf_acf = v_amp = v_cv = 0.0f;
                }
                v_human = det;
            }
            break;
        }
        case TF_TYPE_PHASE: {
            if (tf_len >= 4) {
                float raw = bytes_to_float(&tf_data[0]);
                v_raw = raw;
                process_sample(raw);
            }
            break;
        }
    }
}

// ==================== TF解析 ====================
static void parse_byte(uint8_t b) {
    switch (tf_state) {
        case ST_SOF:       if(b==TF_SOF){xor_acc=b;tf_state=ST_ID_H;} break;
        case ST_ID_H:      xor_acc^=b;tf_id=(uint16_t)b<<8;tf_state=ST_ID_L; break;
        case ST_ID_L:      xor_acc^=b;tf_id|=b;tf_state=ST_LEN_H; break;
        case ST_LEN_H:     xor_acc^=b;tf_len=(uint16_t)b<<8;tf_state=ST_LEN_L; break;
        case ST_LEN_L:
            xor_acc^=b;tf_len|=b;
            if(tf_len>sizeof(tf_data)){tf_reset();break;}
            tf_state=ST_TYPE_H; break;
        case ST_TYPE_H:    xor_acc^=b;tf_type=(uint16_t)b<<8;tf_state=ST_TYPE_L; break;
        case ST_TYPE_L:    xor_acc^=b;tf_type|=b;tf_state=ST_HEAD_CKSUM; break;
        case ST_HEAD_CKSUM:
            if(b!=(uint8_t)(~xor_acc)){tf_reset();break;}
            tf_data_idx=0;xor_acc=0;
            tf_state=(tf_len>0)?ST_DATA:ST_DATA_CKSUM; break;
        case ST_DATA:
            tf_data[tf_data_idx++]=b;xor_acc^=b;
            if(tf_data_idx>=tf_len)tf_state=ST_DATA_CKSUM; break;
        case ST_DATA_CKSUM:
            if(tf_len==0||b==(uint8_t)(~xor_acc))on_frame();
            tf_reset(); break;
    }
}

// ==================== UART任务 ====================
static void uart_task(void*) {
    uint8_t buf[512];
    while (true) {
        int n = Serial2.available();
        if (n > 0) {
            n = Serial2.readBytes(buf, min(n, (int)sizeof(buf)));
            for (int i = 0; i < n; i++) parse_byte(buf[i]);
        } else { vTaskDelay(1); }
    }
}

// ==================== 发送任务 ====================
static void send_task(void*) {
    char msg[128];
    bool was_ready = false;
    uint32_t last_send = 0;

    while (true) {
        bool ready = spp_ready;
        uint32_t now = millis();

        if (ready && !was_ready) {
            BT.println("{\"info\":\"ESP32 Radar v3 Ready (50Hz mode)\"}");
            Serial.println("[BT] SPP open");
        } else if (!ready && was_ready) {
            Serial.println("[BT] SPP closed");
        }
        was_ready = ready;

        if (now - last_send >= SEND_MS) {
            last_send = now;
            snprintf(msg, sizeof(msg),
                "{\"raw\":%.3f,\"br\":%.1f,\"conf_acf\":%.3f,\"amp\":%.3f,\"cv\":%.3f}\n",
                v_raw, v_br, v_conf_acf, v_amp, v_cv);
            if (ready) BT.print(msg);
            Serial.print(msg);
        }
        vTaskDelay(5);
    }
}

// ==================== setup ====================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("=== ESP32 Radar v3 | 50Hz DiffACF + Hilbert-CV ===");
    Serial.printf("ACF: WIN=%d(%.1fs) LAG=%d~%d(%.1f~%.1fbpm) UPDATE=%d(%.1fs)\n",
        ACF_WIN, (float)ACF_WIN/RADAR_FS,
        ACF_LAG_MIN, ACF_LAG_MAX,
        RADAR_FS/(float)ACF_LAG_MAX*60.0f, RADAR_FS/(float)ACF_LAG_MIN*60.0f,
        ACF_UPDATE_N, (float)ACF_UPDATE_N/RADAR_FS);
    Serial.println("Reliable: br>0 && conf_acf>0.08 && cv<0.8");

    // 动态分配大缓冲区（避免 BSS 静态分配超出限制）
    acf_buf    = (float*)malloc(ACF_WIN * sizeof(float));
    s_diff     = (float*)malloc((ACF_WIN-1) * sizeof(float));
    s_lp_lin   = (float*)malloc(ACF_WIN * sizeof(float));
    s_bp_out   = (float*)malloc(ACF_WIN * sizeof(float));
    s_hil_out  = (float*)malloc(ACF_WIN * sizeof(float));

    if (!acf_buf || !s_diff || !s_lp_lin || !s_bp_out || !s_hil_out) {
        Serial.println("MALLOC FAILED! Check free heap.");
        while (1) delay(1000);
    }
    memset(acf_buf, 0, ACF_WIN * sizeof(float));
    Serial.printf("Heap after alloc: %u bytes free\n", ESP.getFreeHeap());

    Serial2.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    Serial2.setRxBufferSize(8192);   // 50Hz帧，增大缓冲区

    BT.register_callback(spp_callback);
    if (!BT.begin(BT_DEVICE_NAME)) {
        Serial.println("BT FAILED"); delay(2000); ESP.restart();
    }
    Serial.printf("BT: %s\n", BT_DEVICE_NAME);

    xTaskCreatePinnedToCore(uart_task, "UART", 8192, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(send_task, "Send", 4096, NULL, 1, NULL, 1);
}

void loop() { vTaskDelay(5000); }
