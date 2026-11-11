/*
 * MiniCar - ESP32-S3 红外避障 + 手机遥控（FreeRTOS 多线程版）
 *
 * 线程划分：
 *   taskIrSensor  (core 1) : 5ms 采样 + 25/80ms 不对称消抖，只负责“看”
 *   taskDrive     (core 1) : 10ms tick 的非阻塞状态机，只负责“动”
 *   taskIndicator (core 0) : 状态灯 / WS2812 显示，只负责“说”
 *   taskComm      (core 0) : 通信层，只负责“听手机的”
 *                            COMM_BLE=1 -> BLE GATT（手机 Chrome 走 Web Bluetooth）
 *                            COMM_WIFI=1 -> WiFi AP + HTTP/WebSocket（原方案）
 *
 * 避障策略（左/右 IR）：
 *   左边有障碍 -> 后退一小段 -> 右转 -> 前进
 *   右边有障碍 -> 后退一小段 -> 左转 -> 前进
 *   两边都有   -> 后退长一点 -> 随机大角度转 -> 前进（连续被堵会越转越多）
 *
 * 手机控制：
 *   BLE 模式（推荐）：手机 Chrome 打开托管的 HTTPS 页面 -> 点「连接小车」选 MiniCar
 *   WiFi 模式：连热点 MiniCar-xxxx，浏览器打开 192.168.4.1
 *   上电默认「手动 + 停止」：不动就一直停着，防止开机自己跑
 *   只有在网页上点「切到自动」才会开始自动避障
 *   手动模式下 IR 仍保留安全保护：检出障碍禁止继续前进，只允许后退/转向
 */

/* ---------------- 功能开关 ---------------- */
// OLED 显示总开关：0 = 完全编译掉（不占引脚、不建任务、不占 flash）
// 调试遥控时先关掉它，确认问题是否出在显示上
#define ENABLE_OLED 0

// 通信方式二选一（ESP32-S3 只有 BLE，没有经典蓝牙）
//   COMM_BLE  1 : 手机 Chrome 走 Web Bluetooth 直连，不用连热点
//   COMM_WIFI 1 : 原来的 WiFi 热点 + 网页方案，保留以便随时切回
// 两个都设 1 也能跑（WiFi/BLE 共存），但射频互抢会拉高延迟，一般没必要
#define COMM_BLE   1
#define COMM_WIFI  0

#include <Adafruit_NeoPixel.h>
#if ENABLE_OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif
#if COMM_WIFI
#include <WiFi.h>
#include <esp_http_server.h>
#endif
#if COMM_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <string.h>
#include <stdlib.h>

#if COMM_WIFI
#include "web_page.h"
#endif

/* ---------------- 引脚 ---------------- */
#define PIN_STRIP   48
#define NUMPIXELS   8
#define BRIGHTNESS  50

Adafruit_NeoPixel strip(NUMPIXELS, PIN_STRIP, NEO_GRB + NEO_KHZ800);

const int led_green = 13;   // 左 IR 指示
const int led_red   = 14;   // 右 IR 指示

const int ir_left  = 3;
const int ir_right = 20;

// 前向 IR 预留（本项目暂不使用）
// const int front_ir_left  = 18;
// const int front_ir_right = 17;

const int motor_left_in1  = 21;
const int motor_left_in2  = 46;
const int motor_right_in3 = 11;
const int motor_right_in4 = 10;

/* ---------------- 蜂鸣器 ---------------- */
#define BUZZER_PIN     6
// 1 = 有源蜂鸣器（高电平就响，内部自带振荡）
// 0 = 无源蜂鸣器（要方波，用 LEDC 出 2.7kHz）
#define BUZZER_ACTIVE  0
#define BUZZER_ACTIVE_ON_LEVEL HIGH   // 有源蜂鸣器“响”对应电平；板上带三极管反相就改 LOW
#define BUZZER_FREQ    2700     // 无源蜂鸣器谐振频率，2~2.7kHz 最响

/* ---------------- SPI OLED 0.97" (SSD1306 128x64，6 针) ---------------- */
#if ENABLE_OLED
// 6 针 = SPI 无 CS（模块内部把 CS 接地）。丝印的 D0 长得像 DO、D1 像 DI：
//   D0 = 时钟 SCK      D1 = 数据 MOSI
//
// ★★ 黑屏排查：只改下面这个数字，重新烧录，接线按对应表格来 ★★
//   0 = D0->IO17  D1->IO19  DC->IO5  RST->IO18   ← 你现在的接法
//   1 = D0->IO19  D1->IO17  DC->IO5  RST->IO18   ← 怀疑 D0/D1 接反
//   2 = D0->IO4   D1->IO7   DC->IO5  RST->IO15   ← 换排针，避开 USB 脚 IO19/IO20
#define OLED_MODE 0

#define OLED_W   128
#define OLED_H   64

#if   OLED_MODE == 1
  #define OLED_CLK   19
  #define OLED_MOSI  17
  #define OLED_DC    5
  #define OLED_RST   18
#elif OLED_MODE == 2
  #define OLED_CLK   4
  #define OLED_MOSI  7
  #define OLED_DC    5
  #define OLED_RST   15
#else
  #define OLED_CLK   17
  #define OLED_MOSI  19
  #define OLED_DC    5
  #define OLED_RST   18
#endif

#define OLED_CS    -1        // -1 = 无 CS（模块内部接地）

// 软件 SPI 构造，参数顺序：宽, 高, MOSI, CLK, DC, RST, CS
Adafruit_SSD1306 display(OLED_W, OLED_H, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RST, OLED_CS);
#endif  // ENABLE_OLED

/* ---------------- 可调参数 ---------------- */
// IR 检测到障碍时输出的电平。若实测方向相反，把 HIGH 改成 LOW
#define IR_ACTIVE_LEVEL LOW

#define IR_SAMPLE_MS      5     // 采样周期
#define IR_DEBOUNCE_ON_MS  25   // 检出障碍：需持续 25ms 才认（快，防撞）
#define IR_DEBOUNCE_OFF_MS 80   // 障碍消失：需持续 80ms 才认（慢，挡住回弹/反光抖动）

#define SETTLE_MS        150    // 停稳后再等这么久才换向（在斜坡+死区之外额外的间隙）
#define BACKUP_MS_ONE    250    // 单侧障碍后退时长
#define BACKUP_MS_BOTH   400    // 双侧障碍后退时长
#define TURN_MS_ONE      450    // 单侧障碍转向时长
#define TURN_MS_BOTH     700    // 双侧障碍转向时长
#define TURN_STEP_MS     250    // 每连续被堵一次，转向时间加长

/* ---------------- 电机 PWM 参数 ---------------- */
// ESP32-S3 LEDC：4 个定时器 × 8 通道，4 路电机占 4 个通道，互不冲突
#define MOTOR_PWM_FREQ   2000     // 2kHz：L298N 这类慢速驱动友好；换 TB6612/DRV8833 可提到 20000 消除啸叫
#define MOTOR_PWM_RES    10       // 10bit -> 占空比 0~1023
#define MOTOR_DUTY_MAX   ((1 << MOTOR_PWM_RES) - 1)

// 换向保护：占空比斜坡 + 过零死区，避免硬切造成电流冲击和齿轮打齿
#define MOTOR_RAMP_ENABLE 0
#define RAMP_UP_STEP     6        // 每 10ms 上升 6%（0→60% 约 100ms）
#define RAMP_DOWN_STEP   10       // 每 10ms 下降 10%（60→0 约 60ms）
#define MOTOR_DEAD_MS    180      // 过零后必须静默这么久，才允许反向或重新起步

#define SPEED_FWD        50       // 前进速度 %
#define SPEED_BACK       50       // 后退速度 %
#define SPEED_TURN       40       // 原地转向速度 %（要克服静摩擦，给大一点）
#define TRIM_LEFT        100      // 左轮修正 %（跑偏时微调，100 = 不修正）
#define TRIM_RIGHT       100      // 右轮修正 %

/* ---------------- 手机遥控参数 ---------------- */
#define AP_SSID_PREFIX   "MiniCar-"      // 实际 SSID 后面拼芯片 ID 后 4 位
#define AP_PASSWORD      "12345678"      // WPA2 要求至少 8 位
#define AP_CHANNEL       6
#define AP_MAX_CLIENTS   4
#define REMOTE_TIMEOUT_MS 600            // 超过这么久没收到指令 -> 停车（防断线乱冲）
#define STATUS_PUSH_MS    500            // 状态回传周期

/* ---------------- 内部常量 ---------------- */
#define IR_LEFT_BIT   0x01
#define IR_RIGHT_BIT  0x02
#define IR_BOTH       (IR_LEFT_BIT | IR_RIGHT_BIT)

enum Phase : uint8_t {
  P_FORWARD = 0,
  P_STOP1,      // 倒车前停顿
  P_BACK,       // 后退
  P_STOP2,      // 转向之前停顿
  P_TURN,       // 原地转向
  P_STOP3,      // 转完回正
  P_IDLE        // 手动模式下的静止
};

static QueueHandle_t irQueue = nullptr;   // 长度 1，永远只存最新采样结果
static volatile uint8_t g_phase = P_IDLE; // 给指示线程看：上电即静止

/* ---------------- 遥控共享状态 ---------------- */
enum : uint8_t { MODE_AUTO = 0, MODE_MANUAL = 1 };

// 只对 32 位对齐变量做单次读写，Xtensa 上是原子的，不加锁也读不到“半截”值
// 上电默认手动：g_lastCmd=0 会让第一条 stale 判断直接成立 -> 立即停车
static volatile uint8_t  g_mode    = MODE_MANUAL;
static volatile int      g_remoteL = 0;    // 手机下发的左轮速度 -100~100
static volatile int      g_remoteR = 0;
static volatile uint32_t g_lastCmd = 0;    // 最近一次收到指令的时刻

/* ---------------- 蜂鸣器 ---------------- */
enum : uint8_t { BUZZ_OFF = 0, BUZZ_BACK = 1, BUZZ_ALARM = 2 };
static volatile uint8_t g_buzzer = BUZZ_OFF;   // 由 drive 线程设，indicator 线程执行

static inline void buzzerWrite(bool on) {
#if BUZZER_ACTIVE
  digitalWrite(BUZZER_PIN, on ? BUZZER_ACTIVE_ON_LEVEL
                             : (BUZZER_ACTIVE_ON_LEVEL == HIGH ? LOW : HIGH));
#else
  ledcWrite(BUZZER_PIN, on ? 128 : 0);   // 8bit 分辨率，50% 占空比 = 方波发声
#endif
}

static void buzzerInit() {
#if BUZZER_ACTIVE
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerWrite(false);
  Serial.println("[BUZ] GPIO6 有源蜂鸣器，直流电平驱动");
#else
  if (!ledcAttach(BUZZER_PIN, BUZZER_FREQ, 8)) {
    Serial.println("[ERR] buzzer LEDC attach failed");
  }
  ledcWrite(BUZZER_PIN, 0);
  Serial.printf("[BUZ] GPIO6 无源蜂鸣器，LEDC %d Hz 方波\n", BUZZER_FREQ);
#endif
}

/* ---------------- 电机底层（PWM + 斜坡 + 死区） ---------------- */
// 每个电机两个输入脚：一个出 PWM、另一个拉低 → 转速可调，方向由哪一相出 PWM 决定。
// 上层只设“目标速度”，motorTick() 每 10ms 把实际占空比往目标靠：
//   加速/减速走斜坡，过零点后强制静默 MOTOR_DEAD_MS 才允许反向 —— 硬切换的冲击就被抹平了。
struct Axis {
  int      cur;       // 当前实际占空比 -100~100
  int      target;    // 目标占空比 -100~100
  uint32_t tZero;     // 最近一次降到 0 的时刻

  // 单轴走一步：cur 朝 target 逼近（写成成员是因为 Arduino 会提前生成函数原型，
  // 独立函数带 Axis& 参数时原型会跑到 struct 定义之前而编译失败）
  void step(uint32_t now) {
#if MOTOR_RAMP_ENABLE
    if (cur == target) return;

    // 已经停在零点：等死区时间过去再起步/反向
    if (cur == 0) {
      if (now - tZero < MOTOR_DEAD_MS) return;
      int s = (abs(target) < RAMP_UP_STEP) ? abs(target) : RAMP_UP_STEP;
      cur = (target > 0) ? s : -s;
      return;
    }

    bool reversing = (cur > 0 && target < 0) || (cur < 0 && target > 0);
    int  s = (reversing || abs(target) <= abs(cur)) ? RAMP_DOWN_STEP : RAMP_UP_STEP;

    if (cur > 0) { cur -= s; if (cur < 0) cur = 0; }
    else         { cur += s; if (cur > 0) cur = 0; }

    if (cur == 0) tZero = now;      // 记录过零时刻，开始数死区
#else
    cur = target;
    if (cur == 0) tZero = now;
#endif
  }
};

static Axis axisL = {0, 0, 0};
static Axis axisR = {0, 0, 0};

static inline int motorDrive(int pinA, int pinB, int speedPct) {
  if (speedPct > 100)  speedPct = 100;
  if (speedPct < -100) speedPct = -100;

  uint32_t duty = (uint32_t)MOTOR_DUTY_MAX * (uint32_t)abs(speedPct) / 100;
  if (speedPct >= 0) {
    ledcWrite(pinA, duty);
    ledcWrite(pinB, 0);
  } else {
    ledcWrite(pinA, 0);
    ledcWrite(pinB, duty);
  }
  return speedPct;
}

// 把当前占空比刷到 LEDC。drive 任务每个 tick 调一次
static void motorTick() {
  uint32_t now = millis();
  axisL.step(now);
  axisR.step(now);
  motorDrive(motor_left_in1,  motor_left_in2,  axisL.cur * TRIM_LEFT  / 100);
  motorDrive(motor_right_in3, motor_right_in4, axisR.cur * TRIM_RIGHT / 100);
}

// 两个轴都真正停住了（目标为 0 且实际已归零）
static bool motorAtRest() {
  return axisL.target == 0 && axisR.target == 0 && axisL.cur == 0 && axisR.cur == 0;
}

// 停稳之后又静默了 SETTLE_MS（换向前要求的间隙）
static bool restElapsed(uint32_t &tRest) {
  uint32_t now = millis();
  if (!motorAtRest()) { tRest = 0; return false; }
  if (tRest == 0) tRest = now;
  return (now - tRest) >= SETTLE_MS;
}

// 立即断电（初始化/急停用），不走斜坡
static void motorStopNow() {
  uint32_t now = millis();
  axisL.cur = axisL.target = 0; axisL.tZero = now;
  axisR.cur = axisR.target = 0; axisR.tZero = now;
  motorDrive(motor_left_in1,  motor_left_in2,  0);
  motorDrive(motor_right_in3, motor_right_in4, 0);
}

static void motorLeft(int speedPct)  { axisL.target = constrain(speedPct, -100, 100); }
static void motorRight(int speedPct) { axisR.target = constrain(speedPct, -100, 100); }

void carForward()  { motorLeft( SPEED_FWD);  motorRight( SPEED_FWD); }   // 双电机正转
void carBackward() { motorLeft(-SPEED_BACK); motorRight(-SPEED_BACK); }
void carTurnLeft() { motorLeft(-SPEED_TURN); motorRight( SPEED_TURN); }  // 左轮反转 + 右轮正转
void carTurnRight(){ motorLeft( SPEED_TURN); motorRight(-SPEED_TURN); }  // 左轮正转 + 右轮反转
void carStop()     { motorLeft(0); motorRight(0); }                      // 平滑停车

/* ---------------- 线程 1：IR 采样 + 消抖 ---------------- */
// 单路时间窗消抖：状态变化必须持续超过阈值才被采纳。
// 检出(0->1)用短阈值 → 反应快；释放(1->0)用长阈值 → 挡住回弹/环境光抖动。
// 每路独立计时，互不拖累。
static uint8_t irDebounce(bool rawOn, uint8_t &stable, uint8_t &cand,
                          uint32_t &tCand, uint32_t now) {
  uint8_t raw = rawOn ? 1 : 0;

  if (raw != cand) {          // 出现新电平，重新计时
    cand  = raw;
    tCand = now;
    return stable;
  }

  uint32_t need = (raw && !stable) ? IR_DEBOUNCE_ON_MS : IR_DEBOUNCE_OFF_MS;
  if ((now - tCand) >= need) stable = raw;

  return stable;
}

void taskIrSensor(void *arg) {
  uint8_t  stableL = 0, candL = 0;
  uint8_t  stableR = 0, candR = 0;
  uint32_t tCandL  = millis(), tCandR = millis();
  uint8_t  lastMask = 0xFF;   // 保证首轮一定发布一次初始状态

  for (;;) {
    uint32_t now = millis();

    uint8_t l = irDebounce(digitalRead(ir_left)  == IR_ACTIVE_LEVEL,
                           stableL, candL, tCandL, now);
    uint8_t r = irDebounce(digitalRead(ir_right) == IR_ACTIVE_LEVEL,
                           stableR, candR, tCandR, now);

    uint8_t mask = (l ? IR_LEFT_BIT : 0) | (r ? IR_RIGHT_BIT : 0);
    if (mask != lastMask) {
      lastMask = mask;
      xQueueOverwrite(irQueue, &mask);   // 覆盖式写入，不会阻塞
      Serial.printf("[IR] mask=0x%02X L=%u R=%u\n", mask, l, r);
    }

    vTaskDelay(pdMS_TO_TICKS(IR_SAMPLE_MS));
  }
}

/* ---------------- 线程 2：行驶状态机 ---------------- */
void taskDrive(void *arg) {
  Phase    ph        = P_FORWARD;
  uint32_t tPhase    = millis();
  uint32_t backMs    = BACKUP_MS_ONE;
  uint32_t turnMs    = TURN_MS_ONE;
  bool     turnRight = true;
  uint8_t  blocked   = 0;        // 连续被堵次数
  uint32_t tClear    = 0;        // 连续畅通计时起点
  uint32_t tRest     = 0;        // 电机停稳时刻（0 = 还在动）

  motorStopNow();
  vTaskDelay(pdMS_TO_TICKS(200));

  for (;;) {
    uint8_t mask = 0;
    if (xQueuePeek(irQueue, &mask, 0) != pdPASS) mask = 0;

    /* ---------- 手机手动遥控 ---------- */
    if (g_mode == MODE_MANUAL) {
      static uint8_t lastGuard = 0;

      ph     = P_FORWARD;      // 退出自动流程，下次切回自动从“前进”重新开始
      tPhase = millis();
      tRest  = 0;

      int l = g_remoteL;
      int r = g_remoteR;
      bool stale = (millis() - g_lastCmd) > REMOTE_TIMEOUT_MS;

      if (stale) { l = 0; r = 0; }                              // 断线/松手 -> 停车
      if (mask != 0) { if (l > 0) l = 0; if (r > 0) r = 0; }     // 有障碍只准后退

      uint8_t guard = (uint8_t)(mask != 0) | (uint8_t)(stale ? 2 : 0);
      if (guard != lastGuard) {
        lastGuard = guard;
        if (guard & 1) Serial.println("[MAN] IR 检出障碍，禁止前进");
        if (guard & 2) Serial.println("[MAN] 指令超时，已停车");
      }

      motorLeft(l);
      motorRight(r);
      motorTick();

      // 手动模式下只要有一侧在倒转，也嘀嘀提示
      g_buzzer = ((l < 0) || (r < 0)) ? BUZZ_BACK : BUZZ_OFF;

      g_phase = (l || r) ? P_FORWARD : P_IDLE;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    uint32_t dt = millis() - tPhase;

    g_buzzer = BUZZ_OFF;        // 默认不响，只有倒车时才开
    switch (ph) {
      case P_FORWARD:
        carForward();
        if (mask == 0) {
          // 连续畅通 1.5s 就认为摆脱困境，转向时长恢复默认
          if (tClear == 0) tClear = millis();
          if (millis() - tClear > 1500) blocked = 0;
        } else {
          tClear = 0;
        }

        if (mask != 0) {
          bool both = (mask == IR_BOTH);
          blocked = (blocked < 9) ? blocked + 1 : 9;

          backMs = both ? BACKUP_MS_BOTH : BACKUP_MS_ONE;
          turnMs = both ? (TURN_MS_BOTH + (blocked - 1) * TURN_STEP_MS)
                        : (TURN_MS_ONE  + (blocked - 1) * TURN_STEP_MS);

          if (both) {
            // 两边都堵：随机挑一边转，避免反复卡死在角落
            turnRight = (random(0, 2) == 1);
          } else {
            turnRight = (mask & IR_LEFT_BIT) ? true : false;  // 左堵右转，右堵左转
          }
          Serial.printf("[DRV] blocked mask=0x%02X -> back %lums then turn %s %lums\n",
                        mask, (unsigned long)backMs, turnRight ? "RIGHT" : "LEFT",
                        (unsigned long)turnMs);

          carStop();
          ph = P_STOP1;
          tPhase = millis();
        }
        break;

      case P_STOP1:   // 前进 → 后退：等车真正停稳，再额外等 SETTLE_MS
        if (!restElapsed(tRest)) break;
        carBackward();
        ph = P_BACK;
        tPhase = millis();
        break;

      case P_BACK:
        g_buzzer = BUZZ_BACK;   // 倒车嘀嘀
        // 死区期间电机还没动，重新计时，保证实际后退时长足量
        if (axisL.cur == 0 && axisR.cur == 0) { tPhase = millis(); break; }
        if (dt >= backMs) { carStop(); ph = P_STOP2; tPhase = millis(); }
        break;

      case P_STOP2:   // 后退 → 转向
        if (!restElapsed(tRest)) break;
        if (turnRight) carTurnRight(); else carTurnLeft();
        ph = P_TURN;
        tPhase = millis();
        break;

      case P_TURN:
        if (axisL.cur == 0 && axisR.cur == 0) { tPhase = millis(); break; }
        if (dt >= turnMs) { carStop(); ph = P_STOP3; tPhase = millis(); }
        break;

      case P_STOP3:   // 转向 → 前进
        if (!restElapsed(tRest)) break;
        ph = P_FORWARD;
        tPhase = millis();
        break;
    }

    motorTick();          // 每 tick 刷新斜坡 + PWM
    g_phase = (uint8_t)ph;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* ---------------- 线程 3：状态指示 + 蜂鸣器节拍 ---------------- */
void taskIndicator(void *arg) {
  uint32_t lastColor = 0xFFFFFFFF;

  uint8_t  lastPat = 0xFF;
  bool     beepOn  = false;
  uint32_t tBeep   = 0;

  for (;;) {
    uint8_t ph   = g_phase;
    uint8_t mask = 0;
    if (xQueuePeek(irQueue, &mask, 0) != pdPASS) mask = 0;

    // 左右 IR 指示灯：显示消抖后的结果，和避障判断保持一致
    digitalWrite(led_green, (mask & IR_LEFT_BIT)  ? HIGH : LOW);
    digitalWrite(led_red,   (mask & IR_RIGHT_BIT) ? HIGH : LOW);

    uint32_t color;
    switch (ph) {
      case P_FORWARD: color = strip.Color(0, 40, 0);   break;  // 绿：前进
      case P_BACK:    color = strip.Color(60, 0, 0);   break;  // 红：后退
      case P_TURN:    color = strip.Color(0, 0, 60);   break;  // 蓝：转向
      default:        color = strip.Color(40, 20, 0);  break;  // 黄：停顿
    }

    if (color != lastColor) {
      strip.fill(color);
      strip.show();
      lastColor = color;
    }

    /* ---- 蜂鸣器节拍：本任务 50ms 一跳，用时间戳生成嘀嘀声 ---- */
    uint8_t pat = g_buzzer;
    if (pat != lastPat) {                 // 换模式，从“静音”重新开始
      lastPat = pat;
      beepOn  = false;
      tBeep   = millis();
      buzzerWrite(false);
    }

    if (pat != BUZZ_OFF) {
      uint32_t onMs  = (pat == BUZZ_ALARM) ? 60 : 100;
      uint32_t offMs = (pat == BUZZ_ALARM) ? 60 : 150;
      uint32_t now   = millis();
      if (now - tBeep >= (beepOn ? onMs : offMs)) {
        beepOn = !beepOn;
        buzzerWrite(beepOn);
        tBeep = now;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/* ---------------- 线程 4：通信层 ---------------- */

/* ---- 指令解析：WiFi 和 BLE 共用同一套文本协议 ---- */
// 指令格式："M,<左>,<右>" 手动差速 / "A" 切自动避障 / "S" 急停
static void handleCmd(char *s) {
  while (*s == ' ') s++;

  if (s[0] == 'M' || s[0] == 'm') {
    char *p = strchr(s, ',');
    if (!p) return;
    int l = atoi(p + 1);
    char *q = strchr(p + 1, ',');
    int r = q ? atoi(q + 1) : l;

    g_remoteL = constrain(l, -100, 100);
    g_remoteR = constrain(r, -100, 100);
    g_mode    = MODE_MANUAL;
    g_lastCmd = millis();
  } else if (s[0] == 'A' || s[0] == 'a') {
    g_remoteL = 0;
    g_remoteR = 0;
    g_mode    = MODE_AUTO;
    Serial.println("[CMD] -> 自动避障");
  } else if (s[0] == 'S' || s[0] == 's') {
    g_remoteL = 0;
    g_remoteR = 0;
    g_lastCmd = millis();
    Serial.println("[CMD] 急停");
  }
}

/* ==================== 方式 A：WiFi 热点 + 网页 ==================== */
#if COMM_WIFI
static httpd_handle_t s_httpd = nullptr;
static volatile int   s_wsFd  = -1;         // 当前 WebSocket 客户端 fd

static esp_err_t handleRoot(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handleWs(httpd_req_t *req) {
  if (req->method == HTTP_GET) {                 // 握手完成，记住客户端
    s_wsFd = httpd_req_to_sockfd(req);
    Serial.printf("[WS] client fd=%d\n", s_wsFd);
    return ESP_OK;
  }

  httpd_ws_frame_t f;
  memset(&f, 0, sizeof(f));
  f.type = HTTPD_WS_TYPE_TEXT;

  esp_err_t ret = httpd_ws_recv_frame(req, &f, 0);   // 先取长度
  if (ret != ESP_OK) return ret;

  size_t len = f.len;
  if (len == 0 || len > 64) return ESP_OK;

  char buf[72];
  f.payload = (uint8_t *)buf;
  ret = httpd_ws_recv_frame(req, &f, len);           // 再读内容
  if (ret != ESP_OK) return ret;
  buf[len] = '\0';

  handleCmd(buf);
  return ESP_OK;
}

static void startWebServer() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port      = 80;
  cfg.max_open_sockets = 4;
  cfg.lru_purge_enable = true;
  cfg.stack_size       = 8192;

  if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
    Serial.println("[WEB] httpd_start failed");
    return;
  }

  httpd_uri_t root;
  memset(&root, 0, sizeof(root));
  root.uri     = "/";
  root.method  = HTTP_GET;
  root.handler = handleRoot;
  httpd_register_uri_handler(s_httpd, &root);

  httpd_uri_t ws;
  memset(&ws, 0, sizeof(ws));
  ws.uri          = "/ws";
  ws.method       = HTTP_GET;
  ws.handler      = handleWs;
  ws.is_websocket = true;
  httpd_register_uri_handler(s_httpd, &ws);

  Serial.println("[WEB] http://192.168.4.1/");
}

void taskCommWifi(void *arg) {
  String ssid = String(AP_SSID_PREFIX) + String((uint32_t)(ESP.getEfuseMac() & 0xFFFF), HEX);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);                    // 关掉省电，降低指令延迟
  bool ok = WiFi.softAP(ssid.c_str(), AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CLIENTS);

  Serial.printf("[WIFI] AP \"%s\" %s, IP=%s\n", ssid.c_str(), ok ? "up" : "FAILED",
                WiFi.softAPIP().toString().c_str());

  if (ok) startWebServer();

  uint32_t tStat = 0;
  for (;;) {
    if (millis() - tStat >= STATUS_PUSH_MS) {
      tStat = millis();

      if (s_wsFd >= 0 && s_httpd != nullptr) {
        uint8_t mask = 0;
        xQueuePeek(irQueue, &mask, 0);

        char msg[40];
        snprintf(msg, sizeof(msg), "ST,%u,%u,%u",
                 (unsigned)(g_mode == MODE_AUTO),      // 1 = 自动，和网页里的 autoMode 同名同义
                 (unsigned)((mask & IR_LEFT_BIT)  ? 1 : 0),
                 (unsigned)((mask & IR_RIGHT_BIT) ? 1 : 0));

        httpd_ws_frame_t f;
        memset(&f, 0, sizeof(f));
        f.type    = HTTPD_WS_TYPE_TEXT;
        f.payload = (uint8_t *)msg;
        f.len     = strlen(msg);

        if (httpd_ws_send_frame_async(s_httpd, s_wsFd, &f) != ESP_OK) {
          Serial.println("[WS] client gone");
          s_wsFd = -1;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
#endif  // COMM_WIFI

/* ==================== 方式 B：BLE GATT（Web Bluetooth） ==================== */
// 手机 Chrome 走 Web Bluetooth 直连，不用连热点、不用记 IP。
// UUID 沿用 Nordic UART Service：nRF Connect 这类工具能直接认出来，方便绕开网页裸调。
//   RX(6e400002) 手机 write  -> 车
//   TX(6e400003) 车 notify   -> 手机
#if COMM_BLE

#define BLE_DEV_NAME  "MiniCar"
#define SVC_UUID      "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_UUID       "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define TX_UUID       "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

static BLEServer         *s_bleServer  = nullptr;
static BLECharacteristic *s_bleTx      = nullptr;
static volatile int       s_bleClients = 0;

class BleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *srv) override {
    s_bleClients = srv->getConnectedCount();
    Serial.printf("[BLE] 手机已连接 (%d)\n", s_bleClients);
  }

  // BLE 断开是静默的：不清状态的话 g_lastCmd 会一直保持旧值，车断开后还会接着跑。
  // 这里直接把目标速度清零并重启广播，不等 REMOTE_TIMEOUT_MS 那 600ms。
  void onDisconnect(BLEServer *srv) override {
    s_bleClients = srv->getConnectedCount();
    g_remoteL = 0;
    g_remoteR = 0;
    g_lastCmd = 0;
    Serial.println("[BLE] 手机断开，已停车，重新广播");
    srv->startAdvertising();
  }
};

class BleRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue();
    if (v.length() == 0 || v.length() > 63) return;
    char buf[64];
    v.toCharArray(buf, sizeof(buf));
    handleCmd(buf);              // 和 WiFi 版完全同一套解析
  }
};

static void startBle() {
  BLEDevice::init(BLE_DEV_NAME);

  s_bleServer = BLEDevice::createServer();
  s_bleServer->setCallbacks(new BleServerCallbacks());

  BLEService *svc = s_bleServer->createService(SVC_UUID);

  BLECharacteristic *rx = svc->createCharacteristic(
      RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new BleRxCallbacks());

  s_bleTx = svc->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  s_bleTx->addDescriptor(new BLE2902());   // CCCD：手机写了 0x0001 才会收到 notify

  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  // 刻意不广播 128 位服务 UUID：单个 UUID 就吃掉 18 字节，加 Flags 和设备名会超过
  // 31 字节的 adv 上限，广告直接起不来。页面靠名字前缀过滤，访问权限由 optionalServices 给。
  adv->setScanResponse(true);              // 设备名放 scan response，adv 包保持精简
  adv->setMinPreferred(0x06);              // 建议 7.5ms 连接间隔（Android 不一定采纳）
  adv->start();

  Serial.printf("[BLE] \"%s\" 广播中，等手机连\n", BLE_DEV_NAME);
}

void taskCommBle(void *arg) {
  startBle();

  uint32_t tStat = 0;
  char msg[40];

  for (;;) {
    if (s_bleClients > 0 && millis() - tStat >= STATUS_PUSH_MS) {
      tStat = millis();

      uint8_t mask = 0;
      xQueuePeek(irQueue, &mask, 0);

      snprintf(msg, sizeof(msg), "ST,%u,%u,%u",
               (unsigned)(g_mode == MODE_AUTO),
               (unsigned)((mask & IR_LEFT_BIT)  ? 1 : 0),
               (unsigned)((mask & IR_RIGHT_BIT) ? 1 : 0));

      s_bleTx->setValue((uint8_t *)msg, strlen(msg));
      s_bleTx->notify();      // 手机没订阅时 Bluedroid 内部会丢掉，不用自己判断
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

#endif  // COMM_BLE

/* ---------------- 线程 5：OLED 状态屏 ---------------- */
#if ENABLE_OLED
static const char *phaseName(uint8_t ph, bool manual) {
  if (manual) return "REMOTE";
  switch (ph) {
    case P_FORWARD: return "FORWARD";
    case P_STOP1:   return "BRAKE";
    case P_BACK:    return "BACK";
    case P_STOP2:   return "SWITCH";
    case P_TURN:    return "TURN";
    case P_STOP3:   return "ALIGN";
    case P_IDLE:    return "IDLE";
  }
  return "?";
}

void taskDisplay(void *arg) {
  // ---- 开机自检画面：能看到这行字，说明 SPI 接线和引脚全对 ----
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("MiniCar");
  display.setTextSize(1);
  display.setCursor(0, 22);
  display.println("OLED self-test");
  display.setTextSize(3);
  display.setCursor(0, 36);
  display.println("OK");
  display.display();
  Serial.println("[OLED] self-test frame sent");
  vTaskDelay(pdMS_TO_TICKS(1500));

  for (;;) {
    uint8_t mask = 0;
    xQueuePeek(irQueue, &mask, 0);

    bool    manual = (g_mode == MODE_MANUAL);
    uint8_t ph     = g_phase;
    char    line[28];

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    // 标题栏：左边车名，右边模式
    display.setCursor(0, 0);
    display.print("MiniCar");
    display.setCursor(manual ? 96 : 104, 0);
    display.print(manual ? "MAN" : "AUTO");
    display.drawFastHLine(0, 10, 128, SSD1306_WHITE);

    // 主状态：大字相位
    display.setTextSize(2);
    display.setCursor(0, 14);
    display.print(phaseName(ph, manual));

    // IR 状态 + 左右目标速度
    display.setTextSize(1);
    snprintf(line, sizeof(line), "IR L%c R%c   L%3d R%3d",
             (mask & IR_LEFT_BIT)  ? '!' : '-',
             (mask & IR_RIGHT_BIT) ? '!' : '-',
             axisL.target, axisR.target);
    display.setCursor(0, 36);
    display.print(line);

    // 连接状态
    display.setCursor(0, 46);
#if COMM_BLE
    display.print(s_bleClients > 0 ? "BLE connected" : "BLE waiting..");
#else
    display.print("AP 192.168.4.1");
    snprintf(line, sizeof(line), "%uc", (unsigned)WiFi.softAPgetStationNum());
    display.setCursor(112, 46);
    display.print(line);
#endif

    // 剩余内存
    snprintf(line, sizeof(line), "heap %uK", (unsigned)(ESP.getFreeHeap() / 1024));
    display.setCursor(0, 56);
    display.print(line);

    display.display();

    vTaskDelay(pdMS_TO_TICKS(200));   // 5Hz 刷新，够看又不浪费 CPU
  }
}
#endif  // ENABLE_OLED

/* ---------------- setup / loop ---------------- */
void setup() {
  Serial.begin(115200);
  Serial.println("\nMiniCar obstacle-avoidance start");

  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.fill(strip.Color(0, 0, 0));
  strip.show();

  pinMode(led_red,   OUTPUT);
  pinMode(led_green, OUTPUT);

  pinMode(ir_left,  INPUT);
  pinMode(ir_right, INPUT);

  // 4 个电机脚全部挂到 LEDC，各自独立通道，默认占空比 0（相当于原来的 digitalWrite 0）
  if (!ledcAttach(motor_left_in1,  MOTOR_PWM_FREQ, MOTOR_PWM_RES) ||
      !ledcAttach(motor_left_in2,  MOTOR_PWM_FREQ, MOTOR_PWM_RES) ||
      !ledcAttach(motor_right_in3, MOTOR_PWM_FREQ, MOTOR_PWM_RES) ||
      !ledcAttach(motor_right_in4, MOTOR_PWM_FREQ, MOTOR_PWM_RES)) {
    Serial.println("[ERR] LEDC attach failed");
  }

  // 上电即停：目标速度全部归零，且默认手动模式（没有手机指令就一直不动）
  motorStopNow();
  Serial.println("[MODE] 上电默认手动/停止，页面上点「切到自动」才自动避障");

  // 蜂鸣器自检：开机嘀嘀两声，听得到就说明 GPIO6 和蜂鸣器都通了
  buzzerInit();
  buzzerWrite(true);  delay(80);
  buzzerWrite(false); delay(80);
  buzzerWrite(true);  delay(80);
  buzzerWrite(false);

#if ENABLE_OLED
  // OLED 初始化（软件 SPI，任意 GPIO）
  // 注意：SPI 屏读不回状态，接线错了 begin() 也可能成功，不显示只能查线
  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println("[ERR] SSD1306 初始化失败（内存分配/SPI 句柄）");
  } else {
    Serial.printf("[OLED] SSD1306 128x64  mode=%d CLK=%d MOSI=%d DC=%d RST=%d CS=%d\n",
                  OLED_MODE, OLED_CLK, OLED_MOSI, OLED_DC, OLED_RST, OLED_CS);
  }
  display.clearDisplay();
  display.display();
#else
  Serial.println("[OLED] 已关闭 (ENABLE_OLED 0)，不占用 IO17/18/19 与 IO5");
#endif

  irQueue = xQueueCreate(1, sizeof(uint8_t));
  configASSERT(irQueue != nullptr);
  uint8_t init = 0;
  xQueueOverwrite(irQueue, &init);

  xTaskCreatePinnedToCore(taskIrSensor,  "ir",   3072, nullptr, 4, nullptr, 1);
  xTaskCreatePinnedToCore(taskDrive,     "drive",4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(taskIndicator, "led",  3072, nullptr, 1, nullptr, 0);
#if ENABLE_OLED
  xTaskCreatePinnedToCore(taskDisplay,   "disp", 4096, nullptr, 1, nullptr, 0);
#endif

  // 通信放在最后起：先把电机/传感器初始化好，避免射频启动期间 GPIO 还悬空
#if COMM_BLE
  xTaskCreatePinnedToCore(taskCommBle,  "ble",  6144, nullptr, 3, nullptr, 0);
#endif
#if COMM_WIFI
  xTaskCreatePinnedToCore(taskCommWifi, "wifi", 8192, nullptr, 3, nullptr, 0);
#endif
}

void loop() {
  // 所有逻辑都在各自的线程里，主循环只做低频看门狗输出
  static uint32_t last = 0;
  if (millis() - last > 2000) {
    last = millis();
    Serial.printf("[SYS] heap=%lu phase=%u\n",
                  (unsigned long)ESP.getFreeHeap(), (unsigned)g_phase);
  }
  vTaskDelay(pdMS_TO_TICKS(1000));
}

/* ---------------- WS2812 演示函数（暂未调用，留用） ---------------- */
void colorWipe(uint32_t color, int wait) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
    strip.show();
    delay(wait);
  }
}

void whiteOverRainbow(int whiteSpeed, int whiteLength) {
  if (whiteLength >= strip.numPixels()) whiteLength = strip.numPixels() - 1;
  int      head          = whiteLength - 1;
  int      tail          = 0;
  int      loops         = 3;
  int      loopNum       = 0;
  uint32_t lastTime      = millis();
  uint32_t firstPixelHue = 0;

  for (;;) {
    for (int i = 0; i < strip.numPixels(); i++) {
      if (((i >= tail) && (i <= head)) ||
          ((tail > head) && ((i >= tail) || (i <= head)))) {
        strip.setPixelColor(i, strip.Color(0, 0, 0, 255));
      } else {
        int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
        strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
      }
    }
    strip.show();
    firstPixelHue += 40;
    if ((millis() - lastTime) > whiteSpeed) {
      if (++head >= strip.numPixels()) {
        head = 0;
        if (++loopNum >= loops) return;
      }
      if (++tail >= strip.numPixels()) tail = 0;
      lastTime = millis();
    }
  }
}

void pulseWhite(uint8_t wait) {
  for (int j = 0; j < 256; j++) {
    strip.fill(strip.Color(0, 0, 0, strip.gamma8(j)));
    strip.show();
    delay(wait);
  }
  for (int j = 255; j >= 0; j--) {
    strip.fill(strip.Color(0, 0, 0, strip.gamma8(j)));
    strip.show();
    delay(wait);
  }
}

void rainbowFade2White(int wait, int rainbowLoops, int whiteLoops) {
  int fadeVal = 0, fadeMax = 100;
  for (uint32_t firstPixelHue = 0; firstPixelHue < rainbowLoops * 65536;
       firstPixelHue += 256) {
    for (int i = 0; i < strip.numPixels(); i++) {
      uint32_t pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue, 255,
        255 * fadeVal / fadeMax)));
    }
    strip.show();
    delay(wait);
    if (firstPixelHue < 65536) {
      if (fadeVal < fadeMax) fadeVal++;
    } else if (firstPixelHue >= ((rainbowLoops - 1) * 65536)) {
      if (fadeVal > 0) fadeVal--;
    } else {
      fadeVal = fadeMax;
    }
  }
  for (int k = 0; k < whiteLoops; k++) {
    for (int j = 0; j < 256; j++) {
      strip.fill(strip.Color(0, 0, 0, strip.gamma8(j)));
      strip.show();
    }
    delay(1000);
    for (int j = 255; j >= 0; j--) {
      strip.fill(strip.Color(0, 0, 0, strip.gamma8(j)));
      strip.show();
    }
  }
  delay(500);
}
