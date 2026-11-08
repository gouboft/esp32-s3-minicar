/*
 * MiniCar - ESP32-S3 左右红外自动避障（FreeRTOS 多线程版）
 *
 * 线程划分：
 *   taskIrSensor  (core 1) : 10ms 采样 + 30ms 消抖，只负责“看”，把结果丢进队列
 *   taskDrive     (core 1) : 10ms tick 的非阻塞状态机，只负责“动”
 *   taskIndicator (core 0) : 状态灯 / WS2812 显示，只负责“说”
 *
 * 避障策略（左/右 IR）：
 *   左边有障碍 -> 后退一小段 -> 右转 -> 前进
 *   右边有障碍 -> 后退一小段 -> 左转 -> 前进
 *   两边都有   -> 后退长一点 -> 随机大角度转 -> 前进（连续被堵会越转越多）
 */

#include <Adafruit_NeoPixel.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

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

/* ---------------- 可调参数 ---------------- */
// IR 检测到障碍时输出的电平。若实测方向相反，把 HIGH 改成 LOW
#define IR_ACTIVE_LEVEL LOW

#define IR_SAMPLE_MS      5     // 采样周期
#define IR_DEBOUNCE_ON_MS  25   // 检出障碍：需持续 25ms 才认（快，防撞）
#define IR_DEBOUNCE_OFF_MS 80   // 障碍消失：需持续 80ms 才认（慢，挡住回弹/反光抖动）

#define SETTLE_MS        100    // 换向前的停顿，保护减速箱
#define BACKUP_MS_ONE    250    // 单侧障碍后退时长
#define BACKUP_MS_BOTH   400    // 双侧障碍后退时长
#define TURN_MS_ONE      450    // 单侧障碍转向时长
#define TURN_MS_BOTH     700    // 双侧障碍转向时长
#define TURN_STEP_MS     250    // 每连续被堵一次，转向时间加长

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
  P_STOP3       // 转完回正
};

static QueueHandle_t irQueue = nullptr;   // 长度 1，永远只存最新采样结果
static volatile uint8_t g_phase = P_FORWARD;  // 给指示线程看

/* ---------------- 电机底层 ---------------- */
inline void motorWrite(int l1, int l2, int r1, int r2) {
  digitalWrite(motor_left_in1,  l1);
  digitalWrite(motor_left_in2,  l2);
  digitalWrite(motor_right_in3, r1);
  digitalWrite(motor_right_in4, r2);
}

void carForward()  { motorWrite(1, 0, 1, 0); }   // 双电机正转
void carBackward() { motorWrite(0, 1, 0, 1); }
void carTurnLeft() { motorWrite(0, 1, 1, 0); }   // 左轮反转 + 右轮正转
void carTurnRight(){ motorWrite(1, 0, 0, 1); }   // 左轮正转 + 右轮反转
void carStop()     { motorWrite(0, 0, 0, 0); }

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

  carStop();
  vTaskDelay(pdMS_TO_TICKS(200));

  for (;;) {
    uint8_t mask = 0;
    if (xQueuePeek(irQueue, &mask, 0) != pdPASS) mask = 0;

    uint32_t dt = millis() - tPhase;

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

      case P_STOP1:
        if (dt >= SETTLE_MS) { carBackward(); ph = P_BACK; tPhase = millis(); }
        break;

      case P_BACK:
        if (dt >= backMs) { carStop(); ph = P_STOP2; tPhase = millis(); }
        break;

      case P_STOP2:
        if (dt >= SETTLE_MS) {
          if (turnRight) carTurnRight(); else carTurnLeft();
          ph = P_TURN;
          tPhase = millis();
        }
        break;

      case P_TURN:
        if (dt >= turnMs) { carStop(); ph = P_STOP3; tPhase = millis(); }
        break;

      case P_STOP3:
        if (dt >= SETTLE_MS) {
          ph = P_FORWARD;
          tPhase = millis();
        }
        break;
    }

    g_phase = (uint8_t)ph;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* ---------------- 线程 3：状态指示 ---------------- */
void taskIndicator(void *arg) {
  uint32_t lastColor = 0xFFFFFFFF;

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

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

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

  pinMode(motor_left_in1,  OUTPUT);
  pinMode(motor_left_in2,  OUTPUT);
  pinMode(motor_right_in3, OUTPUT);
  pinMode(motor_right_in4, OUTPUT);

  carStop();

  irQueue = xQueueCreate(1, sizeof(uint8_t));
  configASSERT(irQueue != nullptr);
  uint8_t init = 0;
  xQueueOverwrite(irQueue, &init);

  xTaskCreatePinnedToCore(taskIrSensor,  "ir",   3072, nullptr, 4, nullptr, 1);
  xTaskCreatePinnedToCore(taskDrive,     "drive",4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(taskIndicator, "led",  3072, nullptr, 1, nullptr, 0);
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
