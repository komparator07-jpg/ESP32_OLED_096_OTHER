#include <Wire.h>
#include <APDS9930.h>

// ======================= НАСТРОЙКИ МОДУЛЯ =======================
#define SDA_PIN 21
#define SCL_PIN 22
#define SENSOR_PERIOD_MS 25      // период опроса датчика (~40 Гц)

// Пороги (в единицах отфильтрованного RAW). Подберите при диагностике.
int threshold1 = 150;            // Порог 1 (нижний)
int threshold2 = 400;            // Порог 2 (верхний)
const int TH_HYST = 8;           // гистерезис, чтобы событие не дребезжало на границе

// ---- Диагностика (1 = включено, 0 = выключено) ----
#define DEBUG_SERIAL   1         // слать RAW в Serial Monitor / Plotter
#define DEBUG_OLED_RAW 1         // показывать RAW и шкалу на OLED
// ================================================================

// ======================= ПЕРЕМЕННЫЕ ДЛЯ ВАШЕГО ПРОЕКТА =======================
int  proxValue = 0;              // отфильтрованный RAW (0..1023)
bool over1 = false;              // true, пока значение выше Порога 1
bool over2 = false;              // true, пока значение выше Порога 2
// События: true ровно один проход loop() в момент пересечения порога
bool th1Rise = false, th1Fall = false;
bool th2Rise = false, th2Fall = false;
bool proxOk  = false;            // датчик найден и работает
// =============================================================================

APDS9930 apds;

// ---- медианный фильтр на 5 отсчётов ----
const int MED_N = 5;
uint16_t medBuf[MED_N];
int medIdx = 0;
bool medFilled = false;

uint16_t median5(uint16_t v) {
  medBuf[medIdx++] = v;
  if (medIdx >= MED_N) { medIdx = 0; medFilled = true; }
  int n = medFilled ? MED_N : medIdx;
  uint16_t t[MED_N];
  for (int i = 0; i < n; i++) t[i] = medBuf[i];
  for (int i = 1; i < n; i++) {
    uint16_t k = t[i]; int j = i - 1;
    while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
    t[j + 1] = k;
  }
  return t[n / 2];
}

float proxEma = 0;
unsigned long tSensor = 0;

// ===================== DEBUG: OLED (НАЧАЛО БЛОКА) =====================
#if DEBUG_OLED_RAW
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(128, 64, &Wire, -1);
const float OLED_RAW_MAX = 1023.0;   // правый край шкалы
float oledBar = 0;
unsigned long tDraw = 0;

void debugOledInit() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) return;
  display.clearDisplay();
  display.display();
  display.setTextColor(SSD1306_WHITE);
}

void debugOledUpdate() {
  oledBar += 0.25 * (proxValue - oledBar);
  unsigned long now = millis();
  if (now - tDraw < 33) return;
  tDraw = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("RAW");
  display.setTextSize(3);
  display.setCursor(0, 14);
  display.print(proxValue);

  const int bx = 2, by = 46, bw = 124, bh = 10;
  display.drawRect(bx - 2, by - 2, bw + 4, bh + 4, SSD1306_WHITE);
  int fill = constrain((int)(bw * (oledBar / OLED_RAW_MAX)), 0, bw);
  display.fillRect(bx, by, fill, bh, SSD1306_WHITE);

  // метки порогов под шкалой
  int x1 = constrain((int)(bw * (threshold1 / OLED_RAW_MAX)), 0, bw);
  int x2 = constrain((int)(bw * (threshold2 / OLED_RAW_MAX)), 0, bw);
  display.drawFastVLine(bx + x1, by + bh + 3, 4, SSD1306_WHITE);
  display.drawFastVLine(bx + x2, by + bh + 3, 4, SSD1306_WHITE);
  display.display();
}
#endif
// ===================== DEBUG: OLED (КОНЕЦ БЛОКА) ======================

// ==================== DEBUG: SERIAL (НАЧАЛО БЛОКА) ====================
#if DEBUG_SERIAL
unsigned long tSerial = 0;

void debugSerialInit() {
  Serial.begin(115200);
}

// Формат подходит для Serial Plotter (Инструменты -> Плоттер по последовательному порту)
void debugSerialUpdate(uint16_t rawNow) {
  unsigned long now = millis();
  if (now - tSerial < 50) return;
  tSerial = now;
  Serial.print("raw:");      Serial.print(rawNow);
  Serial.print(",filtered:"); Serial.print(proxValue);
  Serial.print(",th1:");      Serial.print(threshold1);
  Serial.print(",th2:");      Serial.println(threshold2);
}
#endif
// ==================== DEBUG: SERIAL (КОНЕЦ БЛОКА) =====================

// ======================= ИНИЦИАЛИЗАЦИЯ ДАТЧИКА =======================
// Если Wire.begin() уже вызывается в вашем проекте, удалите его отсюда.
void proxInit() {
#if DEBUG_SERIAL
  debugSerialInit();
#endif
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

#if DEBUG_OLED_RAW
  debugOledInit();
#endif

  proxOk = apds.init();
  if (proxOk) apds.enableProximitySensor(false);
#if DEBUG_SERIAL
  else Serial.println("APDS-9930 not found!");
#endif
}

// ======================= ОБНОВЛЕНИЕ (вызывать в loop) ======================
void proxUpdate() {
  // события живут один проход loop()
  th1Rise = th1Fall = th2Rise = th2Fall = false;

  if (!proxOk) return;

  unsigned long now = millis();
  if (now - tSensor >= SENSOR_PERIOD_MS) {
    tSensor = now;
    uint16_t p;
    if (apds.readProximity(p)) {
      float med = median5(p);

      // адаптивный EMA: на месте - сильно сглаживает, при быстром движении догоняет
      float rel = fabs(med - proxEma) / (proxEma + 15.0);
      float alpha = constrain(0.05 + rel * 1.5, 0.05, 0.6);
      proxEma += alpha * (med - proxEma);
      proxValue = (int)(proxEma + 0.5);

      // пороги с гистерезисом и событиями
      if (!over1 && proxValue >= threshold1)             { over1 = true;  th1Rise = true; }
      else if (over1 && proxValue < threshold1 - TH_HYST) { over1 = false; th1Fall = true; }

      if (!over2 && proxValue >= threshold2)             { over2 = true;  th2Rise = true; }
      else if (over2 && proxValue < threshold2 - TH_HYST) { over2 = false; th2Fall = true; }

#if DEBUG_SERIAL
      debugSerialUpdate(p);
#endif
    }
  }

#if DEBUG_OLED_RAW
  debugOledUpdate();
#endif
}

// ============================== ПРИМЕР ==============================
void setup() {
  proxInit();
}

void loop() {
  proxUpdate();               // вызывать как можно чаще, без delay() в loop

  if (th1Rise) {
    // сработал Порог 1 (значение поднялось выше)
  }
  if (th2Rise) {
    // сработал Порог 2
  }
  if (th1Fall) {
    // значение упало ниже Порога 1
  }

  if (over2) {
    // пока значение выше Порога 2
  }
}
