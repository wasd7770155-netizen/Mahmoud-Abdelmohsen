#include <LiquidCrystal.h>

// ============================================================================
//  Boost Converter Controller (Arduino UNO / ATmega328P)
//  - PWM: Timer1 register-level, ~31.25kHz on D9 (OC1A)
//  - Feedback: A0 via divider R2=100k (top), R3=4.7k (bottom)
//  - LCD: 16x2 (RS=7, E=6, D4=5, D5=4, D6=3, D7=2)
// ============================================================================

// ---------------------- Pins ----------------------
static const uint8_t PWM_PIN = 9;   // OC1A
static const uint8_t FB_PIN  = A0;

static const uint8_t LCD_RS = 7;
static const uint8_t LCD_E  = 6;
static const uint8_t LCD_D4 = 5;
static const uint8_t LCD_D5 = 4;
static const uint8_t LCD_D6 = 3;
static const uint8_t LCD_D7 = 2;
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// ---------------------- Sensing / target ----------------------
static const float R_TOP = 100000.0f;
static const float R_BOT = 4700.0f;
static const float ADC_REF_V = 5.0f;
static const float ADC_MAX = 1023.0f;
static const float DIV_GAIN = (R_TOP + R_BOT) / R_BOT;

static const float VIN_NOMINAL = 9.0f;
static const float VOUT_TARGET = 100.0f;

// ---------------------- Timer1 PWM ----------------------
// Fast PWM mode 14, TOP=ICR1=511, prescaler=1 => 16MHz / 512 = 31.25kHz
static const uint16_t PWM_TOP = 511;
static const float DUTY_MIN = 0.04f;
static const float DUTY_MAX = 0.95f; // Critical for 9V -> 100V

// ---------------------- Control loop ----------------------
// Keep control fast for quicker response.
static const uint32_t CONTROL_PERIOD_US = 333; // ~3kHz for faster response

// Startup assist + regulation for reliability.
enum ControlMode : uint8_t { STARTUP, REGULATE };
ControlMode mode = STARTUP;

static const float STARTUP_DUTY_BEGIN = 0.88f;
static const float STARTUP_DUTY_END   = 0.95f;
static const float STARTUP_RAMP_PER_S = 3.60f;
static const float STARTUP_EXIT_V = 72.0f;
static const float STARTUP_REENTER_V = 58.0f;

// Require persistence before mode change (noise immunity).
static const uint8_t MODE_CONFIRM_COUNT = 6;
uint8_t readyToRegCount = 0;
uint8_t backToStartCount = 0;

// PI-D (with derivative filtering) + feed-forward.
static float Kp = 0.058f;
static float Ki = 2.800f;
static float Kd = 0.00040f;

float iTerm = 0.0f;
float prevErr = 0.0f;
float dErrFilt = 0.0f;
float dutyCmd = STARTUP_DUTY_BEGIN;
float voutFilt = 0.0f;

uint32_t lastControlUs = 0;

// ---------------------- LCD timing ----------------------
// User request: duty-cycle LCD read/update every 100ms.
static const uint32_t LCD_VOLT_MS = 120;
static const uint32_t LCD_DUTY_MS = 100;
uint32_t lastLcdVoltMs = 0;
uint32_t lastLcdDutyMs = 0;

// Heavy averaging for stable display.
static const uint8_t AVG_N = 20;
float vHist[AVG_N] = {0.0f};
float dHist[AVG_N] = {0.0f};
uint8_t histIdx = 0;
uint8_t histCount = 0;

float vDisplay = 0.0f;
float dDisplay = STARTUP_DUTY_BEGIN * 100.0f;

// ---------------------- Utils ----------------------
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline void setDuty(float duty) {
  duty = clampf(duty, DUTY_MIN, DUTY_MAX);
  dutyCmd = duty;
  OCR1A = (uint16_t)(duty * PWM_TOP + 0.5f);
}

float readVoutInstant() {
  // 2-sample average: less noise but still fast for a ~3kHz loop.
  uint16_t a = analogRead(FB_PIN);
  uint16_t b = analogRead(FB_PIN);
  float raw = 0.5f * (a + b);
  float vadc = (raw * ADC_REF_V) / ADC_MAX;
  float vout = vadc * DIV_GAIN;

  // Hard sanity clamp for simulator spikes.
  return clampf(vout, 0.0f, 130.0f);
}

void setupTimer1_31kHz() {
  pinMode(PWM_PIN, OUTPUT);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // Mode 14: WGM13:WGM10 = 1110
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // Non-inverting output on OC1A (D9)
  TCCR1A |= (1 << COM1A1);

  // Prescaler = 1
  TCCR1B |= (1 << CS10);

  ICR1 = PWM_TOP;
  OCR1A = (uint16_t)(STARTUP_DUTY_BEGIN * PWM_TOP);
}

void setup() {
  analogReference(DEFAULT);
  pinMode(FB_PIN, INPUT);

  setupTimer1_31kHz();

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Boost PID Init");

  float v0 = readVoutInstant();
  voutFilt = v0;
  vDisplay = v0;

  for (uint8_t i = 0; i < AVG_N; i++) {
    vHist[i] = v0;
    dHist[i] = STARTUP_DUTY_BEGIN * 100.0f;
  }
  histCount = AVG_N;

  lastControlUs = micros();
  lastLcdVoltMs = millis();
  lastLcdDutyMs = millis();
}

void controlStep(float dt) {
  float vout = readVoutInstant();

  // Faster filter for quicker transient response without excessive ripple sensitivity.
  voutFilt += 0.28f * (vout - voutFilt);

  if (mode == STARTUP) {
    dutyCmd += STARTUP_RAMP_PER_S * dt;
    if (dutyCmd > STARTUP_DUTY_END) dutyCmd = STARTUP_DUTY_END;
    setDuty(dutyCmd);

    if (voutFilt >= STARTUP_EXIT_V) {
      if (++readyToRegCount >= MODE_CONFIRM_COUNT) {
        mode = REGULATE;
        float err = VOUT_TARGET - voutFilt;
        iTerm = dutyCmd - Kp * err;
        iTerm = clampf(iTerm, -0.25f, 0.95f);
        prevErr = err;
        dErrFilt = 0.0f;
        readyToRegCount = 0;
      }
    } else {
      readyToRegCount = 0;
    }
  } else {
    float err = VOUT_TARGET - voutFilt;
    float dErr = (err - prevErr) / dt;
    prevErr = err;

    // Derivative low-pass for reliability/noise immunity.
    dErrFilt += 0.15f * (dErr - dErrFilt);

    // Feed-forward from ideal boost relation D = 1 - Vin/Vout
    float dutyFF = 1.0f - (VIN_NOMINAL / VOUT_TARGET); // ~0.91

    float pTerm = Kp * err;
    float dTerm = Kd * dErrFilt;

    float iCandidate = iTerm + Ki * err * dt;
    float uUnsat = dutyFF + pTerm + iCandidate + dTerm;

    bool satHigh = (uUnsat > DUTY_MAX);
    bool satLow  = (uUnsat < DUTY_MIN);
    bool pushFurtherSat = (satHigh && err > 0.0f) || (satLow && err < 0.0f);
    if (!pushFurtherSat) {
      iTerm = iCandidate;
    }

    float u = clampf(dutyFF + pTerm + iTerm + dTerm, DUTY_MIN, DUTY_MAX);

    // Reliability guard: if overshoot is significant, force lower duty briefly.
    if (voutFilt > 108.0f) {
      u = DUTY_MIN;
      iTerm = clampf(iTerm, -0.10f, 0.60f);
    }

    // Aggressive assist when still far from target.
    if (err > 25.0f && u < 0.93f) u = 0.93f;

    setDuty(u);

    if (voutFilt < STARTUP_REENTER_V) {
      if (++backToStartCount >= MODE_CONFIRM_COUNT) {
        mode = STARTUP;
        dutyCmd = clampf(u, STARTUP_DUTY_BEGIN, STARTUP_DUTY_END);
        backToStartCount = 0;
      }
    } else {
      backToStartCount = 0;
    }
  }

  vHist[histIdx] = vout;
  dHist[histIdx] = dutyCmd * 100.0f;
  histIdx = (histIdx + 1) % AVG_N;
  if (histCount < AVG_N) histCount++;
}

void computeDisplayAverages(float &vAvg, float &dAvg) {
  float vSum = 0.0f;
  float dSum = 0.0f;
  for (uint8_t i = 0; i < histCount; i++) {
    vSum += vHist[i];
    dSum += dHist[i];
  }
  vAvg = vSum / histCount;
  dAvg = dSum / histCount;
}

void updateLcdVoltageLine() {
  float vAvg, dAvg;
  computeDisplayAverages(vAvg, dAvg);

  // Keep Vout very stable visually.
  vDisplay += 0.25f * (vAvg - vDisplay);

  lcd.setCursor(0, 0);
  lcd.print("Vout:");
  lcd.print(vDisplay, 1);
  lcd.print("V   ");
}

void updateLcdDutyLine() {
  float vAvg, dAvg;
  computeDisplayAverages(vAvg, dAvg);

  // Duty line updated every 100ms per request.
  dDisplay = dAvg;

  lcd.setCursor(0, 1);
  lcd.print("D:");
  lcd.print(dDisplay, 1);
  lcd.print("% ");
  lcd.print(mode == STARTUP ? "ST" : "RG");
  lcd.print(" ");
}

void loop() {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastControlUs) >= CONTROL_PERIOD_US) {
    float dt = (nowUs - lastControlUs) * 1e-6f;
    lastControlUs = nowUs;

    if (dt < 0.0003f) dt = 0.0003f;
    if (dt > 0.0100f) dt = 0.0100f;

    controlStep(dt);
  }

  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastLcdVoltMs) >= LCD_VOLT_MS) {
    lastLcdVoltMs = nowMs;
    updateLcdVoltageLine();
  }

  if ((uint32_t)(nowMs - lastLcdDutyMs) >= LCD_DUTY_MS) {
    lastLcdDutyMs = nowMs;
    updateLcdDutyLine();
  }
}
