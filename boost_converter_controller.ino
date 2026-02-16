#include <LiquidCrystal.h>

// ============================================================================
// Precision Boost Converter Controller (Arduino UNO / ATmega328P)
// - PWM: Timer1 register PWM on D9 (OC1A) at ~31.25kHz
// - Feedback: A0 divider, R2=100k (top), R3=4.7k (bottom)
// - LCD: RS=7, E=6, D4=5, D5=4, D6=3, D7=2
// ============================================================================

// ---------------------- Pins ----------------------
static const uint8_t PWM_PIN = 9;
static const uint8_t FB_PIN  = A0;
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

// ---------------------- Sensing + calibration ----------------------
static const float R_TOP = 100000.0f;
static const float R_BOT = 4700.0f;
static const float DIV_GAIN = (R_TOP + R_BOT) / R_BOT;

// IMPORTANT FOR ACCURACY:
// Measure Arduino 5V rail with a multimeter and set ADC_VREF_CAL accordingly.
// Example: if measured 4.96V, set ADC_VREF_CAL to 4.96f.
static const float ADC_VREF_CAL = 5.00f;

// Optional final linear calibration from known points (keep defaults if unknown).
static const float VOUT_GAIN_CAL = 1.000f;
static const float VOUT_OFFSET_CAL = 0.00f;

static const float ADC_MAX = 1023.0f;
static const float VOUT_TARGET = 100.0f;
static const float VIN_NOMINAL = 9.0f;

// ---------------------- Timer1 PWM ----------------------
// Fast PWM mode 14 (TOP=ICR1), prescaler=1 => 16MHz / (1+511) = 31.25kHz
static const uint16_t PWM_TOP = 511;
static const float DUTY_MIN = 0.04f;
static const float DUTY_MAX = 0.95f;

// ---------------------- Control loop ----------------------
static const uint32_t CONTROL_PERIOD_US = 333; // ~3kHz

enum ControlMode : uint8_t { STARTUP, REGULATE };
ControlMode mode = STARTUP;

// Startup assist (for low-Vin startup reliability)
static const float STARTUP_DUTY_BEGIN = 0.84f;
static const float STARTUP_DUTY_END   = 0.95f;
static const float STARTUP_RAMP_PER_S = 2.80f;
static const float STARTUP_EXIT_V = 74.0f;
static const float STARTUP_REENTER_V = 61.0f;
static const uint8_t MODE_CONFIRM_COUNT = 6;
uint8_t readyToRegCount = 0;
uint8_t backToStartCount = 0;

// PI-D with back-calculation anti-windup (more accurate near limit/saturation)
static float Kp = 0.030f;
static float Ki = 0.45f;
static float Kd = 0.00022f;
static float Kaw = 1.50f; // anti-windup feedback gain

float iTerm = 0.0f;
float prevErr = 0.0f;
float dErrFilt = 0.0f;
float dutyCmd = STARTUP_DUTY_BEGIN;
float voutFilt = 0.0f;

uint32_t lastControlUs = 0;

// ---------------------- LCD timing ----------------------
static const uint32_t LCD_VOLT_MS = 120;
static const uint32_t LCD_DUTY_MS = 100;
uint32_t lastLcdVoltMs = 0;
uint32_t lastLcdDutyMs = 0;

// Heavy display averaging (stable readout)
static const uint8_t AVG_N = 20;
float vHist[AVG_N] = {0.0f};
float dHist[AVG_N] = {0.0f};
uint8_t histIdx = 0;
uint8_t histCount = 0;
float vDisplay = 0.0f;
float dDisplay = STARTUP_DUTY_BEGIN * 100.0f;

// ---------------------- Utilities ----------------------
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float dutyFromOcr() {
  return ((float)OCR1A) / PWM_TOP;
}

static inline void setDuty(float duty) {
  duty = clampf(duty, DUTY_MIN, DUTY_MAX);
  OCR1A = (uint16_t)(duty * PWM_TOP + 0.5f);
  dutyCmd = dutyFromOcr(); // actual applied duty
}

float readVoutInstant() {
  // Precision trimmed-mean ADC sampling:
  // 8 samples, discard min+max, average remaining 6.
  uint16_t mn = 1023;
  uint16_t mx = 0;
  uint32_t sum = 0;

  for (uint8_t i = 0; i < 8; ++i) {
    uint16_t r = analogRead(FB_PIN);
    if (r < mn) mn = r;
    if (r > mx) mx = r;
    sum += r;
  }

  sum -= mn;
  sum -= mx;
  float raw = sum / 6.0f;

  float vadc = (raw * ADC_VREF_CAL) / ADC_MAX;
  float vout = vadc * DIV_GAIN;

  // Apply linear calibration (for divider tolerance + ADC gain error)
  vout = vout * VOUT_GAIN_CAL + VOUT_OFFSET_CAL;

  return clampf(vout, 0.0f, 140.0f);
}

void setupTimer1_31kHz() {
  pinMode(PWM_PIN, OUTPUT);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // Mode 14: WGM13:0 = 1110
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // Non-inverting PWM on OC1A (D9)
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
    dHist[i] = dutyFromOcr() * 100.0f;
  }
  histCount = AVG_N;

  lastControlUs = micros();
  lastLcdVoltMs = millis();
  lastLcdDutyMs = millis();
}

void controlStep(float dt) {
  float vout = readVoutInstant();

  // Two-stage filter: first is implicit in trimmed mean, second is IIR.
  voutFilt += 0.20f * (vout - voutFilt);

  if (mode == STARTUP) {
    dutyCmd += STARTUP_RAMP_PER_S * dt;
    if (dutyCmd > STARTUP_DUTY_END) dutyCmd = STARTUP_DUTY_END;
    setDuty(dutyCmd);

    if (voutFilt >= STARTUP_EXIT_V) {
      if (++readyToRegCount >= MODE_CONFIRM_COUNT) {
        mode = REGULATE;

        float dutyFF = 1.0f - (VIN_NOMINAL / VOUT_TARGET);
        float err = VOUT_TARGET - voutFilt;

        iTerm = clampf(dutyCmd - dutyFF - Kp * err, -0.20f, 0.20f);
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

    dErrFilt += 0.10f * (dErr - dErrFilt);

    float dutyFF = 1.0f - (VIN_NOMINAL / VOUT_TARGET); // 0.91 nominal
    float pTerm = Kp * err;
    float dTerm = Kd * dErrFilt;

    float uUnsat = dutyFF + pTerm + iTerm + dTerm;
    float uSat = clampf(uUnsat, DUTY_MIN, DUTY_MAX);

    // Back-calculation anti-windup:
    // iDot = Ki*e + Kaw*(uSat - uUnsat)
    float iDot = Ki * err + Kaw * (uSat - uUnsat);

    // Freeze integration when far below target to avoid forcing permanent 95% display.
    if (err > 14.0f) {
      iDot = Kaw * (uSat - uUnsat);
    }

    iTerm += iDot * dt;
    iTerm = clampf(iTerm, -0.30f, 0.30f);

    float u = clampf(dutyFF + pTerm + iTerm + dTerm, DUTY_MIN, DUTY_MAX);

    // Low-Vin assist for reaching 100V without sticking forever at 95%:
    // only active when significantly below target.
    if (voutFilt < 86.0f && u < 0.93f) {
      u = 0.93f;
    }

    // Overshoot guard with soft recovery (accuracy near 100V)
    if (voutFilt > 103.5f) {
      u = 0.10f;
      iTerm = clampf(iTerm, -0.10f, 0.10f);
    }

    setDuty(u);

    if (voutFilt < STARTUP_REENTER_V) {
      if (++backToStartCount >= MODE_CONFIRM_COUNT) {
        mode = STARTUP;
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

  vDisplay += 0.22f * (vAvg - vDisplay);

  lcd.setCursor(0, 0);
  lcd.print("Vout:");
  lcd.print(vDisplay, 2);
  lcd.print("V  ");
}

void updateLcdDutyLine() {
  float vAvg, dAvg;
  computeDisplayAverages(vAvg, dAvg);

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

    if (dt < 0.0002f) dt = 0.0002f;
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
