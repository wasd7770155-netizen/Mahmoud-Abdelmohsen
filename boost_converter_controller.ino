#include <LiquidCrystal.h>

// ============================================================================
// Boost Converter Controller (Arduino UNO / ATmega328P)
// - Timer1 register PWM on D9 (OC1A) at ~31.25kHz
// - Feedback on A0 through divider: R2=100k (top), R3=4.7k (bottom)
// - LCD 16x2: RS=7, E=6, D4=5, D5=4, D6=3, D7=2
// ============================================================================

// ---------------------- Pins ----------------------
static const uint8_t PWM_PIN = 9;
static const uint8_t FB_PIN  = A0;

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

// ---------------------- Sensing ----------------------
static const float R_TOP = 100000.0f;
static const float R_BOT = 4700.0f;
static const float ADC_REF_V = 5.0f;
static const float ADC_MAX = 1023.0f;
static const float DIV_GAIN = (R_TOP + R_BOT) / R_BOT; // ~22.2766

static const float VIN_NOMINAL = 9.0f;
static const float VOUT_TARGET = 100.0f;

// ---------------------- PWM / Timer1 ----------------------
// Fast PWM mode 14 (TOP = ICR1), prescaler=1
// f_pwm = 16MHz / (1 * (1 + 511)) = 31.25kHz
static const uint16_t PWM_TOP = 511;
static const float DUTY_MIN = 0.04f;
static const float DUTY_MAX = 0.95f; // keep high-duty ability for 9V->100V

// ---------------------- Control ----------------------
static const uint32_t CONTROL_PERIOD_US = 333; // ~3kHz loop

enum ControlMode : uint8_t { STARTUP, REGULATE };
ControlMode mode = STARTUP;

// Fast startup to avoid getting stuck at low output.
static const float STARTUP_DUTY_BEGIN = 0.86f;
static const float STARTUP_DUTY_END   = 0.95f;
static const float STARTUP_RAMP_PER_S = 3.20f;
static const float STARTUP_EXIT_V = 72.0f;
static const float STARTUP_REENTER_V = 60.0f;
static const uint8_t MODE_CONFIRM_COUNT = 6;
uint8_t readyToRegCount = 0;
uint8_t backToStartCount = 0;

// PI-D gains (retuned to avoid permanent 95% saturation / windup)
static float Kp = 0.032f;
static float Ki = 0.55f;
static float Kd = 0.00030f;

float iTerm = 0.0f;
float prevErr = 0.0f;
float dErrFilt = 0.0f;
float dutyCmd = STARTUP_DUTY_BEGIN;
float voutFilt = 0.0f;

uint32_t lastControlUs = 0;

// ---------------------- LCD timing ----------------------
// Requested: duty cycle every 100ms
static const uint32_t LCD_VOLT_MS = 120;
static const uint32_t LCD_DUTY_MS = 100;
uint32_t lastLcdVoltMs = 0;
uint32_t lastLcdDutyMs = 0;

// Heavy moving average for stable display
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

static inline float dutyFromOcr() {
  return ((float)OCR1A) / PWM_TOP;
}

static inline void setDuty(float duty) {
  duty = clampf(duty, DUTY_MIN, DUTY_MAX);
  OCR1A = (uint16_t)(duty * PWM_TOP + 0.5f);
  dutyCmd = dutyFromOcr(); // display actual applied duty (not just requested)
}

float readVoutInstant() {
  // 2-sample average for speed+noise compromise at 3kHz loop
  uint16_t a = analogRead(FB_PIN);
  uint16_t b = analogRead(FB_PIN);
  float raw = 0.5f * (a + b);

  float vadc = (raw * ADC_REF_V) / ADC_MAX;
  float vout = vadc * DIV_GAIN;

  // clamp simulator spikes
  return clampf(vout, 0.0f, 130.0f);
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

  // Prescaler 1
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
  voutFilt += 0.24f * (vout - voutFilt);

  if (mode == STARTUP) {
    dutyCmd += STARTUP_RAMP_PER_S * dt;
    if (dutyCmd > STARTUP_DUTY_END) dutyCmd = STARTUP_DUTY_END;
    setDuty(dutyCmd);

    if (voutFilt >= STARTUP_EXIT_V) {
      if (++readyToRegCount >= MODE_CONFIRM_COUNT) {
        mode = REGULATE;
        float err = VOUT_TARGET - voutFilt;
        iTerm = clampf(dutyCmd - (1.0f - VIN_NOMINAL / VOUT_TARGET) - Kp * err, -0.25f, 0.25f);
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

    // derivative filtering for robustness
    dErrFilt += 0.12f * (dErr - dErrFilt);

    // fixed nominal feed-forward; PI closes model mismatch
    float dutyFF = 1.0f - (VIN_NOMINAL / VOUT_TARGET); // ~0.91

    float pTerm = Kp * err;
    float dTerm = Kd * dErrFilt;

    // Integrator with anti-windup by conditional integration
    float uNoI = dutyFF + pTerm + dTerm;
    float uPre = uNoI + iTerm;

    bool satHighPre = (uPre > DUTY_MAX);
    bool satLowPre  = (uPre < DUTY_MIN);
    bool allowIntegrate = !((satHighPre && err > 0.0f) || (satLowPre && err < 0.0f));

    // while far from target on low-vin cases, keep high duty but avoid integrator windup
    bool farFromTarget = (err > 12.0f);
    if (allowIntegrate && !farFromTarget) {
      iTerm += Ki * err * dt;
      iTerm = clampf(iTerm, -0.30f, 0.30f);
    }

    float u = clampf(uNoI + iTerm, DUTY_MIN, DUTY_MAX);

    // Strong but bounded assist for low-input operation (<11V cases in your test)
    if (voutFilt < 88.0f && u < 0.92f) {
      u = 0.92f;
    }

    // Overshoot guard: bleed duty when well above target
    if (voutFilt > 104.5f) {
      u = 0.12f;
      iTerm = clampf(iTerm, -0.10f, 0.15f);
    }

    setDuty(u);

    if (voutFilt < STARTUP_REENTER_V) {
      if (++backToStartCount >= MODE_CONFIRM_COUNT) {
        mode = STARTUP;
        dutyCmd = clampf(dutyCmd, STARTUP_DUTY_BEGIN, STARTUP_DUTY_END);
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

  // Heavy but responsive voltage display
  vDisplay += 0.22f * (vAvg - vDisplay);

  lcd.setCursor(0, 0);
  lcd.print("Vout:");
  lcd.print(vDisplay, 1);
  lcd.print("V   ");
}

void updateLcdDutyLine() {
  float vAvg, dAvg;
  computeDisplayAverages(vAvg, dAvg);

  // Update every 100ms as requested; show actual applied duty
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
