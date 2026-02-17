#include <LiquidCrystal.h>
#include <math.h>

// ============================================================================
// High-Gain Boost Converter Controller (Proteus + Arduino UNO)
// - PWM on D9 (OC1A) using Timer1 register control (NO analogWrite)
// - Fast PWM Mode 14, TOP=ICR1=511 => ~31.25kHz, 9-bit resolution
// - Target: 9V -> 100V with robust anti-windup and simulation deadband lock
// ============================================================================

// ---------------------------- Pin Mapping -----------------------------------
static const uint8_t PWM_PIN = 9;   // OC1A
static const uint8_t FB_PIN  = A0;
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

// ------------------------ Electrical Parameters ------------------------------
static const float R_TOP = 100000.0f;  // 100k
static const float R_BOT = 4700.0f;    // 4.7k
static const float DIV_GAIN = (R_TOP + R_BOT) / R_BOT;

static const float ADC_VREF = 5.0f;
static const float ADC_MAX  = 1023.0f;

static const float VIN_NOMINAL = 9.0f;
static const float VOUT_TARGET = 100.0f;

// ------------------------ Timer1 / PWM Parameters ---------------------------
// Fast PWM mode 14 (WGM13:0 = 1110), TOP = ICR1
// f_pwm = F_CPU / (N * (1 + TOP)) = 16e6 / (1 * 512) = 31.25kHz
static const uint16_t PWM_TOP = 511;

static const float DUTY_MIN = 0.02f;
static const float DUTY_MAX = 0.96f; // CRITICAL: allow up to ~96% => OCR1A ~490/511

// -------------------------- Control Loop ------------------------------------
static const uint32_t CONTROL_PERIOD_US = 500; // 2kHz

// Deadband/hysteresis lock for simulation stability:
// If |error| < 0.5V -> freeze integrator and hold PWM output (no updates).
static const float DEADBAND_V = 0.5f;

// Aggressive PID for 9V->100V high-gain boost startup/regulation.
static float Kp = 0.080f;
static float Ki = 1.40f;
static float Kd = 0.00020f;

float iTerm = 0.0f;
float prevErr = 0.0f;
float dFilt = 0.0f;
float dutyCmd = 0.90f;

uint32_t lastControlUs = 0;

// ---------------------------- LCD / Display ---------------------------------
static const uint32_t LCD_PERIOD_MS = 250;
uint32_t lastLcdMs = 0;

// Simple moving average of 10 readings for displayed voltage.
static const uint8_t DISP_AVG_N = 10;
float vDispBuf[DISP_AVG_N] = {0.0f};
uint8_t vDispIdx = 0;
uint8_t vDispCount = 0;

// ---------------------------- Utility ---------------------------------------
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float ocrToDuty(uint16_t ocr) {
  return ((float)ocr) / PWM_TOP;
}

static inline uint16_t dutyToOcr(float duty) {
  duty = clampf(duty, DUTY_MIN, DUTY_MAX);
  return (uint16_t)(duty * PWM_TOP + 0.5f);
}

static inline void applyDuty(float duty) {
  uint16_t ocr = dutyToOcr(duty);
  OCR1A = ocr;
  dutyCmd = ocrToDuty(ocr); // store actual applied duty
}

float readVoutInstant() {
  // Light averaging for control path (fast + less switching noise)
  uint16_t a = analogRead(FB_PIN);
  uint16_t b = analogRead(FB_PIN);
  float raw = 0.5f * (a + b);

  float vadc = (raw * ADC_VREF) / ADC_MAX;
  float vout = vadc * DIV_GAIN;
  return clampf(vout, 0.0f, 140.0f);
}

void pushDisplayVoltage(float v) {
  vDispBuf[vDispIdx] = v;
  vDispIdx = (vDispIdx + 1) % DISP_AVG_N;
  if (vDispCount < DISP_AVG_N) vDispCount++;
}

float getDisplayVoltageAvg() {
  float sum = 0.0f;
  for (uint8_t i = 0; i < vDispCount; i++) sum += vDispBuf[i];
  return (vDispCount > 0) ? (sum / vDispCount) : 0.0f;
}

void setupTimer1_31kHz() {
  pinMode(PWM_PIN, OUTPUT);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // Fast PWM mode 14: WGM13:0 = 1110
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // Non-inverting PWM on OC1A (D9)
  TCCR1A |= (1 << COM1A1);

  // Prescaler = 1
  TCCR1B |= (1 << CS10);

  // TOP for 31.25kHz
  ICR1 = PWM_TOP;

  // Start near required high-gain duty
  OCR1A = dutyToOcr(0.90f);
  dutyCmd = ocrToDuty(OCR1A);
}

void setup() {
  analogReference(DEFAULT);
  pinMode(FB_PIN, INPUT);

  setupTimer1_31kHz();

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Boost Ctrl Init");

  float v0 = readVoutInstant();
  for (uint8_t i = 0; i < DISP_AVG_N; i++) vDispBuf[i] = v0;
  vDispCount = DISP_AVG_N;

  lastControlUs = micros();
  lastLcdMs = millis();
}

void controlStep(float dt) {
  float vout = readVoutInstant();
  pushDisplayVoltage(vout);

  // Feed-forward from boost relation for 9V nominal:
  // D = 1 - Vin/Vout => ~0.91 for 9V->100V
  float dutyFF = 1.0f - (VIN_NOMINAL / VOUT_TARGET);

  float err = VOUT_TARGET - vout;

  // ---------------- DEAD BAND LOCK ----------------
  // If we are within +/-0.5V around target, freeze controller output.
  // This avoids tiny high-frequency duty dithers that can trigger
  // Proteus "timestep too small" at high voltage.
  if (fabs(err) < DEADBAND_V) {
    return; // hold current PWM and integral (no update)
  }

  float derr = (err - prevErr) / dt;
  prevErr = err;

  // derivative low-pass (simple, robust)
  dFilt += 0.15f * (derr - dFilt);

  // Immediate integral reset on overshoot request from user:
  // if Vout > target, clear integral to stop sticking near high duty.
  if (vout > VOUT_TARGET) {
    iTerm = 0.0f;
  }

  float pTerm = Kp * err;
  float dTerm = Kd * dFilt;

  // tentative integration
  float iCand = iTerm + Ki * err * dt;

  // anti-windup: block integration if saturated in same direction
  float uUnsatCand = dutyFF + pTerm + dTerm + iCand;
  bool satHigh = (uUnsatCand > DUTY_MAX);
  bool satLow  = (uUnsatCand < DUTY_MIN);
  bool blockI  = (satHigh && err > 0.0f) || (satLow && err < 0.0f);

  if (!blockI) {
    iTerm = iCand;
  }

  iTerm = clampf(iTerm, -0.30f, 0.35f);

  float u = dutyFF + pTerm + dTerm + iTerm;

  // Assist to guarantee low-Vin climb without restricting top duty.
  if (vout < 85.0f && u < 0.93f) u = 0.93f;

  u = clampf(u, DUTY_MIN, DUTY_MAX);
  applyDuty(u);
}

void updateLcd() {
  float vAvg = getDisplayVoltageAvg();

  lcd.setCursor(0, 0);
  lcd.print("Vout:");
  lcd.print(vAvg, 1);
  lcd.print("V   ");

  lcd.setCursor(0, 1);
  lcd.print("D:");
  lcd.print(dutyCmd * 100.0f, 1);
  lcd.print("%    ");
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
  if ((uint32_t)(nowMs - lastLcdMs) >= LCD_PERIOD_MS) {
    lastLcdMs = nowMs;
    updateLcd();
  }
}
