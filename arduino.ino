#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca(0x40);

// ------------------ Fixed settings ------------------
static const float SERVO_FREQ = 50.0f;          // standard servo rate
static const uint16_t UPDATE_DT_MS = 20;        // interpolation update period

// Channels used (your setup)
static const uint8_t CH0 = 0;  // upper servo
static const uint8_t CH1 = 1;  // lower servo

// Your calibrated pulse limits (safety clamps)
static const uint16_t US_MIN_CH0 = 500;
static const uint16_t US_MAX_CH0 = 1600;

static const uint16_t US_MIN_CH1 = 1300;
static const uint16_t US_MAX_CH1 = 2500;

// Your calibrated "home": both point to -X at these pulses
static const uint16_t US0_CH0 = 1500;
static const uint16_t US0_CH1 = 1500;
// ---------------------------------------------------

static uint16_t chUs[16];

static uint16_t clampUs0(int32_t us) {
  if (us < (int32_t)US_MIN_CH0) return US_MIN_CH0;
  if (us > (int32_t)US_MAX_CH0) return US_MAX_CH0;
  return (uint16_t)us;
}
static uint16_t clampUs1(int32_t us) {
  if (us < (int32_t)US_MIN_CH1) return US_MIN_CH1;
  if (us > (int32_t)US_MAX_CH1) return US_MAX_CH1;
  return (uint16_t)us;
}

static uint16_t usToTicks(uint16_t us) {
  // PCA9685 ticks for a 50 Hz period:
  // period = 20,000 us, ticks = us*(4096/period)
  const float period_us = 1000000.0f / SERVO_FREQ;
  float ticks = (us * 4096.0f) / period_us;
  if (ticks < 0) ticks = 0;
  if (ticks > 4095) ticks = 4095;
  return (uint16_t)(ticks + 0.5f);
}

static void writeUS(uint8_t ch, uint16_t us) {
  chUs[ch] = us;
  pca.setPWM(ch, 0, usToTicks(us));
}

static void setBothNow(uint16_t us0, uint16_t us1) {
  writeUS(CH0, us0);
  writeUS(CH1, us1);
}

static void moveBothSmooth(uint16_t target0, uint16_t target1, uint32_t ms) {
  uint16_t start0 = chUs[CH0];
  uint16_t start1 = chUs[CH1];

  if (ms < UPDATE_DT_MS) {
    setBothNow(target0, target1);
    return;
  }

  uint32_t steps = ms / UPDATE_DT_MS;
  if (steps < 1) steps = 1;

  for (uint32_t i = 1; i <= steps; i++) {
    float t = (float)i / (float)steps;
    uint16_t u0 = (uint16_t)(start0 + t * ((int32_t)target0 - (int32_t)start0));
    uint16_t u1 = (uint16_t)(start1 + t * ((int32_t)target1 - (int32_t)start1));
    setBothNow(u0, u1);
    delay(UPDATE_DT_MS);
  }
}

static String readLine() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String out = line;
      line = "";
      out.trim();
      return out;
    }
    line += c;
  }
  return "";
}

static void printHelp() {
  Serial.println("Commands:");
  Serial.println("  home                    : smooth move to 1500/1500 (both point -X)");
  Serial.println("  p <us0> <us1>           : set CH0 and CH1 immediately");
  Serial.println("  g <us0> <us1> <ms>      : smooth move to targets over ms");
  Serial.println("  show                    : print current pulses");
  Serial.println("  help                    : show this help");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pca.begin();
  pca.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Initialize all channels to 1500us (safe startup)
  for (int i = 0; i < 16; i++) {
    chUs[i] = 1500;
    pca.setPWM(i, 0, usToTicks(1500));
  }

  // Move to calibrated home
  setBothNow(US0_CH0, US0_CH1);

  Serial.println("PCA9685 Motion Receiver Ready (CH0 upper, CH1 lower).");
  printHelp();
}

void loop() {
  String cmd = readLine();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("help")) { printHelp(); return; }

  if (cmd.equalsIgnoreCase("show")) {
    Serial.print("CH0="); Serial.print(chUs[CH0]); Serial.print(" us, ");
    Serial.print("CH1="); Serial.print(chUs[CH1]); Serial.println(" us");
    return;
  }

  if (cmd.equalsIgnoreCase("home")) {
    moveBothSmooth(US0_CH0, US0_CH1, 400);
    Serial.println("HOME");
    return;
  }

  // p us0 us1
  if (cmd.startsWith("p ")) {
    int s1 = cmd.indexOf(' ');
    int s2 = cmd.indexOf(' ', s1 + 1);
    if (s2 < 0) { Serial.println("ERR: p <us0> <us1>"); return; }
    int us0 = cmd.substring(s1 + 1, s2).toInt();
    int us1 = cmd.substring(s2 + 1).toInt();
    uint16_t u0 = clampUs0(us0);
    uint16_t u1 = clampUs1(us1);
    setBothNow(u0, u1);
    Serial.print("P "); Serial.print(u0); Serial.print(" "); Serial.println(u1);
    return;
  }

  // g us0 us1 ms
  if (cmd.startsWith("g ")) {
    int s1 = cmd.indexOf(' ');
    int s2 = cmd.indexOf(' ', s1 + 1);
    int s3 = cmd.indexOf(' ', s2 + 1);
    if (s2 < 0 || s3 < 0) { Serial.println("ERR: g <us0> <us1> <ms>"); return; }
    int us0 = cmd.substring(s1 + 1, s2).toInt();
    int us1 = cmd.substring(s2 + 1, s3).toInt();
    int ms  = cmd.substring(s3 + 1).toInt();
    if (ms < 0) ms = 0;

    uint16_t u0 = clampUs0(us0);
    uint16_t u1 = clampUs1(us1);
    moveBothSmooth(u0, u1, (uint32_t)ms);
    Serial.print("G "); Serial.print(u0); Serial.print(" "); Serial.print(u1);
    Serial.print(" "); Serial.println(ms);
    return;
  }

  Serial.println("Unknown. Type 'help'.");
}
