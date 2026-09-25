#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define PIEZO_PIN     34
#define LED_R_PIN     25
#define LED_G_PIN     26
#define LED_B_PIN     27
#define BUZZ_PIN      14

#define BUZZ_FREQ     3000
#define BUZZ_RES      8

#define OVERSAMPLE_COUNT 16
#define SAMPLE_INTERVAL_MS 10
#define WINDOW_SIZE 20

// VREF measured at 0.22V, target 0.889V - flags clipping, doesn't correct it
#define VREF_FLOOR_MV 220
#define CLIP_MARGIN_MV 20

Adafruit_MPU6050 mpu;

float piezoRestMean = 0, piezoThreshold = 0;
float accelRestMean = 0, accelThreshold = 0;

float piezoBuf[WINDOW_SIZE];
float accelBuf[WINDOW_SIZE];
int bufIndex = 0;
bool bufFull = false;

unsigned long piezoTripTime = 0;
unsigned long accelTripTime = 0;
const unsigned long CONSENSUS_WINDOW_MS = 500;
bool alarmActive = false;

int clipFloorCount = 0;
int clipTotalCount = 0;

uint32_t readPiezoMv() {
  uint32_t sum = 0;
  for (int i = 0; i < OVERSAMPLE_COUNT; i++) sum += analogReadMilliVolts(PIEZO_PIN);
  uint32_t avg = sum / OVERSAMPLE_COUNT;

  clipTotalCount++;
  if (avg < CLIP_MARGIN_MV) clipFloorCount++;

  return avg;
}

float readAccelMagnitude() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return sqrt(a.acceleration.x * a.acceleration.x +
              a.acceleration.y * a.acceleration.y +
              a.acceleration.z * a.acceleration.z);
}

void reportClipping(const char *phase) {
  if (clipTotalCount == 0) return;
  float pct = 100.0 * clipFloorCount / clipTotalCount;
  Serial.print(phase);
  Serial.print(" - samples pinned near 0V floor: ");
  Serial.print(pct, 1);
  Serial.println("%");
  if (pct > 10.0) {
    Serial.println("WARNING: negative-half clipping - fix VREF divider before trusting thresholds.");
  }
  clipFloorCount = 0;
  clipTotalCount = 0;
}

// common-cathode, active-HIGH
void setAlarmIdle() {
  digitalWrite(LED_R_PIN, LOW);
  digitalWrite(LED_G_PIN, HIGH);
  digitalWrite(LED_B_PIN, LOW);
  ledcWrite(BUZZ_PIN, 0);
}

void setAlarmCalibrating() {
  digitalWrite(LED_R_PIN, LOW);
  digitalWrite(LED_G_PIN, LOW);
  digitalWrite(LED_B_PIN, HIGH);
  ledcWrite(BUZZ_PIN, 0);
}

void setAlarmTriggered(bool on) {
  digitalWrite(LED_R_PIN, on ? HIGH : LOW);
  digitalWrite(LED_G_PIN, LOW);
  digitalWrite(LED_B_PIN, LOW);
  ledcWrite(BUZZ_PIN, on ? 128 : 0);
}

void triggerAlarm() {
  if (!alarmActive) {
    alarmActive = true;
    Serial.println(">>> EARTHQUAKE DETECTED (piezo + accel consensus) <<<");
    Serial.println(">>> Alarm LATCHED - type 'c' to clear manually <<<");
  }
}

void clearAlarm() {
  alarmActive = false;
  piezoTripTime = 0;
  accelTripTime = 0;
  Serial.println("Alarm manually cleared - back to monitoring.");
  setAlarmIdle();
}

void recordCalibration() {
  setAlarmCalibrating();
  Serial.println("\n=== CALIBRATION START ===");
  Serial.println("Phase 1: keep the board still for 2 seconds...");

  clipFloorCount = 0; clipTotalCount = 0;
  float piezoSum = 0, accelSum = 0;
  int restSamples = 200;
  for (int i = 0; i < restSamples; i++) {
    piezoSum += readPiezoMv();
    accelSum += readAccelMagnitude();
    delay(SAMPLE_INTERVAL_MS);
  }
  piezoRestMean = piezoSum / restSamples;
  accelRestMean = accelSum / restSamples;
  reportClipping("Rest phase");

  Serial.print("Rest -> piezo: "); Serial.print(piezoRestMean, 2);
  Serial.print(" mV, accel: "); Serial.print(accelRestMean, 3);
  Serial.println(" m/s^2");

  Serial.println("Phase 2: SHAKE the board now (5 seconds)...");
  clipFloorCount = 0; clipTotalCount = 0;
  float piezoPeak = piezoRestMean, accelPeak = accelRestMean;
  unsigned long shakeStart = millis();
  while (millis() - shakeStart < 5000) {
    float p = readPiezoMv();
    float a = readAccelMagnitude();
    if (fabs(p - piezoRestMean) > fabs(piezoPeak - piezoRestMean)) piezoPeak = p;
    if (fabs(a - accelRestMean) > fabs(accelPeak - accelRestMean)) accelPeak = a;
    delay(SAMPLE_INTERVAL_MS);
  }
  reportClipping("Shake phase");

  Serial.print("Shake peak -> piezo: "); Serial.print(piezoPeak, 2);
  Serial.print(" mV, accel: "); Serial.print(accelPeak, 3);
  Serial.println(" m/s^2");

  const float THRESH_FACTOR = 0.4;
  piezoThreshold = piezoRestMean + THRESH_FACTOR * fabs(piezoPeak - piezoRestMean);
  accelThreshold = accelRestMean + THRESH_FACTOR * fabs(accelPeak - accelRestMean);

  Serial.print("Thresholds -> piezo: "); Serial.print(piezoThreshold, 2);
  Serial.print(" mV, accel: "); Serial.print(accelThreshold, 3);
  Serial.println(" m/s^2");
  Serial.println("=== CALIBRATION COMPLETE ('r' to redo, 'c' to clear alarm) ===\n");

  setAlarmIdle();
}

void checkDetection() {
  float piezoNow = readPiezoMv();
  float accelNow = readAccelMagnitude();

  piezoBuf[bufIndex] = piezoNow;
  accelBuf[bufIndex] = accelNow;
  bufIndex = (bufIndex + 1) % WINDOW_SIZE;
  if (bufIndex == 0) bufFull = true;

  int count = bufFull ? WINDOW_SIZE : bufIndex;
  float piezoRms = 0, accelRms = 0;
  for (int i = 0; i < count; i++) {
    piezoRms += sq(piezoBuf[i] - piezoRestMean);
    accelRms += sq(accelBuf[i] - accelRestMean);
  }
  piezoRms = sqrt(piezoRms / count) + piezoRestMean;
  accelRms = sqrt(accelRms / count) + accelRestMean;

  unsigned long now = millis();
  if (piezoRms > piezoThreshold) piezoTripTime = now;
  if (accelRms > accelThreshold) accelTripTime = now;

  bool consensus = (now - piezoTripTime < CONSENSUS_WINDOW_MS) &&
                    (now - accelTripTime < CONSENSUS_WINDOW_MS);

  if (consensus) {
    triggerAlarm();
  }

  if (alarmActive) {
    bool blinkOn = (now / 200) % 2 == 0;
    setAlarmTriggered(blinkOn);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  digitalWrite(LED_R_PIN, LOW);
  digitalWrite(LED_G_PIN, LOW);
  digitalWrite(LED_B_PIN, LOW);

  ledcAttach(BUZZ_PIN, BUZZ_FREQ, BUZZ_RES);

  analogReadResolution(12);
  analogSetPinAttenuation(PIEZO_PIN, ADC_11db);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  Serial.println("MPU6050 ready.");
  Serial.println("NOTE: VREF measured at 0.22V (target 0.889V). Expect asymmetric");
  Serial.println("clipping on negative half-cycles until the divider is corrected.");
  recordCalibration();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') recordCalibration();
    if (c == 'c' || c == 'C') clearAlarm();
  }
  checkDetection();
  delay(SAMPLE_INTERVAL_MS);
}