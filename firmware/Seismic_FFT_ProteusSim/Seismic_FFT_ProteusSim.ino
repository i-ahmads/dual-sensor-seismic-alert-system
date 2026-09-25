#include <Wire.h>
#include <arduinoFFT.h>

// Pin map
#define PIN_PIEZO_ADC     A0
#define PIN_LED_RED       8
#define PIN_LED_GREEN     9
#define PIN_LED_BLUE      10
#define PIN_BUZZER        11

#define LED_ON   LOW
#define LED_OFF  HIGH

#define MPU_ADDR          0x68
#define MPU_REG_PWR_MGMT1 0x6B
#define MPU_REG_ACCEL_XH  0x3B
#define ACCEL_SCALE       (1.0 / 16384.0)

#define SAMPLES           256
#define SAMPLING_FREQ_HZ  100.0
#define SAMPLE_INTERVAL_US (uint32_t)(1000000.0 / SAMPLING_FREQ_HZ)

// df = 0.390625 Hz/bin; k_low=ceil(1/df)=3, k_high=floor(10/df)=25
#define BAND_BIN_LO       3
#define BAND_BIN_HI       25

#define REQUIRED_CONSECUTIVE_WINDOWS 2

// Matches Seismic_FFT_Validation.m BAND ENERGY SUMMARY
float PIEZO_THRESHOLD = 15.0;

#define SIMULATE_ACCEL_CONSENSUS false
#define MPU_PRESENT true
#define DEBUG_RELAXED_PERSISTENCE false
#define DEBUG_PRINT_RAW_MPU false
#define DEBUG_VERIFY_MPU_LIVE DEBUG_RELAXED_PERSISTENCE

float vRealP[SAMPLES], vImagP[SAMPLES];
ArduinoFFT<float> fftPiezo(vRealP, vImagP, SAMPLES, SAMPLING_FREQ_HZ);

#define ACCEL_PRESET_THRESHOLD 0.3

double accelBaseline = 0.0;
double accelLatestTilt = 0.0;

int piezoConsecutive = 0;
int accelConsecutive = 0;

enum AlarmState { STANDBY, STAGE1_LOCAL, STAGE2_CONFIRMED };
AlarmState state = STANDBY;

void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_PWR_MGMT1);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

int16_t lastRawAx = 0, lastRawAy = 0;

// sqrt(ax^2+ay^2) tilt signal, not full 3-axis magnitude
double mpuReadTiltSignal() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL_XH);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  (void)az;
  lastRawAx = ax; lastRawAy = ay;

  double gx = ax * ACCEL_SCALE;
  double gy = ay * ACCEL_SCALE;
  return sqrt(gx * gx + gy * gy);
}

void setAlarm(AlarmState s) {
  state = s;
  switch (s) {
    case STANDBY:
      digitalWrite(PIN_LED_GREEN, LED_ON);
      digitalWrite(PIN_LED_BLUE, LED_OFF);
      digitalWrite(PIN_LED_RED, LED_OFF);
      noTone(PIN_BUZZER);
      break;
    case STAGE1_LOCAL:
      digitalWrite(PIN_LED_GREEN, LED_ON);
      digitalWrite(PIN_LED_BLUE, LED_OFF);
      digitalWrite(PIN_LED_RED, LED_OFF);
      noTone(PIN_BUZZER);
      break;
    case STAGE2_CONFIRMED:
      digitalWrite(PIN_LED_GREEN, LED_OFF);
      digitalWrite(PIN_LED_BLUE, LED_OFF);
      digitalWrite(PIN_LED_RED, LED_ON);
      tone(PIN_BUZZER, 3000);
      break;
  }
}

// bandEnergy = sum(mag^2) over BAND_BIN_LO..BAND_BIN_HI, matches .m script
float computeBandEnergy(ArduinoFFT<float> &fft, float *vReal, float *peakFreqOut) {
  fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  fft.compute(FFTDirection::Forward);
  fft.complexToMagnitude();

  float bandEnergy = 0;
  for (int i = BAND_BIN_LO; i <= BAND_BIN_HI; i++) {
    bandEnergy += vReal[i] * vReal[i];
  }

  float peakFreq, peakMag;
  fft.majorPeak(&peakFreq, &peakMag);
  if (peakFreqOut) *peakFreqOut = peakFreq;
  return bandEnergy;
}

bool gatewayConfirmedStage2 = false;
#define DEBUG_SIMULATE_GATEWAY_CONFIRM_AFTER_MS 3000
uint32_t stage1EnteredAt = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PIEZO_ADC, INPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  #if MPU_PRESENT
  Wire.begin();
  mpuInit();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  int whoAmI = Wire.read();
  Serial.print("[WHO_AM_I check] got: 0x");
  Serial.print(whoAmI, HEX);
  Serial.println(whoAmI == 0x68 ? "  -> OK" : "  -> FAIL, check wiring");
  #endif

  setAlarm(STANDBY);
  Serial.println("t_ms,piezo_energy,piezo_peak_hz,accel_active,accel_tilt_g,accel_raw_ax,accel_raw_ay,state");
}

void loop() {
  static uint32_t lastSample = 0;
  static int idx = 0;

  uint32_t now = micros();
  if (now - lastSample < SAMPLE_INTERVAL_US) return;
  lastSample = now;

  int raw = analogRead(PIN_PIEZO_ADC);
  vRealP[idx] = raw * (5.0 / 1023.0);
  vImagP[idx] = 0;

  #if MPU_PRESENT
  double mag = mpuReadTiltSignal();
  accelLatestTilt = mag;
  #if DEBUG_PRINT_RAW_MPU
  static int debugCounter = 0;
  if (++debugCounter >= 10) {
    debugCounter = 0;
    Serial.print("[RAW] ax="); Serial.print(lastRawAx);
    Serial.print(" ay="); Serial.print(lastRawAy);
    Serial.print(" tilt="); Serial.println(mag, 4);
  }
  #endif
  #endif

  idx++;
  if (idx >= SAMPLES) {
    idx = 0;

    float piezoPeakHz;
    float piezoEnergy = computeBandEnergy(fftPiezo, vRealP, &piezoPeakHz);

    bool accelActiveLatest = (accelLatestTilt > ACCEL_PRESET_THRESHOLD);

    if (SIMULATE_ACCEL_CONSENSUS && piezoEnergy > PIEZO_THRESHOLD) {
      accelActiveLatest = true;
    }

    int effectiveRequiredWindows = DEBUG_VERIFY_MPU_LIVE ? 1 : REQUIRED_CONSECUTIVE_WINDOWS;

    piezoConsecutive = (piezoEnergy > PIEZO_THRESHOLD) ? piezoConsecutive + 1 : 0;
    accelConsecutive = accelActiveLatest ? accelConsecutive + 1 : 0;

    bool consensus = (piezoConsecutive >= effectiveRequiredWindows) &&
                      (accelConsecutive >= effectiveRequiredWindows);

    if (state == STANDBY && consensus) {
      setAlarm(STAGE1_LOCAL);
      stage1EnteredAt = millis();
    } else if (state == STAGE1_LOCAL) {
      if (gatewayConfirmedStage2) {
        setAlarm(STAGE2_CONFIRMED);
      }
      #ifdef DEBUG_SIMULATE_GATEWAY_CONFIRM_AFTER_MS
      else if (millis() - stage1EnteredAt > DEBUG_SIMULATE_GATEWAY_CONFIRM_AFTER_MS) {
        Serial.println("[DEBUG] simulating gateway Stage2 confirmation");
        setAlarm(STAGE2_CONFIRMED);
      }
      #endif
    }

    Serial.print(millis()); Serial.print(",");
    Serial.print(piezoEnergy); Serial.print(",");
    Serial.print(piezoPeakHz); Serial.print(",");
    Serial.print(accelActiveLatest ? 1 : 0); Serial.print(",");
    Serial.print(accelLatestTilt); Serial.print(",");
    Serial.print(lastRawAx); Serial.print(",");
    Serial.print(lastRawAy); Serial.print(",");
    Serial.println((int)state);
  }
}