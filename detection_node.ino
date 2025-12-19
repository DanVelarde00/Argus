// ESP32 Detection Node - Radar + GPS + IMU
// Board: FireBeetle 2 ESP32-E
// Optimized for power efficiency with dynamic sampling rates
// Sends detections to central gateway via ESP-NOW

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <DFRobot_C4001.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
static const int RADAR_RX_PIN = D2;   // GPIO for radar RX
static const int RADAR_TX_PIN = D3;   // GPIO for radar TX
static const int GPS_RX_PIN   = 17;   // GPIO 17
static const int GPS_TX_PIN   = 16;   // GPIO 16
static const int I2C_SDA_PIN  = 21;
static const int I2C_SCL_PIN  = 22;

// ============================================================================
// TIMING CONSTANTS - Dynamic for Power Efficiency
// ============================================================================
static const uint32_t SAMPLE_INTERVAL_IDLE_MS    = 200;  // Radar sampling when idle
static const uint32_t SAMPLE_INTERVAL_ACTIVE_MS  = 100;  // Radar sampling during detection
static const uint32_t RADAR_RETRY_MS             = 1000;
static const uint32_t IMU_RETRY_MS               = 750;
static const uint32_t IMU_SAMPLE_IDLE_MS         = 100;  // IMU when idle
static const uint32_t IMU_SAMPLE_ACTIVE_MS       = 50;   // IMU during detection
static const uint32_t GPS_READ_IDLE_MS           = 1000; // GPS 1Hz when idle
static const uint32_t GPS_READ_ACTIVE_MS         = 500;  // GPS 2Hz during detection
static const uint32_t STATUS_PRINT_INTERVAL_MS   = 5000; // Status messages

// ============================================================================
// SENSOR CONSTANTS
// ============================================================================
static const float MAG_DECLINATION_DEG = -13.0f;
static const float MOUNT_OFFSET_DEG    = 0.0f;

static const uint8_t ACC_ADDR_PRIMARY   = 0x19;
static const uint8_t ACC_ADDR_ALTERNATE = 0x18;
static const uint8_t MAG_ADDR           = 0x1E;

// ============================================================================
// DETECTION PARAMETERS - Enhanced for Confidence
// ============================================================================
static const float    MIN_STRONG_ENERGY_NORM    = 0.20f;  // Slightly higher threshold
static const uint8_t  REQUIRED_STRONG_SAMPLES   = 5;      // More samples for confidence
static const uint32_t DETECTION_HOLD_MS         = 500;    // Longer hold time
static const uint32_t CONFIRMED_DETECTION_MS    = 1000;   // Must persist 1+ seconds
static const float    MAX_REASONABLE_SPEED_MPS  = 20.0f;
static const float    MAX_RANGE_METERS          = 23.0f;
static const float    ENERGY_SMOOTH_ALPHA       = 0.25f;
static const float    ENERGY_NOISE_TRACK_ALPHA  = 0.05f;
static const float    ENERGY_NOISE_MARGIN       = 0.06f;
static const float    ENERGY_OUTPUT_SCALE       = 100.0f;

static const unsigned long CALIBRATION_DURATION_MS = 15000;

// ============================================================================
// GATEWAY MAC ADDRESS - UPDATE THIS
// ============================================================================
uint8_t gatewayAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // UPDATE WITH GATEWAY MAC

// ============================================================================
// NODE CONFIGURATION
// ============================================================================
static const uint8_t NODE_ID = 1;  // Unique ID for this detection node (1-255)
static uint32_t gDetectionSequence = 0;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct GpsData {
  bool valid = false;
  float latitude = 0.0f;
  float longitude = 0.0f;
  float altitude = 0.0f;
  uint8_t satellites = 0;
  uint8_t fixQuality = 0;
  float hdop = 99.9f;
  unsigned long lastUpdateMs = 0;
};

// Enhanced detection message with node tracking
struct DetectionMessage {
  uint8_t nodeId;                  // Which node sent this
  uint32_t sequenceNumber;         // Detection sequence number
  unsigned long timestamp;         // Milliseconds since node boot
  float latitude;                  // GPS coordinates
  float longitude;
  float altitude;
  float range_m;                   // Radar data
  float speed_mps;
  float heading_deg;               // IMU heading
  float energyLevel;               // Normalized 0-1
  uint8_t satellites;              // GPS quality
  uint8_t confidenceLevel;         // 0-100 detection confidence
};

struct AxisCalibration {
  float offset = 0.0f;
  float scale  = 1.0f;
};

struct MagCalibration {
  bool   seeded = false;
  float  minX   = 0.0f;
  float  maxX   = 0.0f;
  float  minY   = 0.0f;
  float  maxY   = 0.0f;
  float  minZ   = 0.0f;
  float  maxZ   = 0.0f;
};

struct RadarFilterState {
  uint8_t       strongSampleCount = 0;
  bool          hasActiveTarget   = false;
  unsigned long lastStrongMs      = 0;
  unsigned long firstDetectionMs  = 0;  // Track when detection started
  float         lastEnergyNorm    = 0.0f;
  float         lastRange         = 0.0f;
  float         lastSpeed         = 0.0f;
  float         smoothedEnergyNorm = 0.0f;
  float         noiseFloorNorm     = 0.02f;
  bool          energySeeded       = false;
  bool          detectionSent      = false;
  bool          detectionConfirmed = false;  // High confidence detection
};

struct ImuCalibrationData {
  bool accelValid = false;
  AxisCalibration accel[3];
  bool magValid = false;
  AxisCalibration mag[3];
};

struct ImuCalibrationRuntime {
  bool          active        = false;
  unsigned long startMs       = 0;
  uint32_t      samples       = 0;
  MagCalibration accelMinMax;
  MagCalibration magMinMax;
};

// ============================================================================
// GLOBAL STATE
// ============================================================================
DFRobot_C4001_UART radar(&Serial1, 9600, RADAR_RX_PIN, RADAR_TX_PIN);

static bool                 gRadarReady          = false;
static bool                 gImuReady            = false;
static uint8_t              gAccelAddr           = ACC_ADDR_PRIMARY;
static unsigned long        gLastSampleMs        = 0;
static unsigned long        gLastRadarRetryMs    = 0;
static unsigned long        gLastImuRetryMs      = 0;
static unsigned long        gLastHeadingSampleMs = 0;
static unsigned long        gLastGpsReadMs       = 0;
static unsigned long        gLastStatusPrintMs   = 0;
static float                gLastHeadingDeg      = NAN;
static MagCalibration       gMagCal;
static RadarFilterState     gRadarFilter;
static ImuCalibrationData   gImuCalData;
static ImuCalibrationRuntime gImuCalRuntime;
static GpsData              gGpsData;
static String               gNmeaBuffer = "";
static bool                 gPausedByGateway     = false;  // Detection pause flag

// ============================================================================
// DYNAMIC TIMING STATE
// ============================================================================
static uint32_t gCurrentSampleInterval = SAMPLE_INTERVAL_IDLE_MS;
static uint32_t gCurrentImuInterval    = IMU_SAMPLE_IDLE_MS;
static uint32_t gCurrentGpsInterval    = GPS_READ_IDLE_MS;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
static void finalizeImuCalibration();

// ============================================================================
// ESP-NOW CALLBACK
// ============================================================================
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("[ESP-NOW] Message sent successfully");
  } else {
    Serial.println("[ESP-NOW] Message send failed");
  }
}

// Callback for receiving pause/resume commands from gateway
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 1) return;

  // Simple command protocol: 'P' = pause, 'R' = resume
  if (data[0] == 'P') {
    gPausedByGateway = true;
    Serial.println("[GATEWAY] Detection paused - drone deployed");
  } else if (data[0] == 'R') {
    gPausedByGateway = false;
    gRadarFilter.detectionSent = false;
    gRadarFilter.detectionConfirmed = false;
    Serial.println("[GATEWAY] Detection resumed");
  }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

static bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission(true) == 0);
}

static void resetMinMax(MagCalibration &cal) {
  cal.seeded = false;
  cal.minX = cal.minY = cal.minZ = 0.0f;
  cal.maxX = cal.maxY = cal.maxZ = 0.0f;
}

static void updateMinMax(MagCalibration &cal, float x, float y, float z) {
  if (!cal.seeded) {
    cal.minX = cal.maxX = x;
    cal.minY = cal.maxY = y;
    cal.minZ = cal.maxZ = z;
    cal.seeded = true;
    return;
  }
  cal.minX = fminf(cal.minX, x);
  cal.maxX = fmaxf(cal.maxX, x);
  cal.minY = fminf(cal.minY, y);
  cal.maxY = fmaxf(cal.maxY, y);
  cal.minZ = fminf(cal.minZ, z);
  cal.maxZ = fmaxf(cal.maxZ, z);
}

static void computeAxisCalibration(const MagCalibration &cal, AxisCalibration (&out)[3]) {
  const float minScale = 1e-3f;
  out[0].offset = (cal.maxX + cal.minX) * 0.5f;
  out[1].offset = (cal.maxY + cal.minY) * 0.5f;
  out[2].offset = (cal.maxZ + cal.minZ) * 0.5f;
  out[0].scale = (cal.maxX - cal.minX) * 0.5f;
  out[1].scale = (cal.maxY - cal.minY) * 0.5f;
  out[2].scale = (cal.maxZ - cal.minZ) * 0.5f;
  out[0].scale = (fabsf(out[0].scale) < minScale) ? 1.0f : out[0].scale;
  out[1].scale = (fabsf(out[1].scale) < minScale) ? 1.0f : out[1].scale;
  out[2].scale = (fabsf(out[2].scale) < minScale) ? 1.0f : out[2].scale;
}

static void applyAxisCalibration(const AxisCalibration (&cal)[3], float &x, float &y, float &z) {
  const float minScale = 1e-6f;
  const float sx = (fabsf(cal[0].scale) < minScale) ? 1.0f : cal[0].scale;
  const float sy = (fabsf(cal[1].scale) < minScale) ? 1.0f : cal[1].scale;
  const float sz = (fabsf(cal[2].scale) < minScale) ? 1.0f : cal[2].scale;
  const float avgScale = (fabsf(sx) + fabsf(sy) + fabsf(sz)) / 3.0f;
  const float scaleFactor = (avgScale > minScale) ? avgScale : 1.0f;
  x = ((x - cal[0].offset) / sx) * scaleFactor;
  y = ((y - cal[1].offset) / sy) * scaleFactor;
  z = ((z - cal[2].offset) / sz) * scaleFactor;
}

static bool i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission(true) == 0);
}

static bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  uint8_t received = Wire.requestFrom(static_cast<int>(addr), static_cast<int>(length));
  if (received != length) {
    return false;
  }
  for (uint8_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

static inline int16_t toInt16(uint8_t hi, uint8_t lo) {
  return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

static float normalizeEnergy(uint16_t rawEnergy) {
  const float normalized = static_cast<float>(rawEnergy) / 65535.0f;
  if (!isfinite(normalized)) return 0.0f;
  if (normalized < 0.0f) return 0.0f;
  if (normalized > 1.0f) return 1.0f;
  return normalized;
}

static float wrapDegrees(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

// ============================================================================
// GPS FUNCTIONS
// ============================================================================

static float parseCoordinate(const String &coord, char hemisphere) {
  if (coord.length() == 0) return 0.0f;

  int dotPos = coord.indexOf('.');
  if (dotPos < 3) return 0.0f;

  float degrees = coord.substring(0, dotPos - 2).toFloat();
  float minutes = coord.substring(dotPos - 2).toFloat();

  float decimal = degrees + (minutes / 60.0f);

  if (hemisphere == 'S' || hemisphere == 'W') {
    decimal = -decimal;
  }

  return decimal;
}

static void parseGGA(const String &sentence) {
  int fieldStart = 7;
  int fieldIndex = 0;
  String fields[15];

  for (int i = fieldStart; i < sentence.length(); i++) {
    if (sentence[i] == ',' || sentence[i] == '*') {
      fieldIndex++;
      if (fieldIndex >= 15) break;
    } else {
      fields[fieldIndex] += sentence[i];
    }
  }

  gGpsData.fixQuality = fields[5].toInt();
  gGpsData.satellites = fields[6].toInt();
  gGpsData.hdop = fields[7].toFloat();

  if (gGpsData.fixQuality > 0) {
    gGpsData.latitude = parseCoordinate(fields[1], fields[2].charAt(0));
    gGpsData.longitude = parseCoordinate(fields[3], fields[4].charAt(0));
    gGpsData.altitude = fields[8].toFloat();
    gGpsData.valid = true;
    gGpsData.lastUpdateMs = millis();
  } else {
    gGpsData.valid = false;
  }
}

static void parseRMC(const String &sentence) {
  int fieldStart = 7;
  int fieldIndex = 0;
  String fields[13];

  for (int i = fieldStart; i < sentence.length(); i++) {
    if (sentence[i] == ',' || sentence[i] == '*') {
      fieldIndex++;
      if (fieldIndex >= 13) break;
    } else {
      fields[fieldIndex] += sentence[i];
    }
  }

  if (fields[1] == "A") {
    gGpsData.latitude = parseCoordinate(fields[2], fields[3].charAt(0));
    gGpsData.longitude = parseCoordinate(fields[4], fields[5].charAt(0));
    gGpsData.valid = true;
    gGpsData.lastUpdateMs = millis();
  }
}

static void processNmeaSentence(const String &sentence) {
  if (sentence.startsWith("$GPGGA") || sentence.startsWith("$GNGGA")) {
    parseGGA(sentence);
  } else if (sentence.startsWith("$GPRMC") || sentence.startsWith("$GNRMC")) {
    parseRMC(sentence);
  }
}

static void readGps(unsigned long nowMs) {
  if (nowMs - gLastGpsReadMs < gCurrentGpsInterval) {
    return;
  }
  gLastGpsReadMs = nowMs;

  while (Serial2.available() > 0) {
    char c = Serial2.read();

    if (c == '\n') {
      if (gNmeaBuffer.length() > 0) {
        processNmeaSentence(gNmeaBuffer);
        gNmeaBuffer = "";
      }
    } else if (c != '\r') {
      gNmeaBuffer += c;
      if (gNmeaBuffer.length() > 100) {
        gNmeaBuffer = "";
      }
    }
  }

  // Mark GPS as stale if no update in 5 seconds
  if (gGpsData.valid && (nowMs - gGpsData.lastUpdateMs > 5000)) {
    gGpsData.valid = false;
  }
}

// ============================================================================
// IMU FUNCTIONS
// ============================================================================

static bool readAccel(float &ax, float &ay, float &az) {
  uint8_t raw[6];
  if (!i2cReadBytes(gAccelAddr, 0x28 | 0x80, raw, sizeof(raw))) {
    return false;
  }
  const float gPerLsb = 0.001f;
  ax = toInt16(raw[1], raw[0]) * gPerLsb;
  ay = toInt16(raw[3], raw[2]) * gPerLsb;
  az = toInt16(raw[5], raw[4]) * gPerLsb;
  return true;
}

static bool readMag(float &mx, float &my, float &mz) {
  uint8_t raw[6];
  if (!i2cReadBytes(MAG_ADDR, 0x03, raw, sizeof(raw))) {
    return false;
  }
  const float invGainXY = 1.0f / 1100.0f;
  const float invGainZ  = 1.0f / 980.0f;
  mx = toInt16(raw[0], raw[1]) * invGainXY;
  mz = toInt16(raw[2], raw[3]) * invGainZ;
  my = toInt16(raw[4], raw[5]) * invGainXY;
  return true;
}

static void updateMagCalibration(float mx, float my, float mz) {
  if (!gMagCal.seeded) {
    gMagCal.minX = gMagCal.maxX = mx;
    gMagCal.minY = gMagCal.maxY = my;
    gMagCal.minZ = gMagCal.maxZ = mz;
    gMagCal.seeded = true;
    return;
  }
  gMagCal.minX = fminf(gMagCal.minX, mx);
  gMagCal.maxX = fmaxf(gMagCal.maxX, mx);
  gMagCal.minY = fminf(gMagCal.minY, my);
  gMagCal.maxY = fmaxf(gMagCal.maxY, my);
  gMagCal.minZ = fminf(gMagCal.minZ, mz);
  gMagCal.maxZ = fmaxf(gMagCal.maxZ, mz);
}

static bool initImu() {
  uint8_t accelCandidates[] = { ACC_ADDR_PRIMARY, ACC_ADDR_ALTERNATE };
  gAccelAddr = 0;
  for (uint8_t addr : accelCandidates) {
    if (i2cPing(addr)) {
      gAccelAddr = addr;
      break;
    }
  }
  if (gAccelAddr == 0 || !i2cPing(MAG_ADDR)) {
    return false;
  }
  bool ok = true;
  ok &= i2cWriteByte(gAccelAddr, 0x20, 0x57);
  ok &= i2cWriteByte(gAccelAddr, 0x23, 0x00);
  ok &= i2cWriteByte(MAG_ADDR, 0x00, 0x14);
  ok &= i2cWriteByte(MAG_ADDR, 0x01, 0x20);
  ok &= i2cWriteByte(MAG_ADDR, 0x02, 0x00);
  if (!ok) {
    return false;
  }
  gMagCal = MagCalibration{};
  gLastHeadingDeg = NAN;
  gImuCalData.accelValid = false;
  gImuCalData.magValid = false;
  if (gImuCalRuntime.active) {
    gImuCalRuntime.active = false;
    Serial.println("[IMU] Calibration aborted due to sensor reset");
  }
  return true;
}

static void serviceImuCalibration(float ax, float ay, float az,
                                  float mx, float my, float mz,
                                  unsigned long nowMs) {
  if (!gImuCalRuntime.active) return;

  updateMinMax(gImuCalRuntime.accelMinMax, ax, ay, az);
  updateMinMax(gImuCalRuntime.magMinMax, mx, my, mz);
  ++gImuCalRuntime.samples;

  if (nowMs - gImuCalRuntime.startMs >= CALIBRATION_DURATION_MS) {
    finalizeImuCalibration();
  }
}

static void startImuCalibration() {
  if (!gImuReady) {
    Serial.println("[IMU] Cannot calibrate: sensor offline");
    return;
  }
  gImuCalRuntime.active = true;
  gImuCalRuntime.startMs = millis();
  gImuCalRuntime.samples = 0;
  resetMinMax(gImuCalRuntime.accelMinMax);
  resetMinMax(gImuCalRuntime.magMinMax);
  Serial.println("[IMU] Calibration started - rotate device through all orientations (15s)");
}

static void cancelImuCalibration() {
  if (gImuCalRuntime.active) {
    gImuCalRuntime.active = false;
    Serial.println("[IMU] Calibration cancelled");
  }
}

static void finalizeImuCalibration() {
  if (!gImuCalRuntime.active) return;
  gImuCalRuntime.active = false;

  const bool accelValid = gImuCalRuntime.accelMinMax.seeded;
  const bool magValid = gImuCalRuntime.magMinMax.seeded;

  if (accelValid) {
    computeAxisCalibration(gImuCalRuntime.accelMinMax, gImuCalData.accel);
  }
  if (magValid) {
    computeAxisCalibration(gImuCalRuntime.magMinMax, gImuCalData.mag);
    gMagCal = gImuCalRuntime.magMinMax;
  }

  gImuCalData.accelValid = accelValid;
  gImuCalData.magValid = magValid;

  Serial.println("[IMU] Calibration complete");
  Serial.print("[IMU] Accel: ");
  Serial.print(accelValid ? "OK" : "FAIL");
  Serial.print(" | Mag: ");
  Serial.println(magValid ? "OK" : "FAIL");
}

static float computeHeadingDeg(unsigned long nowMs, bool &valid) {
  if (!gImuReady) {
    valid = false;
    return gLastHeadingDeg;
  }
  if (nowMs - gLastHeadingSampleMs < gCurrentImuInterval) {
    valid = isfinite(gLastHeadingDeg);
    return gLastHeadingDeg;
  }
  gLastHeadingSampleMs = nowMs;

  float ax, ay, az, mx, my, mz;
  if (!readAccel(ax, ay, az) || !readMag(mx, my, mz)) {
    gImuReady = false;
    valid = false;
    gLastHeadingDeg = NAN;
    return gLastHeadingDeg;
  }

  serviceImuCalibration(ax, ay, az, mx, my, mz, nowMs);

  if (!gImuCalData.magValid) {
    updateMagCalibration(mx, my, mz);
  }

  float axCal = ax, ayCal = ay, azCal = az;
  float mxCal = mx, myCal = my, mzCal = mz;

  if (gImuCalData.accelValid) {
    applyAxisCalibration(gImuCalData.accel, axCal, ayCal, azCal);
  }

  if (gImuCalData.magValid) {
    applyAxisCalibration(gImuCalData.mag, mxCal, myCal, mzCal);
  } else if (gMagCal.seeded) {
    AxisCalibration fallback[3];
    computeAxisCalibration(gMagCal, fallback);
    applyAxisCalibration(fallback, mxCal, myCal, mzCal);
  }

  const float roll  = atan2f(ayCal, azCal);
  const float denom = sqrtf(ayCal * ayCal + azCal * azCal);
  const float pitch = atanf(-axCal / (denom > 1e-6f ? denom : 1e-6f));

  const float sinRoll  = sinf(roll);
  const float cosRoll  = cosf(roll);
  const float sinPitch = sinf(pitch);
  const float cosPitch = cosf(pitch);

  const float mxh = mxCal * cosPitch + mzCal * sinPitch;
  const float myh = mxCal * sinRoll * sinPitch + myCal * cosRoll - mzCal * sinRoll * cosPitch;

  float heading = atan2f(-myh, mxh) * 180.0f / PI;
  heading = wrapDegrees(heading + MAG_DECLINATION_DEG + MOUNT_OFFSET_DEG);

  gLastHeadingDeg = heading;
  valid = true;
  return heading;
}

static void ensureRadarReady(unsigned long nowMs) {
  if (gRadarReady) return;
  if (nowMs - gLastRadarRetryMs < RADAR_RETRY_MS) return;

  gLastRadarRetryMs = nowMs;
  Serial.println("[RADAR] Retrying initialization...");
  Serial1.end();
  Serial1.begin(9600, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  gRadarReady = radar.begin();
  if (gRadarReady) {
    Serial.println("[RADAR] Online");
  }
}

static void ensureImuReady(unsigned long nowMs) {
  if (gImuReady) return;
  if (nowMs - gLastImuRetryMs < IMU_RETRY_MS) return;

  gLastImuRetryMs = nowMs;
  Serial.println("[IMU] Retrying initialization...");
  gImuReady = initImu();
  if (gImuReady) {
    Serial.println("[IMU] Online");
  }
}

// ============================================================================
// DETECTION & ESP-NOW FUNCTIONS
// ============================================================================

static uint8_t calculateConfidence() {
  // Confidence based on energy level and detection duration
  float energyConfidence = (gRadarFilter.smoothedEnergyNorm - gRadarFilter.noiseFloorNorm) /
                           (1.0f - gRadarFilter.noiseFloorNorm);
  energyConfidence = constrain(energyConfidence * 100.0f, 0.0f, 100.0f);

  unsigned long detectionDuration = millis() - gRadarFilter.firstDetectionMs;
  float durationConfidence = min(100.0f, (detectionDuration / 1000.0f) * 50.0f); // 50% per second

  return (uint8_t)((energyConfidence * 0.6f) + (durationConfidence * 0.4f));
}

static void sendDetectionViaEspNow(float range_m, float speed_mps, float heading_deg) {
  if (gPausedByGateway) {
    Serial.println("[DETECTION] Paused by gateway - suppressing transmission");
    return;
  }

  if (!gGpsData.valid) {
    Serial.println("[DETECTION] No GPS fix - cannot send");
    return;
  }

  DetectionMessage msg;
  msg.nodeId = NODE_ID;
  msg.sequenceNumber = ++gDetectionSequence;
  msg.timestamp = millis();
  msg.latitude = gGpsData.latitude;
  msg.longitude = gGpsData.longitude;
  msg.altitude = gGpsData.altitude;
  msg.range_m = range_m;
  msg.speed_mps = speed_mps;
  msg.heading_deg = heading_deg;
  msg.energyLevel = gRadarFilter.smoothedEnergyNorm;
  msg.satellites = gGpsData.satellites;
  msg.confidenceLevel = calculateConfidence();

  esp_err_t result = esp_now_send(gatewayAddress, (uint8_t *) &msg, sizeof(msg));

  Serial.println("\n========== DETECTION CONFIRMED ==========");
  Serial.printf("Node ID: %d | Seq: %d | Confidence: %d%%\n",
                msg.nodeId, msg.sequenceNumber, msg.confidenceLevel);
  Serial.printf("Location: %.6f, %.6f (Alt: %.1fm, Sats: %d)\n",
                msg.latitude, msg.longitude, msg.altitude, msg.satellites);
  Serial.printf("Target: Range=%.2fm Speed=%.2fm/s Heading=%.1f°\n",
                msg.range_m, msg.speed_mps, msg.heading_deg);
  Serial.printf("Energy: %.1f%% | Noise floor: %.1f%%\n",
                msg.energyLevel * 100.0f, gRadarFilter.noiseFloorNorm * 100.0f);
  Serial.printf("ESP-NOW: %s\n", result == ESP_OK ? "SENT" : "FAILED");
  Serial.println("=========================================\n");
}

static void updateDynamicTimings(bool activeDetection) {
  if (activeDetection) {
    gCurrentSampleInterval = SAMPLE_INTERVAL_ACTIVE_MS;
    gCurrentImuInterval = IMU_SAMPLE_ACTIVE_MS;
    gCurrentGpsInterval = GPS_READ_ACTIVE_MS;
  } else {
    gCurrentSampleInterval = SAMPLE_INTERVAL_IDLE_MS;
    gCurrentImuInterval = IMU_SAMPLE_IDLE_MS;
    gCurrentGpsInterval = GPS_READ_IDLE_MS;
  }
}

static void applyRadarFilter(unsigned long now,
                             uint16_t &targets,
                             float &range_m,
                             float &speed_mps,
                             float &energyNorm,
                             float heading_deg) {
  const bool hasRawTarget = (targets > 0) && isfinite(range_m) && (range_m >= 0.0f);
  const bool rangeAcceptable = hasRawTarget && (range_m <= MAX_RANGE_METERS);
  const bool speedReasonable = isfinite(speed_mps) && fabsf(speed_mps) <= MAX_REASONABLE_SPEED_MPS;

  // Initialize energy tracking
  if (!gRadarFilter.energySeeded) {
    gRadarFilter.smoothedEnergyNorm = energyNorm;
    gRadarFilter.noiseFloorNorm = energyNorm;
    gRadarFilter.energySeeded = true;
  } else {
    gRadarFilter.smoothedEnergyNorm += ENERGY_SMOOTH_ALPHA * (energyNorm - gRadarFilter.smoothedEnergyNorm);
    const bool updateNoise = !rangeAcceptable || (energyNorm < MIN_STRONG_ENERGY_NORM);
    if (updateNoise) {
      gRadarFilter.noiseFloorNorm +=
          ENERGY_NOISE_TRACK_ALPHA * (gRadarFilter.smoothedEnergyNorm - gRadarFilter.noiseFloorNorm);
    }
  }

  gRadarFilter.smoothedEnergyNorm = constrain(gRadarFilter.smoothedEnergyNorm, 0.0f, 1.0f);
  gRadarFilter.noiseFloorNorm = constrain(gRadarFilter.noiseFloorNorm, 0.0f, 1.0f);

  const float dynamicThreshold = max(MIN_STRONG_ENERGY_NORM, gRadarFilter.noiseFloorNorm + ENERGY_NOISE_MARGIN);
  const bool energyStrong = gRadarFilter.smoothedEnergyNorm >= dynamicThreshold;

  // Detection logic with prolonged confirmation
  if (rangeAcceptable && energyStrong && speedReasonable) {
    if (gRadarFilter.strongSampleCount == 0) {
      gRadarFilter.firstDetectionMs = now;
    }

    if (gRadarFilter.strongSampleCount < REQUIRED_STRONG_SAMPLES) {
      ++gRadarFilter.strongSampleCount;
    }

    gRadarFilter.lastStrongMs = now;
    gRadarFilter.lastEnergyNorm = gRadarFilter.smoothedEnergyNorm;
    gRadarFilter.lastRange = min(range_m, MAX_RANGE_METERS);
    gRadarFilter.lastSpeed = speed_mps;

    // Check if we have enough samples AND sufficient duration
    unsigned long detectionDuration = now - gRadarFilter.firstDetectionMs;
    bool durationMet = detectionDuration >= CONFIRMED_DETECTION_MS;
    bool samplesMet = gRadarFilter.strongSampleCount >= REQUIRED_STRONG_SAMPLES;

    if (!gRadarFilter.hasActiveTarget && samplesMet && durationMet) {
      gRadarFilter.hasActiveTarget = true;
      gRadarFilter.detectionConfirmed = true;
      gRadarFilter.detectionSent = false;
      updateDynamicTimings(true);  // Switch to active sampling
    }

    // Send detection message once per confirmed event
    if (gRadarFilter.detectionConfirmed && !gRadarFilter.detectionSent) {
      sendDetectionViaEspNow(gRadarFilter.lastRange, gRadarFilter.lastSpeed, heading_deg);
      gRadarFilter.detectionSent = true;
    }
  } else {
    // Lost target - use hold time before clearing
    if (now - gRadarFilter.lastStrongMs > DETECTION_HOLD_MS) {
      gRadarFilter.hasActiveTarget = false;
      gRadarFilter.strongSampleCount = 0;
      gRadarFilter.detectionSent = false;
      gRadarFilter.detectionConfirmed = false;
      gRadarFilter.firstDetectionMs = 0;
      updateDynamicTimings(false);  // Switch to idle sampling
    }
    if (!gRadarFilter.hasActiveTarget) {
      gRadarFilter.lastEnergyNorm = gRadarFilter.smoothedEnergyNorm;
    }
  }

  // Output filtered values
  if (!gRadarFilter.hasActiveTarget) {
    targets = 0;
    range_m = 0.0f;
    speed_mps = 0.0f;
    energyNorm = gRadarFilter.smoothedEnergyNorm;
  } else {
    targets = max<uint16_t>(targets, 1);
    range_m = gRadarFilter.lastRange;
    speed_mps = gRadarFilter.lastSpeed;
    energyNorm = gRadarFilter.lastEnergyNorm;
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n\n========================================");
  Serial.println("  ESP32 DETECTION NODE - ARGUS SYSTEM");
  Serial.println("========================================");
  Serial.printf("Node ID: %d\n", NODE_ID);
  Serial.flush();

  // WiFi and ESP-NOW initialization
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println("** UPDATE GATEWAY WITH THIS MAC **");

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init FAILED!");
  } else {
    Serial.println("[ESP-NOW] Initialized");
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    // Add gateway peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, gatewayAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("[ESP-NOW] Failed to add gateway peer");
    } else {
      Serial.println("[ESP-NOW] Gateway peer added");
    }
  }

  // Reduce WiFi power consumption (ESP-NOW still works)
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

  // I2C for IMU
  Serial.println("[I2C] Initializing...");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeout(5);

  // Radar (Serial1)
  Serial.println("[RADAR] Initializing...");
  Serial1.begin(9600, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  delay(100);
  gRadarReady = radar.begin();
  Serial.println(gRadarReady ? "[RADAR] Online" : "[RADAR] Failed - will retry");

  // GPS (Serial2)
  Serial.println("[GPS] Initializing...");
  Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(100);
  Serial2.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"); // GGA + RMC
  delay(100);
  Serial2.println("$PMTK220,1000*1F"); // 1Hz

  // IMU
  Serial.println("[IMU] Initializing...");
  gImuReady = initImu();
  Serial.println(gImuReady ? "[IMU] Online" : "[IMU] Failed - will retry");

  Serial.println("\n========================================");
  Serial.println("Commands: 'c' = calibrate IMU, 'x' = cancel");
  Serial.println("CSV: time,tgt,range,speed,energy,hdg,gps,lat,lon,sat");
  Serial.println("========================================\n");

  updateDynamicTimings(false);  // Start in idle mode
  Serial.flush();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  const unsigned long now = millis();

  // Handle serial commands
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == 'c' || c == 'C') {
      startImuCalibration();
    } else if (c == 'x' || c == 'X') {
      cancelImuCalibration();
    }
  }

  // Continuous GPS reading
  readGps(now);

  // Ensure sensors ready
  ensureRadarReady(now);
  ensureImuReady(now);

  // Status print (every 5 seconds)
  if (now - gLastStatusPrintMs >= STATUS_PRINT_INTERVAL_MS) {
    gLastStatusPrintMs = now;
    Serial.printf("[STATUS] Radar:%s IMU:%s GPS:%s%s Seq:%d\n",
                  gRadarReady ? "OK" : "ERR",
                  gImuReady ? "OK" : "ERR",
                  gGpsData.valid ? "FIX" : "NO",
                  gPausedByGateway ? " PAUSED" : "",
                  gDetectionSequence);
  }

  if (!gRadarReady) {
    return;
  }

  // Sample radar at dynamic interval
  if (now - gLastSampleMs < gCurrentSampleInterval) {
    return;
  }
  gLastSampleMs = now;

  // Read radar
  uint16_t targets = radar.getTargetNumber();
  float range_m = radar.getTargetRange();
  float speed_mps = radar.getTargetSpeed();
  uint16_t energyRaw = radar.getTargetEnergy();
  float energyNorm = normalizeEnergy(energyRaw);

  // Validate radar data
  if (!isfinite(range_m) || range_m < 0.0f) {
    range_m = 0.0f;
  } else if (range_m > MAX_RANGE_METERS) {
    range_m = MAX_RANGE_METERS;
  }
  if (!isfinite(speed_mps)) {
    speed_mps = 0.0f;
  }

  // Compute heading
  bool headingValid = false;
  const float headingDeg = computeHeadingDeg(now, headingValid);

  // Apply filtering and detection logic
  applyRadarFilter(now, targets, range_m, speed_mps, energyNorm, headingDeg);

  // CSV output (reduced when paused)
  if (!gPausedByGateway || (now % 1000 < 200)) {  // Less frequent output when paused
    Serial.print(now);
    Serial.print(',');
    Serial.print(targets);
    Serial.print(',');
    Serial.print(range_m, 3);
    Serial.print(',');
    Serial.print(speed_mps, 3);
    Serial.print(',');
    Serial.print(energyNorm * ENERGY_OUTPUT_SCALE, 2);
    Serial.print(',');
    Serial.print(headingValid ? String(headingDeg, 2) : "nan");
    Serial.print(',');
    Serial.print(gGpsData.valid ? "1" : "0");
    Serial.print(',');
    if (gGpsData.valid) {
      Serial.print(gGpsData.latitude, 6);
      Serial.print(',');
      Serial.print(gGpsData.longitude, 6);
      Serial.print(',');
      Serial.println(gGpsData.satellites);
    } else {
      Serial.println("nan,nan,0");
    }
  }
}
