// ESP32 Gateway Controller - Central Hub for Detection Nodes
// Board: FireBeetle 2 ESP32-E
// Receives detections via ESP-NOW, communicates with Raspberry Pi via Serial
// Manages threat confirmation and drone deployment coordination

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ============================================================================
// CONFIGURATION
// ============================================================================
static const uint8_t MAX_NODES = 10;              // Maximum detection nodes
static const uint32_t NODE_TIMEOUT_MS = 30000;    // Node considered offline after 30s
static const uint32_t THREAT_CONFIRMATION_MS = 2000;  // Threat must persist 2 seconds
static const uint8_t MIN_CONFIDENCE_LEVEL = 70;   // Minimum confidence % for threat
static const uint32_t MISSION_TIMEOUT_MS = 120000; // Auto-resume after 2 minutes if no drone response
static const uint32_t STATUS_PRINT_INTERVAL_MS = 10000; // Status every 10s

// Serial to Raspberry Pi (hardware UART)
// Using Serial2 for Pi communication (Serial is USB debug)
static const int PI_RX_PIN = 16;  // GPIO 16 -> Pi TX
static const int PI_TX_PIN = 17;  // GPIO 17 -> Pi RX
static const uint32_t PI_BAUD_RATE = 115200;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Detection message from nodes (must match detection_node.ino)
struct DetectionMessage {
  uint8_t nodeId;
  uint32_t sequenceNumber;
  unsigned long timestamp;
  float latitude;
  float longitude;
  float altitude;
  float range_m;
  float speed_mps;
  float heading_deg;
  float energyLevel;
  uint8_t satellites;
  uint8_t confidenceLevel;
};

// Node registration and tracking
struct NodeInfo {
  bool active = false;
  uint8_t nodeId = 0;
  uint8_t macAddress[6] = {0};
  float latitude = 0.0f;          // Last known position
  float longitude = 0.0f;
  float altitude = 0.0f;
  unsigned long lastSeenMs = 0;
  uint32_t lastSequence = 0;
  uint32_t totalDetections = 0;
};

// Active threat tracking
struct ThreatInfo {
  bool active = false;
  uint8_t sourceNodeId = 0;
  float targetLat = 0.0f;          // Calculated target position
  float targetLon = 0.0f;
  float range_m = 0.0f;
  float heading_deg = 0.0f;
  float speed_mps = 0.0f;
  uint8_t confidence = 0;
  unsigned long firstDetectedMs = 0;
  unsigned long lastUpdateMs = 0;
  bool confirmed = false;
  bool droneSent = false;
};

// Mission state
enum MissionState {
  IDLE,
  THREAT_DETECTED,
  DRONE_DEPLOYING,
  DRONE_EN_ROUTE,
  DRONE_ENGAGING,
  MISSION_COMPLETE
};

// ============================================================================
// GLOBAL STATE
// ============================================================================
static NodeInfo gNodes[MAX_NODES];
static ThreatInfo gCurrentThreat;
static MissionState gMissionState = IDLE;
static unsigned long gMissionStartMs = 0;
static unsigned long gLastStatusPrintMs = 0;
static uint32_t gTotalDetectionsReceived = 0;
static uint32_t gTotalThreatsConfirmed = 0;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

static void macToString(const uint8_t *mac, char *buffer) {
  sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static int findNodeByMac(const uint8_t *mac) {
  for (int i = 0; i < MAX_NODES; i++) {
    if (gNodes[i].active &&
        memcmp(gNodes[i].macAddress, mac, 6) == 0) {
      return i;
    }
  }
  return -1;
}

static int findNodeById(uint8_t nodeId) {
  for (int i = 0; i < MAX_NODES; i++) {
    if (gNodes[i].active && gNodes[i].nodeId == nodeId) {
      return i;
    }
  }
  return -1;
}

static int findFreeNodeSlot() {
  for (int i = 0; i < MAX_NODES; i++) {
    if (!gNodes[i].active) {
      return i;
    }
  }
  return -1;
}

static void registerNode(const uint8_t *mac, uint8_t nodeId) {
  int idx = findNodeByMac(mac);
  if (idx < 0) {
    idx = findFreeNodeSlot();
    if (idx < 0) {
      Serial.println("[GATEWAY] ERROR: No free node slots!");
      return;
    }
  }

  gNodes[idx].active = true;
  gNodes[idx].nodeId = nodeId;
  memcpy(gNodes[idx].macAddress, mac, 6);
  gNodes[idx].lastSeenMs = millis();

  char macStr[18];
  macToString(mac, macStr);
  Serial.printf("[GATEWAY] Node registered: ID=%d MAC=%s\n", nodeId, macStr);
}

static void updateNodeInfo(int nodeIdx, const DetectionMessage &msg) {
  NodeInfo &node = gNodes[nodeIdx];
  node.latitude = msg.latitude;
  node.longitude = msg.longitude;
  node.altitude = msg.altitude;
  node.lastSeenMs = millis();
  node.lastSequence = msg.sequenceNumber;
  node.totalDetections++;
}

// ============================================================================
// GEOGRAPHIC CALCULATIONS
// ============================================================================

// Calculate target position from detection bearing and range
static void calculateTargetPosition(float nodeLat, float nodeLon,
                                    float bearing_deg, float range_m,
                                    float &targetLat, float &targetLon) {
  const float EARTH_RADIUS_M = 6371000.0f;
  float lat1 = nodeLat * DEG_TO_RAD;
  float lon1 = nodeLon * DEG_TO_RAD;
  float brng = bearing_deg * DEG_TO_RAD;
  float angDist = range_m / EARTH_RADIUS_M;

  float lat2 = asin(sin(lat1) * cos(angDist) +
                    cos(lat1) * sin(angDist) * cos(brng));
  float lon2 = lon1 + atan2(sin(brng) * sin(angDist) * cos(lat1),
                            cos(angDist) - sin(lat1) * sin(lat2));

  targetLat = lat2 * RAD_TO_DEG;
  targetLon = lon2 * RAD_TO_DEG;
}

// ============================================================================
// THREAT ASSESSMENT
// ============================================================================

static void evaluateThreat(const DetectionMessage &msg, int nodeIdx) {
  const unsigned long now = millis();

  // Check if this is a high-confidence detection
  if (msg.confidenceLevel < MIN_CONFIDENCE_LEVEL) {
    Serial.printf("[THREAT] Low confidence (%d%%) from Node %d - ignoring\n",
                  msg.confidenceLevel, msg.nodeId);
    return;
  }

  // Calculate target position
  float targetLat, targetLon;
  calculateTargetPosition(msg.latitude, msg.longitude,
                          msg.heading_deg, msg.range_m,
                          targetLat, targetLon);

  // Check if we have an active threat
  if (!gCurrentThreat.active) {
    // New threat
    gCurrentThreat.active = true;
    gCurrentThreat.sourceNodeId = msg.nodeId;
    gCurrentThreat.targetLat = targetLat;
    gCurrentThreat.targetLon = targetLon;
    gCurrentThreat.range_m = msg.range_m;
    gCurrentThreat.heading_deg = msg.heading_deg;
    gCurrentThreat.speed_mps = msg.speed_mps;
    gCurrentThreat.confidence = msg.confidenceLevel;
    gCurrentThreat.firstDetectedMs = now;
    gCurrentThreat.lastUpdateMs = now;
    gCurrentThreat.confirmed = false;
    gCurrentThreat.droneSent = false;

    Serial.println("\n========== NEW THREAT DETECTED ==========");
    Serial.printf("Source: Node %d | Confidence: %d%%\n",
                  msg.nodeId, msg.confidenceLevel);
    Serial.printf("Target Pos: %.6f, %.6f\n", targetLat, targetLon);
    Serial.printf("Range: %.1fm | Heading: %.1f° | Speed: %.1fm/s\n",
                  msg.range_m, msg.heading_deg, msg.speed_mps);
    Serial.println("Waiting for confirmation...");
    Serial.println("=========================================\n");

    gMissionState = THREAT_DETECTED;

  } else if (gCurrentThreat.sourceNodeId == msg.nodeId) {
    // Update from same node
    gCurrentThreat.targetLat = targetLat;
    gCurrentThreat.targetLon = targetLon;
    gCurrentThreat.range_m = msg.range_m;
    gCurrentThreat.heading_deg = msg.heading_deg;
    gCurrentThreat.speed_mps = msg.speed_mps;
    gCurrentThreat.confidence = max(gCurrentThreat.confidence, msg.confidenceLevel);
    gCurrentThreat.lastUpdateMs = now;

    // Check if threat has persisted long enough to confirm
    unsigned long duration = now - gCurrentThreat.firstDetectedMs;
    if (!gCurrentThreat.confirmed && duration >= THREAT_CONFIRMATION_MS) {
      gCurrentThreat.confirmed = true;
      gTotalThreatsConfirmed++;

      Serial.println("\n========== THREAT CONFIRMED ==========");
      Serial.printf("Duration: %lums | Confidence: %d%%\n",
                    duration, gCurrentThreat.confidence);
      Serial.printf("Target: %.6f, %.6f\n",
                    gCurrentThreat.targetLat, gCurrentThreat.targetLon);
      Serial.println("Initiating drone deployment...");
      Serial.println("======================================\n");

      // Send to Raspberry Pi
      sendThreatToPi();

      // Pause all nodes
      sendPauseCommand();

      gMissionState = DRONE_DEPLOYING;
      gMissionStartMs = now;
    }
  }
}

// ============================================================================
// ESP-NOW CALLBACKS
// ============================================================================

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(DetectionMessage)) {
    Serial.printf("[ESP-NOW] Invalid message size: %d bytes\n", len);
    return;
  }

  DetectionMessage msg;
  memcpy(&msg, data, sizeof(msg));

  gTotalDetectionsReceived++;

  // Register or update node
  int nodeIdx = findNodeByMac(info->src_addr);
  if (nodeIdx < 0) {
    registerNode(info->src_addr, msg.nodeId);
    nodeIdx = findNodeByMac(info->src_addr);
  }

  if (nodeIdx >= 0) {
    updateNodeInfo(nodeIdx, msg);

    // Print detection info
    Serial.printf("[DETECTION] Node %d Seq %d | Pos: %.6f,%.6f | Range: %.1fm Speed: %.1fm/s Hdg: %.1f° | Conf: %d%% Energy: %.1f%%\n",
                  msg.nodeId, msg.sequenceNumber,
                  msg.latitude, msg.longitude,
                  msg.range_m, msg.speed_mps, msg.heading_deg,
                  msg.confidenceLevel, msg.energyLevel * 100.0f);

    // Forward to Pi for logging
    sendDetectionToPi(msg);

    // Evaluate if this is a threat
    if (gMissionState == IDLE || gMissionState == THREAT_DETECTED) {
      evaluateThreat(msg, nodeIdx);
    }
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional: track send status
}

// ============================================================================
// RASPBERRY PI COMMUNICATION
// ============================================================================

static void sendDetectionToPi(const DetectionMessage &msg) {
  // JSON-like format for easy parsing on Pi
  Serial2.print("{\"type\":\"detection\",");
  Serial2.printf("\"node\":%d,\"seq\":%d,\"ts\":%lu,",
                 msg.nodeId, msg.sequenceNumber, msg.timestamp);
  Serial2.printf("\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,",
                 msg.latitude, msg.longitude, msg.altitude);
  Serial2.printf("\"range\":%.1f,\"speed\":%.1f,\"heading\":%.1f,",
                 msg.range_m, msg.speed_mps, msg.heading_deg);
  Serial2.printf("\"energy\":%.2f,\"conf\":%d,\"sats\":%d}\n",
                 msg.energyLevel, msg.confidenceLevel, msg.satellites);
}

static void sendThreatToPi() {
  Serial2.print("{\"type\":\"threat_confirmed\",");
  Serial2.printf("\"node\":%d,\"conf\":%d,",
                 gCurrentThreat.sourceNodeId, gCurrentThreat.confidence);
  Serial2.printf("\"target_lat\":%.6f,\"target_lon\":%.6f,",
                 gCurrentThreat.targetLat, gCurrentThreat.targetLon);
  Serial2.printf("\"range\":%.1f,\"speed\":%.1f,\"heading\":%.1f}\n",
                 gCurrentThreat.range_m, gCurrentThreat.speed_mps,
                 gCurrentThreat.heading_deg);
}

static void sendStatusToPi() {
  int activeNodes = 0;
  for (int i = 0; i < MAX_NODES; i++) {
    if (gNodes[i].active) activeNodes++;
  }

  const char *stateStr[] = {"IDLE", "THREAT_DETECTED", "DRONE_DEPLOYING",
                            "DRONE_EN_ROUTE", "DRONE_ENGAGING", "MISSION_COMPLETE"};

  Serial2.print("{\"type\":\"status\",");
  Serial2.printf("\"state\":\"%s\",\"nodes\":%d,\"detections\":%d,\"threats\":%d}\n",
                 stateStr[gMissionState], activeNodes,
                 gTotalDetectionsReceived, gTotalThreatsConfirmed);
}

// ============================================================================
// COMMAND BROADCASTING
// ============================================================================

static void sendPauseCommand() {
  uint8_t cmd = 'P';
  esp_now_send(NULL, &cmd, 1);  // Broadcast to all peers
  Serial.println("[GATEWAY] PAUSE command sent to all nodes");
}

static void sendResumeCommand() {
  uint8_t cmd = 'R';
  esp_now_send(NULL, &cmd, 1);  // Broadcast to all peers
  Serial.println("[GATEWAY] RESUME command sent to all nodes");
}

// ============================================================================
// RASPBERRY PI COMMAND HANDLER
// ============================================================================

static void handlePiCommand(const String &cmd) {
  cmd.trim();

  if (cmd.startsWith("DRONE_STATUS:")) {
    String status = cmd.substring(13);
    status.trim();

    if (status == "LAUNCHED") {
      gMissionState = DRONE_EN_ROUTE;
      Serial.println("[PI] Drone launched");
    } else if (status == "ENGAGING") {
      gMissionState = DRONE_ENGAGING;
      Serial.println("[PI] Drone engaging target");
    } else if (status == "COMPLETE") {
      gMissionState = MISSION_COMPLETE;
      Serial.println("[PI] Mission complete - resuming detection");

      // Reset threat
      gCurrentThreat.active = false;
      gCurrentThreat.confirmed = false;
      gCurrentThreat.droneSent = false;

      // Resume nodes
      sendResumeCommand();

      // Return to idle
      gMissionState = IDLE;
    } else if (status == "FAILED") {
      Serial.println("[PI] Mission failed - resuming detection");
      gCurrentThreat.active = false;
      sendResumeCommand();
      gMissionState = IDLE;
    }
  } else if (cmd == "STATUS") {
    sendStatusToPi();
  } else if (cmd == "RESUME") {
    Serial.println("[PI] Manual resume requested");
    gCurrentThreat.active = false;
    sendResumeCommand();
    gMissionState = IDLE;
  } else {
    Serial.printf("[PI] Unknown command: %s\n", cmd.c_str());
  }
}

static void readPiCommands() {
  static String buffer = "";

  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == '\n') {
      if (buffer.length() > 0) {
        handlePiCommand(buffer);
        buffer = "";
      }
    } else if (c != '\r') {
      buffer += c;
      if (buffer.length() > 200) {
        buffer = "";  // Overflow protection
      }
    }
  }
}

// ============================================================================
// NODE HEALTH MONITORING
// ============================================================================

static void checkNodeHealth() {
  const unsigned long now = millis();

  for (int i = 0; i < MAX_NODES; i++) {
    if (gNodes[i].active) {
      if (now - gNodes[i].lastSeenMs > NODE_TIMEOUT_MS) {
        char macStr[18];
        macToString(gNodes[i].macAddress, macStr);
        Serial.printf("[GATEWAY] Node %d (%s) TIMEOUT - marking inactive\n",
                      gNodes[i].nodeId, macStr);
        gNodes[i].active = false;
      }
    }
  }
}

static void printSystemStatus() {
  Serial.println("\n========== GATEWAY STATUS ==========");

  const char *stateStr[] = {"IDLE", "THREAT_DETECTED", "DRONE_DEPLOYING",
                            "DRONE_EN_ROUTE", "DRONE_ENGAGING", "MISSION_COMPLETE"};
  Serial.printf("Mission State: %s\n", stateStr[gMissionState]);

  int activeNodes = 0;
  Serial.println("\nActive Nodes:");
  for (int i = 0; i < MAX_NODES; i++) {
    if (gNodes[i].active) {
      activeNodes++;
      char macStr[18];
      macToString(gNodes[i].macAddress, macStr);
      Serial.printf("  Node %d (%s): Detections=%d LastSeen=%lus ago\n",
                    gNodes[i].nodeId, macStr, gNodes[i].totalDetections,
                    (millis() - gNodes[i].lastSeenMs) / 1000);
    }
  }
  if (activeNodes == 0) {
    Serial.println("  (none)");
  }

  Serial.printf("\nStatistics:\n");
  Serial.printf("  Total Detections Received: %d\n", gTotalDetectionsReceived);
  Serial.printf("  Total Threats Confirmed: %d\n", gTotalThreatsConfirmed);

  if (gCurrentThreat.active) {
    Serial.println("\nActive Threat:");
    Serial.printf("  Source: Node %d\n", gCurrentThreat.sourceNodeId);
    Serial.printf("  Target: %.6f, %.6f\n",
                  gCurrentThreat.targetLat, gCurrentThreat.targetLon);
    Serial.printf("  Confidence: %d%% | Confirmed: %s\n",
                  gCurrentThreat.confidence,
                  gCurrentThreat.confirmed ? "YES" : "NO");
  }

  Serial.println("====================================\n");
}

// ============================================================================
// MISSION TIMEOUT HANDLER
// ============================================================================

static void checkMissionTimeout() {
  if (gMissionState != IDLE && gMissionState != THREAT_DETECTED) {
    unsigned long elapsed = millis() - gMissionStartMs;
    if (elapsed > MISSION_TIMEOUT_MS) {
      Serial.println("[GATEWAY] Mission timeout - auto-resuming");
      gCurrentThreat.active = false;
      sendResumeCommand();
      gMissionState = IDLE;
    }
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n========================================");
  Serial.println("  ESP32 GATEWAY CONTROLLER - ARGUS");
  Serial.println("========================================");
  Serial.flush();

  // Initialize Serial2 for Raspberry Pi communication
  Serial2.begin(PI_BAUD_RATE, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
  Serial.println("[SERIAL] Pi UART initialized");

  // WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);
  Serial.print("[WIFI] MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println("** CONFIGURE NODES WITH THIS MAC **");

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init FAILED!");
    while (1) delay(1000);
  }

  Serial.println("[ESP-NOW] Initialized");
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // Lower power consumption
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

  Serial.println("\n========================================");
  Serial.println("Gateway ready - waiting for nodes...");
  Serial.println("Pi Commands: STATUS, RESUME, DRONE_STATUS:<status>");
  Serial.println("========================================\n");

  // Send initial status to Pi
  delay(1000);
  sendStatusToPi();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  const unsigned long now = millis();

  // Read commands from Raspberry Pi
  readPiCommands();

  // Check node health
  static unsigned long lastHealthCheck = 0;
  if (now - lastHealthCheck >= 5000) {
    lastHealthCheck = now;
    checkNodeHealth();
  }

  // Print status periodically
  if (now - gLastStatusPrintMs >= STATUS_PRINT_INTERVAL_MS) {
    gLastStatusPrintMs = now;
    printSystemStatus();
    sendStatusToPi();
  }

  // Check for mission timeout
  checkMissionTimeout();

  delay(10);  // Small delay to prevent tight looping
}
