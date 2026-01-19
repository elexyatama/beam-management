#include <WiFi.h>
#include <PubSubClient.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

// ==================== CONFIG ====================
const char* ssid        = "-";
const char* password    = "-";
const char* mqtt_server = "broker.hivemq.com";
const int   mqtt_port   = 1883;
const char* mqtt_topic  = "beam/geofence/sync";

#define GEOFENCE_FILE "/geofences.json"
unsigned long outsideStopStartTime = 0;  // Waktu mulai kondisi berhenti di luar zona
const unsigned long OUTSIDE_STOP_DELAY = 10000;  // 10 detik 

// ==================== HARDWARE ====================
#define BUZZER_PIN 27
#define LED1_PIN 25
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

// ==================== GEOFENCE STRUCT ====================
#define MAX_GEOFENCES 20
#define MAX_POINTS    20

struct Geofence {
  String id;
  String name;
  float coords[MAX_POINTS][2];
  int pointCount;
};

Geofence geofences[MAX_GEOFENCES];
int geofenceCount = 0;

bool gpsDebugMode = false;

// ==================== MQTT & WIFI ====================
WiFiClient espClient;
PubSubClient client(espClient);

// ==================== BUZZER SOUNDS ====================

// Tambahkan variabel global (di luar fungsi)
bool buzzerActive = false;
unsigned long lastBeepTime = 0;
const unsigned long beepInterval = 800;  // beep setiap 800 ms (bisa diatur)

// Fungsi untuk aktifkan buzzer (pola beep pendek)
void triggerBuzzer() {
  if (millis() - lastBeepTime >= beepInterval) {
    tone(BUZZER_PIN, 2000, 200);  // beep 2000 Hz selama 200 ms
    lastBeepTime = millis();
  }
}

void playTone(int freq, int duration) {
  tone(BUZZER_PIN, freq, duration);
  delay(duration + 10);
  noTone(BUZZER_PIN);
}

void beepShort() { playTone(1500, 80); }

void beepMqttConnected() {
  playTone(800, 100); delay(100);
  playTone(1200, 120); delay(100);
  playTone(1600, 200);
}

void beepSyncReceived() {
  playTone(900, 80); delay(80);
  playTone(1100, 80); delay(80);
  playTone(1400, 200);
}

void beepEnteredZone() {
  playTone(600, 300); delay(100);
  playTone(600, 300); delay(100);
  playTone(800, 600);
}

// ==================== LITTLEFS STORAGE ====================
bool saveGeofencesToFlash() {
  DynamicJsonDocument doc(8192);
  JsonArray array = doc.to<JsonArray>();

  for (int i = 0; i < geofenceCount; i++) {
    JsonObject obj = array.createNestedObject();
    obj["id"]   = geofences[i].id;
    obj["name"] = geofences[i].name;

    JsonArray coords = obj.createNestedArray("coords");
    for (int j = 0; j < geofences[i].pointCount; j++) {
      JsonArray point = coords.createNestedArray();
      point.add(geofences[i].coords[j][0]); // lat
      point.add(geofences[i].coords[j][1]); // lng
    }
  }

  File file = LittleFS.open(GEOFENCE_FILE, "w");
  if (!file) {
    Serial.println("Failed to open file for writing");
    return false;
  }
  serializeJson(doc, file);
  file.close();

  Serial.printf("Geofences SAVED to flash (%d total)\n", geofenceCount);
  return true;
}

bool loadGeofencesFromFlash() {
  if (!LittleFS.exists(GEOFENCE_FILE)) {
    Serial.println("No saved geofences found in flash");
    return false;
  }

  File file = LittleFS.open(GEOFENCE_FILE, "r");
  if (!file) {
    Serial.println("Failed to open saved geofences");
    return false;
  }

  DynamicJsonDocument doc(8192 * 8192);
  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) {
    Serial.println("Failed to parse saved JSON");
    return false;
  }

  geofenceCount = 0;
  JsonArray arr = doc.as<JsonArray>();

  for (JsonObject o : arr) {
    if (geofenceCount >= MAX_GEOFENCES) break;

    geofences[geofenceCount].id   = o["id"].as<String>();
    geofences[geofenceCount].name = o["name"].as<String>();

    JsonArray coords = o["coords"];
    int n = 0;
    for (JsonVariant v : coords) {
      if (n >= MAX_POINTS) break;
      geofences[geofenceCount].coords[n][0] = v[0].as<float>();
      geofences[geofenceCount].coords[n][1] = v[1].as<float>();
      n++;
    }
    geofences[geofenceCount].pointCount = n;
    geofenceCount++;
  }

  Serial.printf("LOADED %d geofences from flash!\n", geofenceCount);
  beepShort(); beepShort(); // double beep = loaded from memory
  return true;
}

// ==================== WIFI ====================
void connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 50) {
    delay(300);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected → " + WiFi.localIP().toString());
    beepShort();
  } else {
    Serial.println("\nWiFi Failed!");
    while (1) delay(100);
  }
}

// ==================== MQTT ====================
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("MQTT connecting...");
    String clientId = "ESP32_" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println(" CONNECTED!");
      client.subscribe(mqtt_topic);
      beepMqttConnected();
    } else {
      Serial.print(" failed, rc=");
      Serial.println(client.state());
      delay(3000);
    }
  }
}
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  payload[len] = '\0';
  String msg = String((char*)payload);

  Serial.println("\n=== MQTT Message ===");
  Serial.println("Topic: " + String(topic));
  Serial.println(msg);

  if (String(topic) != mqtt_topic) return;

  DynamicJsonDocument doc(24576);
  DeserializationError err = deserializeJson(doc, msg);
  if (err || doc["command"] != "SYNC_GEOFENCES") {
    playTone(300, 800);
    return;
  }

  // ======================================================
  // LATENCY CALCULATION (64-bit safe)
  // ======================================================
  unsigned long long sentTime = 0;

  if (doc.containsKey("timestamp")) {
      sentTime = doc["timestamp"].as<unsigned long long>();
      unsigned long recvMillis = millis();

      static long long timeOffset = -1;
      unsigned long long latency = 0;

      if (timeOffset < 0) {
          // First packet: sync time difference
          timeOffset = (long long)sentTime - (long long)recvMillis;

          Serial.printf("Time offset established: %lld\n", timeOffset);
      }

      unsigned long long adjustedNow = recvMillis + timeOffset;
      latency = adjustedNow - sentTime;

      Serial.printf("Latency: %llu ms\n", latency);
  } else {
      Serial.println("No timestamp received in payload");
  }


  // ======================================================
  // PARSE GEOFENCES
  // ======================================================
  geofenceCount = 0;
  JsonArray arr = doc["data"];

  for (JsonObject o : arr) {
    if (geofenceCount >= MAX_GEOFENCES) break;

    geofences[geofenceCount].id   = o["id"].as<String>();
    geofences[geofenceCount].name = o["name"].as<String>();

    JsonArray coords = o["coords"];
    int n = 0;

    for (JsonVariant v : coords) {
      if (n >= MAX_POINTS) break;
      geofences[geofenceCount].coords[n][0] = v[0].as<float>();
      geofences[geofenceCount].coords[n][1] = v[1].as<float>();
      n++;
    }

    geofences[geofenceCount].pointCount = n;
    geofenceCount++;
  }

  Serial.printf("SYNCED %d geofences → Saving to flash...\n", geofenceCount);
  saveGeofencesToFlash();
  beepSyncReceived();
}



// ==================== POINT IN POLYGON ====================
bool isPointInPolygon(float lat, float lon, Geofence &g) {
  int crossings = 0;
  for (int i = 0, j = g.pointCount - 1; i < g.pointCount; j = i++) {
    float y1 = g.coords[i][0], x1 = g.coords[i][1];
    float y2 = g.coords[j][0], x2 = g.coords[j][1];

    if ((y1 > lat) != (y2 > lat) &&
        (lon < x1 + (x2 - x1) * (lat - y1) / (y2 - y1 + 1e-9)))
      crossings++;
  }
  return crossings & 1;
}

// === CEK DALAM ATAU LUAR ZONA ===
void checkGeofences() {
  // Pastikan GPS valid dan ada geofence
  if (!gps.location.isValid() || geofenceCount == 0) {
    noTone(BUZZER_PIN);
    buzzerActive = false;
    digitalWrite(LED1_PIN, LOW);
    
    // Reset timer jika GPS tidak valid
    outsideStopStartTime = 0;
    
    return;
  }

  float currentSpeed = gps.speed.kmph();
  bool isStopped = (currentSpeed < 5.0);  // Saran: gunakan 5 lagi, bukan 1 (lihat catatan bawah)
  bool isOutsideAllZones = true;

  // Cek apakah di dalam salah satu zona
  for (int i = 0; i < geofenceCount; i++) {
    if (isPointInPolygon(gps.location.lat(), gps.location.lng(), geofences[i])) {
      isOutsideAllZones = false;
      break;
    }
  }

  // Kontrol LED1
  if (isOutsideAllZones) {
    digitalWrite(LED1_PIN, HIGH);
  } else {
    digitalWrite(LED1_PIN, LOW);
  }

  // Logika alarm dengan delay 10 detik
  if (isStopped && isOutsideAllZones) {
    // Kondisi berbahaya sedang aktif
    if (outsideStopStartTime == 0) {
      // Baru pertama kali masuk kondisi ini
      outsideStopStartTime = millis();
      Serial.println("Peringatan: Kendaraan berhenti di luar zona. Timer 10 detik dimulai...");
    }

    // Hitung sudah berapa lama kondisi ini bertahan
    unsigned long elapsed = millis() - outsideStopStartTime;

    if (elapsed >= OUTSIDE_STOP_DELAY) {
      // Sudah lebih dari 10 detik → aktifkan alarm
      if (!buzzerActive) {
        Serial.println("ALARM: Berhenti di luar zona lebih dari 10 detik! Harap pindah ke zona parkir resmi.");
        buzzerActive = true;
       // digitalWrite(LED1_PIN, HIGH);

      }
      triggerBuzzer();
    }
    // Jika belum 10 detik, buzzer tetap mati (tapi LED1 tetap nyala)
  }
  else {
    // Kondisi aman: sedang bergerak ATAU di dalam zona
    if (buzzerActive) {
      noTone(BUZZER_PIN);
      Serial.println("Aman: Alarm dimatikan (bergerak atau masuk zona).");
      buzzerActive = false;
      //digitalWrite(LED1_PIN, LOW);

    }

    // Reset timer karena sudah tidak dalam kondisi berbahaya
    outsideStopStartTime = 0;
  }
}

// ==================== SERIAL COMMANDS ====================
void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "LIST") {
    Serial.println("\n=== GEOFENCES ===");
    for (int i = 0; i < geofenceCount; i++) {
      Serial.printf("%d. %s (%d points)\n", i+1, geofences[i].name.c_str(), geofences[i].pointCount);
    }
    if (geofenceCount == 0) Serial.println("(none)");
  }
  else if (cmd == "INFO") {
    Serial.println("\n=== STATUS ===");
    Serial.printf("Geofences: %d\n", geofenceCount);
    Serial.printf("WiFi: %s (%d dBm)\n", WiFi.status() == WL_CONNECTED ? "OK" : "NO", WiFi.RSSI());
    Serial.printf("MQTT: %s\n", client.connected() ? "Connected" : "Disconnected");
    Serial.printf("GPS Fix: %s (%d sats)\n", gps.location.isValid() ? "YES" : "NO", gps.satellites.value());
    if (gps.location.isValid())
      Serial.printf("Lat/Lng: %.6f, %.6f\n", gps.location.lat(), gps.location.lng());
  }
  else if (cmd == "RESET") {
    geofenceCount = 0;
    Serial.println("Geofences cleared from RAM (flash still has data)");
  }
  else if (cmd == "FORMAT") {
    LittleFS.format();
    Serial.println("Flash storage formatted!");
  }
  else if (cmd == "GPS") {
    gpsDebugMode = !gpsDebugMode;
    Serial.println(gpsDebugMode ? "GPS debug ON" : "GPS debug OFF");
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  configTime(7 * 3600, 0, "pool.ntp.org");
  pinMode(LED1_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);

  // Mount LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed!");
    return;
  }
  Serial.println("LittleFS mounted");

  SerialGPS.begin(9600, SERIAL_8N1, 17, 16);

  // Load saved geofences first (offline mode works!)
  loadGeofencesFromFlash();

  connectWiFi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  client.setBufferSize(8192);

  reconnectMQTT();
}

// ==================== LOOP ====================
void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  static uint32_t lastCheck = 0;
  if (millis() - lastCheck > 1000) {
    lastCheck = millis();
    checkGeofences();

    if (gpsDebugMode && gps.location.isValid()) {
      Serial.printf("GPS: %.6f, %.6f (Sats: %d)\n",
                    gps.location.lat(), gps.location.lng(), gps.satellites.value());
    }
  }

  handleSerial();
}