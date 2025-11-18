/*
 * MiniOS WiFi Client
 * Versión ligera para conectar al backend centralizado
 *
 * Características:
 * - Conexión WebSocket al backend
 * - Identificación por MAC Address
 * - Control GPIO remoto
 * - Sensores DHT
 * - Actualizaciones OTA
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>
#include <DHT.h>

// ============================================
// CONFIGURACIÓN
// ============================================

#define FIRMWARE_VERSION "1.0.0"

// WiFi - Configurar aquí o vía Serial
String WIFI_SSID = "CASA ROJAS";
String WIFI_PASS = "STD2024....";

// Backend
String BACKEND_HOST = "minios.iot-robotics.cl";  // Dominio del backend
int BACKEND_PORT = 80;  // Puerto 80 con Nginx, 443 con SSL

// Hardware
#define MAX_GPIOS 20
#define MAX_DHT_SENSORS 4
#define LED_STATUS 2  // LED integrado

// ============================================
// ESTRUCTURAS
// ============================================

enum GPIOMode {
  MODE_OUTPUT,
  MODE_INPUT,
  MODE_INPUT_PULLUP,
  MODE_PWM
};

struct GPIOConfig {
  int pin;
  GPIOMode mode;
  int value;
  String name;
  bool active;
  bool loopEnabled;
  unsigned long loopInterval;
  unsigned long lastLoop;
};

struct DHTConfig {
  int pin;
  DHT* sensor;
  String name;
  String type;
  float temperature;
  float humidity;
  bool active;
  unsigned long readInterval;
  unsigned long lastRead;
};

// ============================================
// VARIABLES GLOBALES
// ============================================

WebSocketsClient webSocket;
Preferences preferences;

GPIOConfig gpios[MAX_GPIOS];
int gpioCount = 0;

DHTConfig dhtSensors[MAX_DHT_SENSORS];
int dhtCount = 0;

String deviceMac;
int deviceId = 0;
bool isRegistered = false;

unsigned long lastDataSend = 0;
const unsigned long DATA_SEND_INTERVAL = 5000;

unsigned long lastReconnect = 0;
const unsigned long RECONNECT_INTERVAL = 5000;

// OTA
bool otaInProgress = false;
int otaId = 0;
String otaFilename = "";
int otaFilesize = 0;
String otaChecksum = "";

// ============================================
// PROTOTIPOS DE FUNCIONES
// ============================================

void connectWiFi();
void connectWebSocket();
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);
void registerDevice();
void handleWebSocketMessage(const char* payload);
void handleConfig(JsonDocument& doc);
void handleCommand(JsonDocument& doc);
void sendSensorData();
void readDHTSensors();
void processGpioLoops();
void startOTA(int id, String filename, int filesize, String checksum);
void reportOTAStatus(const char* status, String error);
void loadConfig();
void saveConfig();
void handleSerial();

// ============================================
// SETUP
// ============================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("MiniOS WiFi Client v" FIRMWARE_VERSION);
  Serial.println("========================================");

  // LED de estado
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);

  // Obtener MAC
  deviceMac = WiFi.macAddress();
  Serial.print("MAC Address: ");
  Serial.println(deviceMac);

  // Cargar configuración guardada
  loadConfig();

  // Mostrar comandos disponibles
  Serial.println("\nComandos disponibles:");
  Serial.println("  wifi <ssid> <pass> - Configurar WiFi");
  Serial.println("  server <host> <port> - Configurar backend");
  Serial.println("  status - Ver estado");
  Serial.println("  reboot - Reiniciar");

  // Conectar WiFi
  if (WIFI_SSID.length() > 0) {
    connectWiFi();
  } else {
    Serial.println("\n⚠️  WiFi no configurado. Usa: wifi <ssid> <pass>");
  }
}

// ============================================
// LOOP PRINCIPAL
// ============================================

void loop() {
  // Procesar comandos Serial
  handleSerial();

  // WebSocket loop
  webSocket.loop();

  // Verificar conexión WiFi
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastReconnect > RECONNECT_INTERVAL) {
      lastReconnect = millis();
      connectWiFi();
    }
    return;
  }

  // Leer sensores DHT
  readDHTSensors();

  // Procesar loops GPIO
  processGpioLoops();

  // Enviar datos periódicamente
  if (isRegistered && millis() - lastDataSend > DATA_SEND_INTERVAL) {
    lastDataSend = millis();
    sendSensorData();
  }
}

// ============================================
// WIFI
// ============================================

void connectWiFi() {
  if (WIFI_SSID.length() == 0) return;

  Serial.print("Conectando a WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID.c_str(), WIFI_PASS.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi conectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    digitalWrite(LED_STATUS, HIGH);

    // Conectar al backend
    if (BACKEND_HOST.length() > 0) {
      connectWebSocket();
    } else {
      Serial.println("⚠️  Backend no configurado. Usa: server <host> <port>");
    }
  } else {
    Serial.println("\n❌ Error conectando WiFi");
    digitalWrite(LED_STATUS, LOW);
  }
}

// ============================================
// WEBSOCKET
// ============================================

void connectWebSocket() {
  Serial.print("Conectando a backend: ");
  Serial.print(BACKEND_HOST);
  Serial.print(":");
  Serial.println(BACKEND_PORT);

  webSocket.begin(BACKEND_HOST.c_str(), BACKEND_PORT, "/ws/device");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("📴 WebSocket desconectado");
      isRegistered = false;
      break;

    case WStype_CONNECTED:
      Serial.println("🔌 WebSocket conectado");
      registerDevice();
      break;

    case WStype_TEXT:
      handleWebSocketMessage((char*)payload);
      break;

    case WStype_ERROR:
      Serial.println("❌ WebSocket error");
      break;

    default:
      break;
  }
}

void registerDevice() {
  StaticJsonDocument<256> doc;
  doc["type"] = "register";
  doc["mac_address"] = deviceMac;
  doc["firmware_version"] = FIRMWARE_VERSION;
  doc["ip_address"] = WiFi.localIP().toString();

  String json;
  serializeJson(doc, json);
  webSocket.sendTXT(json);

  Serial.println("📱 Registro enviado al backend");
}

void handleWebSocketMessage(const char* payload) {
  StaticJsonDocument<2048> doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.print("Error parseando JSON: ");
    Serial.println(error.c_str());
    return;
  }

  const char* type = doc["type"];

  if (strcmp(type, "config") == 0) {
    handleConfig(doc);
  } else if (strcmp(type, "command") == 0) {
    handleCommand(doc);
  }
}

void handleConfig(JsonDocument& doc) {
  deviceId = doc["device_id"];
  isRegistered = true;

  Serial.print("✅ Registrado como dispositivo ID: ");
  Serial.println(deviceId);

  // Configurar GPIOs
  JsonArray gpioArray = doc["gpio"].as<JsonArray>();
  gpioCount = 0;
  for (JsonObject gpio : gpioArray) {
    if (gpioCount >= MAX_GPIOS) break;

    gpios[gpioCount].pin = gpio["pin"];
    gpios[gpioCount].name = gpio["name"].as<String>();
    gpios[gpioCount].value = gpio["value"];
    gpios[gpioCount].active = gpio["active"];
    gpios[gpioCount].loopEnabled = gpio["loop_enabled"];
    gpios[gpioCount].loopInterval = gpio["loop_interval"];
    gpios[gpioCount].lastLoop = 0;

    String mode = gpio["mode"].as<String>();
    if (mode == "OUTPUT") {
      gpios[gpioCount].mode = MODE_OUTPUT;
      pinMode(gpios[gpioCount].pin, OUTPUT);
      digitalWrite(gpios[gpioCount].pin, gpios[gpioCount].value);
    } else if (mode == "INPUT") {
      gpios[gpioCount].mode = MODE_INPUT;
      pinMode(gpios[gpioCount].pin, INPUT);
    } else if (mode == "INPUT_PULLUP") {
      gpios[gpioCount].mode = MODE_INPUT_PULLUP;
      pinMode(gpios[gpioCount].pin, INPUT_PULLUP);
    } else if (mode == "PWM") {
      gpios[gpioCount].mode = MODE_PWM;
      ledcAttach(gpios[gpioCount].pin, 5000, 8);
      ledcWrite(gpios[gpioCount].pin, gpios[gpioCount].value);
    }

    gpioCount++;
  }

  Serial.print("GPIOs configurados: ");
  Serial.println(gpioCount);

  // Configurar DHT
  JsonArray dhtArray = doc["dht"].as<JsonArray>();

  // Limpiar sensores anteriores
  for (int i = 0; i < dhtCount; i++) {
    if (dhtSensors[i].sensor) {
      delete dhtSensors[i].sensor;
    }
  }
  dhtCount = 0;

  for (JsonObject dht : dhtArray) {
    if (dhtCount >= MAX_DHT_SENSORS) break;

    int pin = dht["pin"];
    String type = dht["sensor_type"].as<String>();

    dhtSensors[dhtCount].pin = pin;
    dhtSensors[dhtCount].name = dht["name"].as<String>();
    dhtSensors[dhtCount].type = type;
    dhtSensors[dhtCount].active = dht["active"];
    dhtSensors[dhtCount].readInterval = dht["read_interval"] | 5000;
    dhtSensors[dhtCount].lastRead = 0;
    dhtSensors[dhtCount].temperature = 0;
    dhtSensors[dhtCount].humidity = 0;

    uint8_t dhtType = (type == "DHT22") ? DHT22 : DHT11;
    dhtSensors[dhtCount].sensor = new DHT(pin, dhtType);
    dhtSensors[dhtCount].sensor->begin();

    dhtCount++;
  }

  Serial.print("Sensores DHT configurados: ");
  Serial.println(dhtCount);

  // Verificar OTA pendiente
  if (!doc["ota"].isNull()) {
    JsonObject ota = doc["ota"];
    startOTA(
      ota["id"],
      ota["filename"].as<String>(),
      ota["filesize"],
      ota["checksum"].as<String>()
    );
  }
}

void handleCommand(JsonDocument& doc) {
  const char* action = doc["action"];

  if (strcmp(action, "set_gpio") == 0) {
    int pin = doc["pin"];
    int value = doc["value"];

    for (int i = 0; i < gpioCount; i++) {
      if (gpios[i].pin == pin) {
        gpios[i].value = value;

        if (gpios[i].mode == MODE_OUTPUT) {
          digitalWrite(pin, value);
        } else if (gpios[i].mode == MODE_PWM) {
          ledcWrite(pin, value);
        }

        Serial.print("GPIO ");
        Serial.print(pin);
        Serial.print(" = ");
        Serial.println(value);
        break;
      }
    }
  }
  else if (strcmp(action, "update_gpio") == 0) {
    // Reconfigurar todos los GPIOs
    handleConfig(doc);
  }
  else if (strcmp(action, "update_dht") == 0) {
    handleConfig(doc);
  }
  else if (strcmp(action, "reboot") == 0) {
    Serial.println("🔄 Reiniciando...");
    delay(1000);
    ESP.restart();
  }
  else if (strcmp(action, "ota_update") == 0) {
    startOTA(
      doc["ota_id"],
      doc["filename"].as<String>(),
      doc["filesize"],
      doc["checksum"].as<String>()
    );
  }
}

// ============================================
// ENVÍO DE DATOS
// ============================================

void sendSensorData() {
  StaticJsonDocument<1024> doc;
  doc["type"] = "data";
  doc["mac_address"] = deviceMac;

  JsonObject payload = doc.createNestedObject("payload");

  // Agregar datos DHT (primer sensor como principal)
  if (dhtCount > 0 && dhtSensors[0].temperature != 0) {
    payload["temperature"] = dhtSensors[0].temperature;
    payload["humidity"] = dhtSensors[0].humidity;
  }

  // Agregar estados GPIO
  JsonArray gpioArray = payload.createNestedArray("gpio");
  for (int i = 0; i < gpioCount; i++) {
    if (gpios[i].active) {
      JsonObject g = gpioArray.createNestedObject();
      g["pin"] = gpios[i].pin;

      if (gpios[i].mode == MODE_INPUT || gpios[i].mode == MODE_INPUT_PULLUP) {
        gpios[i].value = digitalRead(gpios[i].pin);
      }
      g["value"] = gpios[i].value;
    }
  }

  String json;
  serializeJson(doc, json);
  webSocket.sendTXT(json);
}

// ============================================
// SENSORES DHT
// ============================================

void readDHTSensors() {
  for (int i = 0; i < dhtCount; i++) {
    if (!dhtSensors[i].active || !dhtSensors[i].sensor) continue;

    if (millis() - dhtSensors[i].lastRead >= dhtSensors[i].readInterval) {
      dhtSensors[i].lastRead = millis();

      float h = dhtSensors[i].sensor->readHumidity();
      float t = dhtSensors[i].sensor->readTemperature();

      if (!isnan(h) && !isnan(t)) {
        dhtSensors[i].temperature = t;
        dhtSensors[i].humidity = h;
      }
    }
  }
}

// ============================================
// GPIO LOOPS
// ============================================

void processGpioLoops() {
  for (int i = 0; i < gpioCount; i++) {
    if (!gpios[i].loopEnabled || gpios[i].mode != MODE_OUTPUT) continue;

    if (millis() - gpios[i].lastLoop >= gpios[i].loopInterval) {
      gpios[i].lastLoop = millis();
      gpios[i].value = !gpios[i].value;
      digitalWrite(gpios[i].pin, gpios[i].value);
    }
  }
}

// ============================================
// OTA
// ============================================

void startOTA(int id, String filename, int filesize, String checksum) {
  Serial.println("\n========================================");
  Serial.println("📦 Iniciando actualización OTA");
  Serial.print("Versión: ");
  Serial.println(filename);
  Serial.print("Tamaño: ");
  Serial.println(filesize);
  Serial.println("========================================\n");

  otaInProgress = true;
  otaId = id;
  otaFilename = filename;
  otaFilesize = filesize;
  otaChecksum = checksum;

  // Descargar firmware
  String url = "http://" + BACKEND_HOST + ":" + String(BACKEND_PORT) + "/api/ota/download/" + filename;

  HTTPClient http;
  http.begin(url);
  http.setTimeout(30000);

  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();

    if (contentLength > 0) {
      bool canBegin = Update.begin(contentLength);

      if (canBegin) {
        Serial.println("Descargando firmware...");

        WiFiClient* stream = http.getStreamPtr();
        size_t written = Update.writeStream(*stream);

        if (written == contentLength) {
          Serial.println("Firmware descargado correctamente");
        } else {
          Serial.print("Error: solo se escribieron ");
          Serial.print(written);
          Serial.print(" de ");
          Serial.println(contentLength);
        }

        if (Update.end()) {
          if (Update.isFinished()) {
            Serial.println("✅ OTA completado exitosamente");
            reportOTAStatus("success", "");

            Serial.println("Reiniciando...");
            delay(1000);
            ESP.restart();
          } else {
            reportOTAStatus("failed", "Update not finished");
          }
        } else {
          String error = String(Update.getError());
          Serial.print("❌ Error OTA: ");
          Serial.println(error);
          reportOTAStatus("failed", error);
        }
      } else {
        reportOTAStatus("failed", "Not enough space");
      }
    }
  } else {
    String error = "HTTP error: " + String(httpCode);
    Serial.println(error);
    reportOTAStatus("failed", error);
  }

  http.end();
  otaInProgress = false;
}

void reportOTAStatus(const char* status, String error) {
  StaticJsonDocument<256> doc;
  doc["type"] = "ota_status";
  doc["mac_address"] = deviceMac;
  doc["ota_id"] = otaId;
  doc["status"] = status;
  if (error.length() > 0) {
    doc["error"] = error;
  }

  String json;
  serializeJson(doc, json);
  webSocket.sendTXT(json);
}

// ============================================
// CONFIGURACIÓN PERSISTENTE
// ============================================

void loadConfig() {
  preferences.begin("minios", true);

  WIFI_SSID = preferences.getString("wifi_ssid", "");
  WIFI_PASS = preferences.getString("wifi_pass", "");
  BACKEND_HOST = preferences.getString("backend_host", "");
  BACKEND_PORT = preferences.getInt("backend_port", 3000);

  preferences.end();

  Serial.println("Configuración cargada");
}

void saveConfig() {
  preferences.begin("minios", false);

  preferences.putString("wifi_ssid", WIFI_SSID);
  preferences.putString("wifi_pass", WIFI_PASS);
  preferences.putString("backend_host", BACKEND_HOST);
  preferences.putInt("backend_port", BACKEND_PORT);

  preferences.end();

  Serial.println("✅ Configuración guardada");
}

// ============================================
// COMANDOS SERIAL
// ============================================

void handleSerial() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.length() == 0) return;

  // Parsear comando
  int space1 = input.indexOf(' ');
  String cmd = (space1 > 0) ? input.substring(0, space1) : input;
  String args = (space1 > 0) ? input.substring(space1 + 1) : "";

  if (cmd == "wifi") {
    int space2 = args.indexOf(' ');
    if (space2 > 0) {
      WIFI_SSID = args.substring(0, space2);
      WIFI_PASS = args.substring(space2 + 1);
      saveConfig();
      connectWiFi();
    } else {
      Serial.println("Uso: wifi <ssid> <password>");
    }
  }
  else if (cmd == "server") {
    int space2 = args.indexOf(' ');
    if (space2 > 0) {
      BACKEND_HOST = args.substring(0, space2);
      BACKEND_PORT = args.substring(space2 + 1).toInt();
      saveConfig();

      if (WiFi.status() == WL_CONNECTED) {
        connectWebSocket();
      }
    } else {
      Serial.println("Uso: server <host> <port>");
    }
  }
  else if (cmd == "status") {
    Serial.println("\n--- Estado ---");
    Serial.print("MAC: ");
    Serial.println(deviceMac);
    Serial.print("WiFi: ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "Conectado" : "Desconectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Backend: ");
    Serial.print(BACKEND_HOST);
    Serial.print(":");
    Serial.println(BACKEND_PORT);
    Serial.print("Registrado: ");
    Serial.println(isRegistered ? "Sí" : "No");
    Serial.print("GPIOs: ");
    Serial.println(gpioCount);
    Serial.print("DHT: ");
    Serial.println(dhtCount);
    Serial.println("--------------\n");
  }
  else if (cmd == "reboot") {
    Serial.println("Reiniciando...");
    delay(500);
    ESP.restart();
  }
  else {
    Serial.println("Comando no reconocido");
  }
}
