/*
 * MiniOS WiFi - Sistema Operativo Minimalista con WiFi para ESP32-S3
 * Versión con conectividad WiFi configurable
 * Compilar directamente en Arduino IDE
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// ============================================
// CONFIGURACIÓN
// ============================================
#define MAX_TASKS 8
#define SERIAL_BAUD 115200
#define AP_SSID "MiniOS-ESP32"
#define AP_PASS "12345678"      // Cambiar antes de desplegar: es publica en el repo
#define WEB_PORT 80

// Reintento de conexion WiFi cuando se pierde la red (ms)
#define WIFI_RETRY_INTERVAL 30000

// LED de la placa.
// En el core 3.x, LED_BUILTIN de las variantes C3/S3 no es un GPIO: vale
// SOC_GPIO_PIN_COUNT + 8 y digitalWrite() lo desvia al driver RMT del LED
// direccionable, sin tocar nunca el nivel del pin. En placas con un LED normal
// (C3 SuperMini) eso deja el pin como lo dejo pinMode(), o sea a nivel bajo,
// y con un LED activo a nivel bajo se queda encendido para siempre.
#if CONFIG_IDF_TARGET_ESP32C3
  #define LED_PIN        8   // C3 SuperMini: LED azul
  #define LED_ACTIVE_LOW 1   // se enciende con nivel bajo
#elif defined(LED_BUILTIN)
  #define LED_PIN        LED_BUILTIN
  #define LED_ACTIVE_LOW 0
#else
  #define LED_PIN        2   // ESP32 clasico: LED en GPIO 2 en la mayoria de devkits
  #define LED_ACTIVE_LOW 0
#endif

// Autenticacion de la interfaz web.
// Con la contrasena vacia no se pide nada (comportamiento de siempre).
// Al ponerle valor, todas las rutas piden usuario/contrasena por HTTP Basic.
#define WEB_USER "admin"
#define WEB_PASSWORD ""

// ============================================
// SISTEMA DE TAREAS SIMPLE
// ============================================
struct Task {
    void (*function)();      // Función a ejecutar
    unsigned long interval;  // Intervalo en ms (0 = una vez)
    unsigned long lastRun;   // Última ejecución
    bool active;             // Tarea activa
    const char* name;        // Nombre de la tarea
    int priority;            // Prioridad (0-3)
};

Task tasks[MAX_TASKS];
int taskCount = 0;
unsigned long systemTime = 0;

// ============================================
// VARIABLES WIFI
// ============================================
WebServer server(WEB_PORT);
Preferences preferences;
String wifiSSID = "";
String wifiPassword = "";
bool wifiConnected = false;
bool apMode = false;
bool webServerStarted = false;   // los handlers solo se registran una vez
IPAddress localIP;

// ============================================
// SISTEMA DE CONTROL GPIO
// ============================================
#define MAX_GPIOS 10

// GPIOs clasificados por función según ESP32-S3
// Las tablas dependen del chip. Con una tabla de S3 en un C3 el sketch compila
// igual (son enteros), pero GPIO_ApplyConfig() acaba haciendo pinMode() sobre la
// flash SPI interna y el chip se cuelga antes de que el USB llegue a enumerar.

#if CONFIG_IDF_TARGET_ESP32C3

// --- ESP32-C3 (probado en C3 SuperMini) ---
// Solo existen GPIO 0..21. Reservados por hardware:
//   11..17 -> flash SPI interna (tocarlos cuelga el chip)
//   18, 19 -> USB D- / D+ (los usas y pierdes el puerto serie)
//   20, 21 -> UART0
// En la SuperMini, además, 11..19 ni siquiera salen al conector.

// ADC1: canales 0..4. (GPIO 5 es ADC2 y no se puede usar con WiFi activo)
const int ANALOG_GPIOS[] = {0, 1, 2, 3, 4};
const int ANALOG_GPIOS_COUNT = 5;

// Uso general. GPIO 9 es el botón BOOT y GPIO 8 lleva el LED: van en I2C_GPIOS
// para que no se puedan reasignar por la interfaz web.
const int DIGITAL_GPIOS[] = {5, 6, 7, 10};
const int DIGITAL_GPIOS_COUNT = 4;

const int I2C_GPIOS[] = {8, 9};  // 8=SDA (y LED), 9=SCL (y BOOT)
const int I2C_GPIOS_COUNT = 2;

// El C3 no tiene un bus SPI externo dedicado en esta placa
const int SPI_GPIOS[] = {-1};
const int SPI_GPIOS_COUNT = 0;

#else  // ESP32-S3 y equivalentes

#if !CONFIG_IDF_TARGET_ESP32S3
#warning "Mapa de pines no verificado para este chip: se usa el del ESP32-S3. Revisa las tablas antes de grabar."
#endif

// GPIOs Analógicas (ADC)
const int ANALOG_GPIOS[] = {1, 2, 4, 5, 6, 7};
const int ANALOG_GPIOS_COUNT = 6;

// GPIOs Digitales (uso general)
const int DIGITAL_GPIOS[] = {0, 3, 14, 15, 16, 17, 18, 19, 20, 21, 36, 37, 38, 39, 40, 41, 42, 45, 46};
const int DIGITAL_GPIOS_COUNT = 19;

// GPIOs I2C (por defecto)
const int I2C_GPIOS[] = {8, 9};  // 8=SDA, 9=SCL
const int I2C_GPIOS_COUNT = 2;

// GPIOs SPI (usar con precaución)
const int SPI_GPIOS[] = {10, 11, 12, 13};
const int SPI_GPIOS_COUNT = 4;

#endif

// Modos de GPIO
enum GPIOMode {
    GPIO_DISABLED = 0,
    GPIO_OUTPUT = 1,
    GPIO_INPUT = 2,
    GPIO_INPUT_PULLUP = 3,
    GPIO_PWM = 4
};

// Estructura de configuración de GPIO
struct GPIOConfig {
    int pin;                 // Número de pin GPIO
    GPIOMode mode;          // Modo del pin
    int value;              // Valor actual (0/1 para digital, 0-4095 para analog, 0-255 para PWM)
    int pwmChannel;         // Canal PWM (0-15 para ESP32)
    String name;            // Nombre descriptivo
    bool active;            // Pin configurado
    bool loopEnabled;       // Parpadeo automático activado (solo OUTPUT)
    unsigned long loopInterval;  // Intervalo de parpadeo en ms
    unsigned long lastToggle;    // Última vez que se cambió el estado

    // Para lecturas analógicas con conversión
    bool hasFormula;        // Tiene fórmula de conversión
    float multiplier;       // Factor de multiplicación (ej: 3.3/4095)
    float offset;           // Offset a sumar después de multiplicar
    String unit;            // Unidad de medida (mA, V, °C, etc)
    String formulaType;     // Tipo de sensor (4-20mA, 0-10V, custom, etc)
    float convertedValue;   // Valor después de aplicar la fórmula
};

GPIOConfig gpioConfigs[MAX_GPIOS];
int gpioCount = 0;

// ============================================
// SISTEMA DE SENSORES DHT
// ============================================
#define MAX_DHT_SENSORS 4

// Estructura de configuración de sensor DHT
struct DHTConfig {
    int pin;                 // Pin GPIO donde está conectado
    DHT* sensor;            // Puntero al objeto DHT
    String name;            // Nombre descriptivo
    float temperature;      // Última temperatura leída
    float humidity;         // Última humedad leída
    bool active;            // Sensor configurado
    unsigned long lastRead; // Última lectura
    bool lastReadOk;        // Última lectura exitosa
};

DHTConfig dhtSensors[MAX_DHT_SENSORS];
int dhtCount = 0;

// ============================================
// SISTEMA DE PANTALLA TFT (SPI por software)
// ============================================
// Pines para ST7735 SPI (128x160)
#if CONFIG_IDF_TARGET_ESP32C3
// En el C3 los pines 11 y 12 son la flash interna: inicializar la pantalla ahi
// mata el chip. Estos son los que quedan libres; ajustar al cableado real.
// Ojo: en una C3 SuperMini la pantalla se lleva casi todas las E/S disponibles.
#define TFT_CS    7   // Chip select
#define TFT_RST   10  // Reset
#define TFT_DC    6   // Data/Command
#define TFT_MOSI  5   // SPI MOSI
#define TFT_SCLK  4   // SPI Clock
#else
#define TFT_CS    10  // Chip select
#define TFT_RST   9   // Reset
#define TFT_DC    8   // Data/Command
#define TFT_MOSI  11  // SPI MOSI
#define TFT_SCLK  12  // SPI Clock
#endif

// Colores comunes
#define TFT_BLACK   0x0000
#define TFT_BLUE    0x001F
#define TFT_RED     0xF800
#define TFT_GREEN   0x07E0
#define TFT_CYAN    0x07FF
#define TFT_MAGENTA 0xF81F
#define TFT_YELLOW  0xFFE0
#define TFT_WHITE   0xFFFF
#define TFT_ORANGE  0xFD20
#define TFT_GREENYELLOW 0xAFE5

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

bool tftEnabled = false;
bool tftInitialized = false;
uint8_t tftDisplayMode = 0;  // 0=Apagado, 1=Info Sistema, 2=Sensores, 3=GPIO, 4=Personalizado
bool tftFirstDraw = true;     // Primera vez dibujando la pantalla

// Variables para almacenar valores anteriores (evitar redibujo completo)
struct TFT_LastValues {
    unsigned long freeHeap;
    float temperature;
    unsigned long uptime;
    String wifiSSID;
    String ipAddress;
    int rssi;
    bool wifiStatus;  // 0=desconectado, 1=conectado, 2=AP
    float dhtTemp[MAX_DHT_SENSORS];
    float dhtHum[MAX_DHT_SENSORS];
    bool dhtStatus[MAX_DHT_SENSORS];
    float analogValues[MAX_GPIOS];
};

TFT_LastValues tftLast;

// ============================================
// UTILIDADES
// ============================================

// Enciende o apaga el LED de la placa respetando su polaridad
void ledWrite(bool on) {
#if LED_ACTIVE_LOW
    digitalWrite(LED_PIN, on ? LOW : HIGH);
#else
    digitalWrite(LED_PIN, on ? HIGH : LOW);
#endif
}


// Escapa texto para meterlo dentro de una cadena JSON.
// Sin esto, un nombre con comillas rompia el JSON entero y la interfaz web
// se quedaba en "Cargando..." para siempre.
String jsonEscape(const String &text) {
    String out;
    out.reserve(text.length() + 8);

    for(size_t i = 0; i < text.length(); i++) {
        char c = text.charAt(i);
        switch(c) {
            case '\"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if((uint8_t)c < 0x20) {
                    char buf[7];
                    snprintf(buf, sizeof(buf), "\\u%04x", (uint8_t)c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }

    return out;
}

// Escapa texto para insertarlo en HTML o en un atributo delimitado por comillas
// dobles. No toca la comilla simple: se usa junto a jsEscape().
String htmlEscape(const String &text) {
    String out;
    out.reserve(text.length() + 8);

    for(size_t i = 0; i < text.length(); i++) {
        char c = text.charAt(i);
        switch(c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '\"': out += "&quot;"; break;
            default: out += c;
        }
    }

    return out;
}

// Escapa texto para una cadena JavaScript delimitada por comillas simples.
// Se aplica ANTES de htmlEscape cuando el JS va dentro de un atributo.
String jsEscape(const String &text) {
    String out;
    out.reserve(text.length() + 8);

    for(size_t i = 0; i < text.length(); i++) {
        char c = text.charAt(i);
        if(c == '\\' || c == '\'') out += '\\';
        out += c;
    }

    return out;
}

// ============================================
// NÚCLEO DEL OS
// ============================================

// Inicializar sistema
void OS_Init() {
    Serial.begin(SERIAL_BAUD);
    delay(100);
    
    // Limpiar tareas
    for(int i = 0; i < MAX_TASKS; i++) {
        tasks[i].active = false;
    }
    
    Serial.println("\n╔════════════════════════════╗");
    Serial.println("║   MiniOS ESP32-S3 + WiFi   ║");
    Serial.println("║      Sistema iniciado      ║");
    Serial.println("╚════════════════════════════╝");
    Serial.printf("RAM libre: %lu bytes\n", (unsigned long)ESP.getFreeHeap());
    Serial.printf("CPU: %lu MHz\n", (unsigned long)ESP.getCpuFreqMHz());

    // Inicializar GPIO
    GPIO_Init();

    // Inicializar DHT
    DHT_Init();

    // Inicializar pantalla TFT (opcional)
    // TFT_Init();  // Descomentar si se tiene pantalla conectada

    // Inicializar WiFi
    WiFi_Init();

    Serial.println("\n📡 Comandos: help, ps, free, temp, wifi, ip, led, reboot");
    Serial.println("💡 Use la interfaz web para configurar GPIO y sensores DHT\n");
}

// Añadir tarea
int OS_AddTask(void (*func)(), unsigned long interval, const char* name, int priority = 1) {
    if(taskCount >= MAX_TASKS) {
        Serial.println("[ERROR] Máximo de tareas alcanzado");
        return -1;
    }
    
    for(int i = 0; i < MAX_TASKS; i++) {
        if(!tasks[i].active) {
            tasks[i].function = func;
            tasks[i].interval = interval;
            tasks[i].lastRun = 0;
            tasks[i].active = true;
            tasks[i].name = name;
            tasks[i].priority = priority;
            taskCount++;
            
            Serial.printf("[OS] Tarea '%s' creada (Pri:%d)\n", name, priority);
            return i;
        }
    }
    return -1;
}

// Eliminar tarea
void OS_RemoveTask(int id) {
    if(id >= 0 && id < MAX_TASKS && tasks[id].active) {
        Serial.printf("[OS] Tarea '%s' eliminada\n", tasks[id].name);
        tasks[id].active = false;
        taskCount--;
    }
}

// Scheduler simple con prioridades
void OS_Run() {
    systemTime = millis();
    
    // Ejecutar tareas por prioridad (3 = alta, 0 = baja)
    for(int priority = 3; priority >= 0; priority--) {
        for(int i = 0; i < MAX_TASKS; i++) {
            if(tasks[i].active && tasks[i].priority == priority) {
                // Verificar si toca ejecutar
                if(tasks[i].interval == 0) {
                    // Ejecutar una sola vez
                    tasks[i].function();
                    tasks[i].active = false;
                    taskCount--;
                } else if(systemTime - tasks[i].lastRun >= tasks[i].interval) {
                    // Ejecutar periódicamente
                    tasks[i].function();
                    tasks[i].lastRun = systemTime;
                }
            }
        }
    }
}

// ============================================
// FUNCIONES DE CONTROL GPIO
// ============================================

// Verificar si un GPIO existe en una lista
bool GPIO_InList(int pin, const int* list, int count) {
    for(int i = 0; i < count; i++) {
        if(list[i] == pin) return true;
    }
    return false;
}

// Verificar si un GPIO es válido (existe en alguna categoría)
bool GPIO_IsValid(int pin) {
    return GPIO_InList(pin, ANALOG_GPIOS, ANALOG_GPIOS_COUNT) ||
           GPIO_InList(pin, DIGITAL_GPIOS, DIGITAL_GPIOS_COUNT) ||
           GPIO_InList(pin, I2C_GPIOS, I2C_GPIOS_COUNT) ||
           GPIO_InList(pin, SPI_GPIOS, SPI_GPIOS_COUNT);
}

// Verificar si un GPIO es apropiado para un modo específico
bool GPIO_IsAppropriate(int pin, GPIOMode mode) {
    // GPIOs analógicas solo para INPUT
    // Los pines SPI e I2C no entran en ningun caso del switch: quedan excluidos
    bool isAnalog = GPIO_InList(pin, ANALOG_GPIOS, ANALOG_GPIOS_COUNT);
    bool isDigital = GPIO_InList(pin, DIGITAL_GPIOS, DIGITAL_GPIOS_COUNT);

    switch(mode) {
        case GPIO_OUTPUT:
        case GPIO_PWM:
            // OUTPUT y PWM: solo digitales (no analógicas, no SPI por seguridad)
            return isDigital;

        case GPIO_INPUT:
        case GPIO_INPUT_PULLUP:
            // INPUT: analógicas o digitales (no SPI por seguridad)
            return isAnalog || isDigital;

        default:
            return false;
    }
}

// Obtener lista de GPIOs actualmente en uso
// maxCount evita desbordar el buffer del llamador si algún día crecen los MAX_*
void GPIO_GetInUse(int* inUseList, int* count, int maxCount) {
    *count = 0;

    // Pines de pantalla TFT (si está inicializada)
    if(tftInitialized) {
        const int tftPins[] = {TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK};
        for(unsigned int i = 0; i < sizeof(tftPins) / sizeof(tftPins[0]); i++) {
            if(*count >= maxCount) return;
            inUseList[(*count)++] = tftPins[i];
        }
    }

    // Pines configurados como GPIO
    for(int i = 0; i < MAX_GPIOS; i++) {
        if(gpioConfigs[i].active) {
            if(*count >= maxCount) return;
            inUseList[(*count)++] = gpioConfigs[i].pin;
        }
    }

    // Pines usados por sensores DHT
    for(int i = 0; i < MAX_DHT_SENSORS; i++) {
        if(dhtSensors[i].active) {
            if(*count >= maxCount) return;
            inUseList[(*count)++] = dhtSensors[i].pin;
        }
    }
}

// Verificar si un pin está en uso
bool GPIO_IsInUse(int pin) {
    int inUseList[50];
    int count = 0;
    GPIO_GetInUse(inUseList, &count, 50);

    for(int i = 0; i < count; i++) {
        if(inUseList[i] == pin) return true;
    }
    return false;
}

// Inicializar sistema GPIO
void GPIO_Init() {
    // Limpiar configuraciones
    for(int i = 0; i < MAX_GPIOS; i++) {
        gpioConfigs[i].active = false;
        gpioConfigs[i].pin = -1;
        gpioConfigs[i].mode = GPIO_DISABLED;
        gpioConfigs[i].value = 0;
        gpioConfigs[i].pwmChannel = -1;
        gpioConfigs[i].name = "";
        gpioConfigs[i].loopEnabled = false;
        gpioConfigs[i].loopInterval = 500;
        gpioConfigs[i].lastToggle = 0;
        gpioConfigs[i].hasFormula = false;
        gpioConfigs[i].multiplier = 1.0;
        gpioConfigs[i].offset = 0.0;
        gpioConfigs[i].unit = "";
        gpioConfigs[i].formulaType = "";
        gpioConfigs[i].convertedValue = 0.0;
    }
    gpioCount = 0;

    // Cargar configuración desde NVS
    GPIO_LoadConfig();

    Serial.printf("[GPIO] Sistema inicializado (%d pines configurados)\n", gpioCount);
}

// Guardar configuración en NVS
// Los pines activos se guardan en ranuras consecutivas (g0_, g1_, ...) aunque
// esten dispersos en el array: antes se guardaba con el indice del array pero
// se leia hasta "count", asi que al borrar un pin se perdian los demas.
void GPIO_SaveConfig() {
    preferences.begin("gpio", false);

    int saved = 0;

    for(int i = 0; i < MAX_GPIOS; i++) {
        if(!gpioConfigs[i].active) continue;

        String prefix = "g" + String(saved) + "_";
        preferences.putInt((prefix + "pin").c_str(), gpioConfigs[i].pin);
        preferences.putInt((prefix + "mode").c_str(), (int)gpioConfigs[i].mode);
        preferences.putInt((prefix + "val").c_str(), gpioConfigs[i].value);
        preferences.putString((prefix + "name").c_str(), gpioConfigs[i].name);
        preferences.putBool((prefix + "loop").c_str(), gpioConfigs[i].loopEnabled);
        preferences.putUInt((prefix + "intv").c_str(), gpioConfigs[i].loopInterval);

        // Guardar parámetros de fórmula
        preferences.putBool((prefix + "hasf").c_str(), gpioConfigs[i].hasFormula);
        if(gpioConfigs[i].hasFormula) {
            preferences.putFloat((prefix + "mult").c_str(), gpioConfigs[i].multiplier);
            preferences.putFloat((prefix + "offs").c_str(), gpioConfigs[i].offset);
            preferences.putString((prefix + "unit").c_str(), gpioConfigs[i].unit);
            preferences.putString((prefix + "ftyp").c_str(), gpioConfigs[i].formulaType);
        }

        saved++;
    }

    preferences.putInt("count", saved);
    preferences.end();

    Serial.printf("[GPIO] Configuración guardada (%d pines)\n", saved);
}

// Cargar configuración desde NVS
void GPIO_LoadConfig() {
    preferences.begin("gpio", true); // read-only
    int savedCount = preferences.getInt("count", 0);

    for(int i = 0; i < MAX_GPIOS && i < savedCount; i++) {
        String prefix = "g" + String(i) + "_";
        int pin = preferences.getInt((prefix + "pin").c_str(), -1);

        GPIOMode savedMode = (GPIOMode)preferences.getInt((prefix + "mode").c_str(), 0);

        // Se descarta lo que ya no encaje con el pin: configuraciones viejas
        // guardadas antes de que existieran estas comprobaciones
        if(pin >= 0 && GPIO_IsValid(pin) && GPIO_IsAppropriate(pin, savedMode)) {
            gpioConfigs[i].pin = pin;
            gpioConfigs[i].mode = savedMode;
            gpioConfigs[i].value = preferences.getInt((prefix + "val").c_str(), 0);
            gpioConfigs[i].name = preferences.getString((prefix + "name").c_str(), "GPIO" + String(pin));
            gpioConfigs[i].loopEnabled = preferences.getBool((prefix + "loop").c_str(), false);
            gpioConfigs[i].loopInterval = preferences.getUInt((prefix + "intv").c_str(), 500);
            gpioConfigs[i].lastToggle = 0;

            // Cargar parámetros de fórmula
            gpioConfigs[i].hasFormula = preferences.getBool((prefix + "hasf").c_str(), false);
            if(gpioConfigs[i].hasFormula) {
                gpioConfigs[i].multiplier = preferences.getFloat((prefix + "mult").c_str(), 1.0);
                gpioConfigs[i].offset = preferences.getFloat((prefix + "offs").c_str(), 0.0);
                gpioConfigs[i].unit = preferences.getString((prefix + "unit").c_str(), "");
                gpioConfigs[i].formulaType = preferences.getString((prefix + "ftyp").c_str(), "");
            }

            gpioConfigs[i].active = true;

            // Aplicar configuración al pin
            GPIO_ApplyConfig(i);
            gpioCount++;
        }
    }

    preferences.end();
}

// Aplicar configuración física a un pin
void GPIO_ApplyConfig(int index) {
    if(index < 0 || index >= MAX_GPIOS || !gpioConfigs[index].active) return;

    GPIOConfig &cfg = gpioConfigs[index];

    switch(cfg.mode) {
        case GPIO_OUTPUT:
            pinMode(cfg.pin, OUTPUT);
            digitalWrite(cfg.pin, cfg.value);
            break;

        case GPIO_INPUT:
            pinMode(cfg.pin, INPUT);
            break;

        case GPIO_INPUT_PULLUP:
            pinMode(cfg.pin, INPUT_PULLUP);
            break;

        case GPIO_PWM:
            // ESP32 Core 3.x usa ledcAttach directamente (sin canales)
            ledcAttach(cfg.pin, 5000, 8); // 5 KHz, 8 bits
            ledcWrite(cfg.pin, cfg.value);
            break;

        default:
            break;
    }
}

// Configurar un GPIO
int GPIO_Configure(int pin, GPIOMode mode, String name = "") {
    if(!GPIO_IsValid(pin)) {
        Serial.printf("[GPIO] ⚠️ GPIO %d no existe o no está disponible\n", pin);
        return -1;
    }

    if(!GPIO_IsAppropriate(pin, mode)) {
        Serial.printf("[GPIO] ⚠️ GPIO %d no es apropiado para este modo\n", pin);
        if(GPIO_InList(pin, ANALOG_GPIOS, ANALOG_GPIOS_COUNT)) {
            Serial.println("[GPIO] Este pin es analógico, solo puede usarse como INPUT");
        } else if(GPIO_InList(pin, SPI_GPIOS, SPI_GPIOS_COUNT)) {
            Serial.println("[GPIO] Este pin es SPI, se recomienda no usarlo");
        }
        return -1;
    }

    // Buscar si ya existe como GPIO configurado
    bool alreadyConfigured = false;
    for(int i = 0; i < MAX_GPIOS; i++) {
        if(gpioConfigs[i].active && gpioConfigs[i].pin == pin) {
            alreadyConfigured = true;
            break;
        }
    }

    // Si no está configurado como GPIO, verificar si está en uso por otra cosa
    if(!alreadyConfigured && GPIO_IsInUse(pin)) {
        Serial.printf("[GPIO] ⚠️ GPIO %d ya está en uso\n", pin);
        if(tftInitialized && (pin == TFT_CS || pin == TFT_DC || pin == TFT_RST || pin == TFT_MOSI || pin == TFT_SCLK)) {
            Serial.println("[GPIO] Este pin está siendo usado por la pantalla TFT");
        }
        for(int i = 0; i < MAX_DHT_SENSORS; i++) {
            if(dhtSensors[i].active && dhtSensors[i].pin == pin) {
                Serial.printf("[GPIO] Este pin está siendo usado por sensor DHT: %s\n", dhtSensors[i].name.c_str());
            }
        }
        return -1;
    }

    // Buscar si ya existe para actualizar
    for(int i = 0; i < MAX_GPIOS; i++) {
        if(gpioConfigs[i].active && gpioConfigs[i].pin == pin) {
            // Al salir de PWM hay que soltar el canal LEDC: si no, el pin sigue
            // atado al periférico y digitalWrite() no se comporta como se espera
            if(gpioConfigs[i].mode == GPIO_PWM && mode != GPIO_PWM) {
                ledcDetach(pin);
            }

            // La fórmula de conversión solo tiene sentido en entradas
            if(mode != GPIO_INPUT && mode != GPIO_INPUT_PULLUP) {
                gpioConfigs[i].hasFormula = false;
                gpioConfigs[i].multiplier = 1.0;
                gpioConfigs[i].offset = 0.0;
                gpioConfigs[i].unit = "";
                gpioConfigs[i].formulaType = "";
                gpioConfigs[i].convertedValue = 0.0;
            }

            // El valor anterior puede no tener sentido en el modo nuevo
            gpioConfigs[i].value = 0;

            gpioConfigs[i].mode = mode;
            if(name.length() > 0) gpioConfigs[i].name = name;
            GPIO_ApplyConfig(i);
            GPIO_SaveConfig();
            Serial.printf("[GPIO] Pin %d actualizado\n", pin);
            return i;
        }
    }

    // Crear nueva configuración
    if(gpioCount >= MAX_GPIOS) {
        Serial.println("[GPIO] Máximo de pines alcanzado");
        return -1;
    }

    for(int i = 0; i < MAX_GPIOS; i++) {
        if(!gpioConfigs[i].active) {
            gpioConfigs[i].pin = pin;
            gpioConfigs[i].mode = mode;
            gpioConfigs[i].value = 0;
            gpioConfigs[i].pwmChannel = -1;
            gpioConfigs[i].name = (name.length() > 0) ? name : "GPIO" + String(pin);
            gpioConfigs[i].active = true;
            gpioCount++;

            GPIO_ApplyConfig(i);
            GPIO_SaveConfig();

            Serial.printf("[GPIO] Pin %d configurado como %s\n", pin,
                mode == GPIO_OUTPUT ? "OUTPUT" :
                mode == GPIO_INPUT ? "INPUT" :
                mode == GPIO_INPUT_PULLUP ? "INPUT_PULLUP" :
                mode == GPIO_PWM ? "PWM" : "DISABLED");

            return i;
        }
    }

    return -1;
}

// Escribir valor digital a un GPIO
bool GPIO_DigitalWrite(int pin, int value) {
    for(int i = 0; i < MAX_GPIOS; i++) {
        if(gpioConfigs[i].active && gpioConfigs[i].pin == pin) {
            if(gpioConfigs[i].mode == GPIO_OUTPUT) {
                digitalWrite(pin, value);
                gpioConfigs[i].value = value;
                return true;
            }
        }
    }
    return false;
}

// Leer valor de un GPIO (digital o analógico)
int GPIO_DigitalRead(int pin) {
    for(int i = 0; i < MAX_GPIOS; i++) {
        if(gpioConfigs[i].active && gpioConfigs[i].pin == pin) {
            if(gpioConfigs[i].mode == GPIO_INPUT || gpioConfigs[i].mode == GPIO_INPUT_PULLUP) {
                // Verificar si es pin analógico
                bool isAnalog = GPIO_InList(pin, ANALOG_GPIOS, ANALOG_GPIOS_COUNT);

                if(isAnalog) {
                    // Leer valor analógico (0-4095 en ESP32)
                    int rawValue = analogRead(pin);
                    gpioConfigs[i].value = rawValue;

                    // Aplicar fórmula si está configurada
                    if(gpioConfigs[i].hasFormula) {
                        gpioConfigs[i].convertedValue = (rawValue * gpioConfigs[i].multiplier) + gpioConfigs[i].offset;
                    } else {
                        gpioConfigs[i].convertedValue = rawValue;
                    }
                } else {
                    // Leer valor digital
                    int value = digitalRead(pin);
                    gpioConfigs[i].value = value;
                    gpioConfigs[i].convertedValue = value;
                }

                return gpioConfigs[i].value;
            }
        }
    }
    return -1;
}

// Escribir valor PWM a un GPIO
bool GPIO_PWMWrite(int pin, int value) {
    if(value < 0) value = 0;
    if(value > 255) value = 255;

    for(int i = 0; i < MAX_GPIOS; i++) {
        if(gpioConfigs[i].active && gpioConfigs[i].pin == pin) {
            if(gpioConfigs[i].mode == GPIO_PWM) {
                ledcWrite(pin, value);  // ESP32 Core 3.x usa el pin directamente
                gpioConfigs[i].value = value;
                return true;
            }
        }
    }
    return false;
}

// Eliminar configuración de un GPIO
bool GPIO_Remove(int pin) {
    for(int i = 0; i < MAX_GPIOS; i++) {
        if(gpioConfigs[i].active && gpioConfigs[i].pin == pin) {
            // Limpiar PWM si aplica
            if(gpioConfigs[i].mode == GPIO_PWM) {
                ledcDetach(gpioConfigs[i].pin);  // ESP32 Core 3.x
            }

            gpioConfigs[i].active = false;
            gpioConfigs[i].pin = -1;
            gpioCount--;

            GPIO_SaveConfig();
            Serial.printf("[GPIO] Pin %d eliminado\n", pin);
            return true;
        }
    }
    return false;
}

// Obtener estado de todos los GPIOs (para API)
String GPIO_GetStatusJSON() {
    String json = "[";
    bool first = true;

    for(int i = 0; i < MAX_GPIOS; i++) {
        if(gpioConfigs[i].active) {
            if(!first) json += ",";
            first = false;

            // Leer valor actual si es entrada
            if(gpioConfigs[i].mode == GPIO_INPUT || gpioConfigs[i].mode == GPIO_INPUT_PULLUP) {
                GPIO_DigitalRead(gpioConfigs[i].pin);
            }

            json += "{";
            json += "\"pin\":" + String(gpioConfigs[i].pin) + ",";
            json += "\"name\":\"" + jsonEscape(gpioConfigs[i].name) + "\",";
            json += "\"mode\":" + String((int)gpioConfigs[i].mode) + ",";
            json += "\"value\":" + String(gpioConfigs[i].value) + ",";
            json += "\"loop\":" + String(gpioConfigs[i].loopEnabled ? "true" : "false") + ",";
            json += "\"interval\":" + String(gpioConfigs[i].loopInterval) + ",";

            // Información de fórmula
            json += "\"hasFormula\":" + String(gpioConfigs[i].hasFormula ? "true" : "false");
            if(gpioConfigs[i].hasFormula) {
                json += ",\"convertedValue\":" + String(gpioConfigs[i].convertedValue, 3);
                json += ",\"unit\":\"" + jsonEscape(gpioConfigs[i].unit) + "\"";
                json += ",\"formulaType\":\"" + jsonEscape(gpioConfigs[i].formulaType) + "\"";
            }

            json += "}";
        }
    }

    json += "]";
    return json;
}

// Activar/desactivar modo loop en un GPIO OUTPUT
bool GPIO_SetLoop(int pin, bool enabled, unsigned long interval = 500) {
    for(int i = 0; i < MAX_GPIOS; i++) {
        if(gpioConfigs[i].active && gpioConfigs[i].pin == pin) {
            if(gpioConfigs[i].mode == GPIO_OUTPUT) {
                gpioConfigs[i].loopEnabled = enabled;
                if(interval > 0) {
                    gpioConfigs[i].loopInterval = interval;
                }
                gpioConfigs[i].lastToggle = millis();
                GPIO_SaveConfig();
                return true;
            }
        }
    }
    return false;
}

// Tarea para manejar el parpadeo automático de GPIOs
void taskGPIOBlink() {
    unsigned long now = millis();

    for(int i = 0; i < MAX_GPIOS; i++) {
        if(gpioConfigs[i].active &&
           gpioConfigs[i].mode == GPIO_OUTPUT &&
           gpioConfigs[i].loopEnabled) {

            // Verificar si toca cambiar el estado
            if(now - gpioConfigs[i].lastToggle >= gpioConfigs[i].loopInterval) {
                // Toggle del pin
                gpioConfigs[i].value = !gpioConfigs[i].value;
                digitalWrite(gpioConfigs[i].pin, gpioConfigs[i].value);
                gpioConfigs[i].lastToggle = now;
            }
        }
    }
}

// ============================================
// FUNCIONES DE CONTROL DHT
// ============================================

// Inicializar sistema DHT
void DHT_Init() {
    // Limpiar configuraciones
    for(int i = 0; i < MAX_DHT_SENSORS; i++) {
        dhtSensors[i].active = false;
        dhtSensors[i].pin = -1;
        dhtSensors[i].sensor = nullptr;
        dhtSensors[i].name = "";
        dhtSensors[i].temperature = 0.0;
        dhtSensors[i].humidity = 0.0;
        dhtSensors[i].lastRead = 0;
        dhtSensors[i].lastReadOk = false;
    }
    dhtCount = 0;

    // Cargar configuración desde NVS
    DHT_LoadConfig();

    Serial.printf("[DHT] Sistema inicializado (%d sensores configurados)\n", dhtCount);
}

// Guardar configuración en NVS
// Mismo compactado de ranuras que en GPIO_SaveConfig
void DHT_SaveConfig() {
    preferences.begin("dht", false);

    int saved = 0;

    for(int i = 0; i < MAX_DHT_SENSORS; i++) {
        if(!dhtSensors[i].active) continue;

        String prefix = "d" + String(saved) + "_";
        preferences.putInt((prefix + "pin").c_str(), dhtSensors[i].pin);
        preferences.putString((prefix + "name").c_str(), dhtSensors[i].name);
        saved++;
    }

    preferences.putInt("count", saved);
    preferences.end();

    Serial.printf("[DHT] Configuración guardada (%d sensores)\n", saved);
}

// Cargar configuración desde NVS
void DHT_LoadConfig() {
    preferences.begin("dht", true); // read-only
    int savedCount = preferences.getInt("count", 0);

    for(int i = 0; i < MAX_DHT_SENSORS && i < savedCount; i++) {
        String prefix = "d" + String(i) + "_";
        int pin = preferences.getInt((prefix + "pin").c_str(), -1);

        if(pin >= 0 && GPIO_InList(pin, DIGITAL_GPIOS, DIGITAL_GPIOS_COUNT)) {
            dhtSensors[i].pin = pin;
            dhtSensors[i].name = preferences.getString((prefix + "name").c_str(), "DHT11-" + String(pin));
            dhtSensors[i].sensor = new DHT(pin, DHT11);
            dhtSensors[i].sensor->begin();
            dhtSensors[i].active = true;
            dhtSensors[i].lastReadOk = false;
            dhtCount++;
            Serial.printf("[DHT] Sensor cargado en GPIO %d\n", pin);
        }
    }

    preferences.end();
}

// Configurar un nuevo sensor DHT
int DHT_Configure(int pin, String name = "") {
    // Un DHT necesita un pin digital bidireccional. GPIO_IsSafe solo comprobaba
    // que el pin existiera, asi que por API se podia poner un DHT en un pin
    // I2C o SPI y chocar con la pantalla.
    if(!GPIO_InList(pin, DIGITAL_GPIOS, DIGITAL_GPIOS_COUNT)) {
        Serial.printf("[DHT] ⚠️ GPIO %d no es un pin digital válido para DHT\n", pin);
        return -1;
    }

    // Verificar si el pin ya está en uso como DHT
    for(int i = 0; i < MAX_DHT_SENSORS; i++) {
        if(dhtSensors[i].active && dhtSensors[i].pin == pin) {
            Serial.printf("[DHT] Pin %d ya está configurado como DHT\n", pin);
            return -1;
        }
    }

    // Verificar si está en uso por otra cosa
    if(GPIO_IsInUse(pin)) {
        Serial.printf("[DHT] ⚠️ GPIO %d ya está en uso\n", pin);
        if(tftInitialized && (pin == TFT_CS || pin == TFT_DC || pin == TFT_RST || pin == TFT_MOSI || pin == TFT_SCLK)) {
            Serial.println("[DHT] Este pin está siendo usado por la pantalla TFT");
        }
        for(int i = 0; i < MAX_GPIOS; i++) {
            if(gpioConfigs[i].active && gpioConfigs[i].pin == pin) {
                Serial.printf("[DHT] Este pin está siendo usado como GPIO: %s\n", gpioConfigs[i].name.c_str());
            }
        }
        return -1;
    }

    // Buscar espacio libre
    if(dhtCount >= MAX_DHT_SENSORS) {
        Serial.println("[DHT] Máximo de sensores alcanzado");
        return -1;
    }

    for(int i = 0; i < MAX_DHT_SENSORS; i++) {
        if(!dhtSensors[i].active) {
            dhtSensors[i].pin = pin;
            dhtSensors[i].name = (name.length() > 0) ? name : "DHT11-" + String(pin);
            dhtSensors[i].sensor = new DHT(pin, DHT11);
            dhtSensors[i].sensor->begin();
            dhtSensors[i].active = true;
            dhtSensors[i].temperature = 0.0;
            dhtSensors[i].humidity = 0.0;
            dhtSensors[i].lastRead = 0;
            dhtSensors[i].lastReadOk = false;
            dhtCount++;

            DHT_SaveConfig();

            Serial.printf("[DHT] Sensor DHT11 configurado en GPIO %d\n", pin);
            return i;
        }
    }

    return -1;
}

// Leer datos de un sensor DHT
bool DHT_Read(int index) {
    if(index < 0 || index >= MAX_DHT_SENSORS || !dhtSensors[index].active) {
        return false;
    }

    float h = dhtSensors[index].sensor->readHumidity();
    float t = dhtSensors[index].sensor->readTemperature();

    // Verificar si la lectura es válida
    if(isnan(h) || isnan(t)) {
        dhtSensors[index].lastReadOk = false;
        return false;
    }

    dhtSensors[index].humidity = h;
    dhtSensors[index].temperature = t;
    dhtSensors[index].lastRead = millis();
    dhtSensors[index].lastReadOk = true;

    return true;
}

// Eliminar un sensor DHT
bool DHT_Remove(int pin) {
    for(int i = 0; i < MAX_DHT_SENSORS; i++) {
        if(dhtSensors[i].active && dhtSensors[i].pin == pin) {
            // Liberar memoria del sensor
            if(dhtSensors[i].sensor != nullptr) {
                delete dhtSensors[i].sensor;
                dhtSensors[i].sensor = nullptr;
            }

            dhtSensors[i].active = false;
            dhtSensors[i].pin = -1;
            dhtCount--;

            DHT_SaveConfig();
            Serial.printf("[DHT] Sensor en GPIO %d eliminado\n", pin);
            return true;
        }
    }
    return false;
}

// Obtener estado de todos los sensores DHT (para API)
String DHT_GetStatusJSON() {
    String json = "[";
    bool first = true;

    for(int i = 0; i < MAX_DHT_SENSORS; i++) {
        if(dhtSensors[i].active) {
            if(!first) json += ",";
            first = false;

            json += "{";
            json += "\"pin\":" + String(dhtSensors[i].pin) + ",";
            json += "\"name\":\"" + jsonEscape(dhtSensors[i].name) + "\",";
            json += "\"temperature\":" + String(dhtSensors[i].temperature, 1) + ",";
            json += "\"humidity\":" + String(dhtSensors[i].humidity, 1) + ",";
            json += "\"lastRead\":" + String(dhtSensors[i].lastRead) + ",";
            json += "\"status\":\"" + String(dhtSensors[i].lastReadOk ? "ok" : "error") + "\"";
            json += "}";
        }
    }

    json += "]";
    return json;
}

// Tarea para leer sensores DHT periódicamente
void taskDHTRead() {
    for(int i = 0; i < MAX_DHT_SENSORS; i++) {
        if(dhtSensors[i].active) {
            DHT_Read(i);
        }
    }
}

// ============================================
// FUNCIONES DE PANTALLA TFT
// ============================================

// Inicializar pantalla TFT
bool TFT_Init() {
    if(tftInitialized) return true;

    // La comprobación de conflictos era unidireccional: GPIO y DHT miraban si la
    // pantalla usaba el pin, pero la pantalla arrancaba encima de lo que hubiera.
    const int tftPins[] = {TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK};
    for(unsigned int i = 0; i < sizeof(tftPins) / sizeof(tftPins[0]); i++) {
        if(GPIO_IsInUse(tftPins[i])) {
            Serial.printf("[TFT] ⚠️ GPIO %d ya está en uso, no se inicializa la pantalla\n", tftPins[i]);
            return false;
        }
    }

    Serial.println("[TFT] Inicializando pantalla...");

    tft.initR(INITR_BLACKTAB);  // Inicializar ST7735 (128x160)
    tft.setRotation(1);         // Horizontal
    tft.fillScreen(TFT_BLACK);

    tftInitialized = true;
    tftEnabled = true;

    Serial.println("[TFT] Pantalla inicializada");
    return true;
}

// Apagar pantalla
void TFT_Off() {
    if(!tftInitialized) return;
    tft.fillScreen(TFT_BLACK);
    tftEnabled = false;
    tftDisplayMode = 0;
    tftFirstDraw = true;  // Forzar redibujo completo al volver a encender
}

// Borrar y redibujar solo un valor en la pantalla
void TFT_UpdateValue(int y, int x, String newValue, uint16_t color = TFT_WHITE) {
    // Borrar área del valor anterior (desde x hasta final de línea)
    tft.fillRect(x, y, 160 - x, 8, TFT_BLACK);
    // Escribir nuevo valor
    tft.setCursor(x, y);
    tft.setTextColor(color);
    tft.setTextSize(1);
    tft.print(newValue);
}

// Mostrar información del sistema (actualización optimizada)
void TFT_ShowSystemInfo() {
    if(!tftInitialized || !tftEnabled) return;

    unsigned long freeHeap = ESP.getFreeHeap();
    float temperature = temperatureRead();
    unsigned long uptime = millis() / 1000;
    String ipAddr = wifiConnected ? WiFi.localIP().toString() : (apMode ? WiFi.softAPIP().toString() : "");
    String ssid = wifiConnected ? WiFi.SSID() : (apMode ? String(AP_SSID) : "");
    int rssi = wifiConnected ? WiFi.RSSI() : 0;

    // Si es primera vez, dibujar todo
    if(tftFirstDraw) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextSize(1);

        // Título
        tft.setCursor(0, 0);
        tft.setTextColor(TFT_CYAN);
        tft.setTextSize(2);
        tft.println("MiniOS");

        // Etiquetas estáticas
        tft.setTextSize(1);
        tft.setTextColor(TFT_WHITE);
        tft.setCursor(0, 20);
        tft.print("CPU: ");
        tft.print(ESP.getCpuFreqMHz());
        tft.println(" MHz");

        tft.setCursor(0, 30);
        tft.print("RAM: ");

        tft.setCursor(0, 40);
        tft.print("Temp: ");

        tft.setCursor(0, 50);
        tft.print("Uptime: ");

        tft.setCursor(0, 70);
        tft.print("WiFi: ");

        tft.setCursor(0, 80);
        tft.print("SSID: ");

        tft.setCursor(0, 90);
        tft.print("IP: ");

        if(wifiConnected) {
            tft.setCursor(0, 100);
            tft.print("RSSI: ");
        }

        // Guardar valores iniciales
        tftLast.freeHeap = freeHeap;
        tftLast.temperature = temperature;
        tftLast.uptime = uptime;
        tftLast.ipAddress = ipAddr;
        tftLast.wifiSSID = ssid;
        tftLast.rssi = rssi;
        tftLast.wifiStatus = wifiConnected ? 1 : (apMode ? 2 : 0);
    }

    // Actualizar solo valores que cambiaron

    // RAM (actualizar cada vez)
    if(freeHeap != tftLast.freeHeap || tftFirstDraw) {
        TFT_UpdateValue(30, 30, String(freeHeap / 1024) + " KB");
        tftLast.freeHeap = freeHeap;
    }

    // Temperatura (solo si cambió significativamente)
    if(abs(temperature - tftLast.temperature) > 0.5 || tftFirstDraw) {
        TFT_UpdateValue(40, 36, String(temperature, 1) + " C");
        tftLast.temperature = temperature;
    }

    // Uptime (actualizar cada vez)
    if(uptime != tftLast.uptime || tftFirstDraw) {
        TFT_UpdateValue(50, 48, String(uptime) + " s");
        tftLast.uptime = uptime;
    }

    // Estado WiFi
    int currentStatus = wifiConnected ? 1 : (apMode ? 2 : 0);
    if(currentStatus != tftLast.wifiStatus || tftFirstDraw) {
        if(wifiConnected) {
            TFT_UpdateValue(70, 36, "Conectado", TFT_GREEN);
        } else if(apMode) {
            TFT_UpdateValue(70, 36, "Modo AP   ", TFT_YELLOW);
        } else {
            TFT_UpdateValue(70, 36, "Desconectado", TFT_RED);
        }
        tftLast.wifiStatus = currentStatus;

        // Forzar actualización de SSID e IP
        tftLast.wifiSSID = "";
        tftLast.ipAddress = "";
    }

    // SSID
    if(ssid != tftLast.wifiSSID || tftFirstDraw) {
        TFT_UpdateValue(80, 30, ssid);
        tftLast.wifiSSID = ssid;
    }

    // IP
    if(ipAddr != tftLast.ipAddress || tftFirstDraw) {
        TFT_UpdateValue(90, 24, ipAddr);
        tftLast.ipAddress = ipAddr;
    }

    // RSSI (solo si conectado)
    if(wifiConnected && (rssi != tftLast.rssi || tftFirstDraw)) {
        TFT_UpdateValue(100, 36, String(rssi) + " dBm");
        tftLast.rssi = rssi;
    }
}

// Mostrar sensores DHT (actualización optimizada)
void TFT_ShowSensors() {
    if(!tftInitialized || !tftEnabled) return;

    // Si es primera vez, dibujar todo
    if(tftFirstDraw) {
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(0, 0);
        tft.setTextColor(TFT_CYAN);
        tft.setTextSize(2);
        tft.println("Sensores");
        tft.setTextSize(1);

        int y = 20;
        bool hasSensors = false;

        // Dibujar etiquetas de sensores DHT
        for(int i = 0; i < MAX_DHT_SENSORS; i++) {
            if(dhtSensors[i].active) {
                hasSensors = true;
                tft.setCursor(0, y);
                tft.setTextColor(TFT_YELLOW);
                tft.println(dhtSensors[i].name.c_str());
                y += 10;

                tft.setCursor(0, y);
                tft.setTextColor(TFT_WHITE);
                tft.print("Temp: ");
                y += 10;

                tft.setCursor(0, y);
                tft.print("Hum:  ");
                y += 20;

                // Guardar valores iniciales
                tftLast.dhtTemp[i] = dhtSensors[i].temperature;
                tftLast.dhtHum[i] = dhtSensors[i].humidity;
                tftLast.dhtStatus[i] = dhtSensors[i].lastReadOk;
            }
        }

        // Dibujar etiquetas de sensores analógicos
        for(int i = 0; i < MAX_GPIOS; i++) {
            if(gpioConfigs[i].active &&
               (gpioConfigs[i].mode == GPIO_INPUT || gpioConfigs[i].mode == GPIO_INPUT_PULLUP) &&
               gpioConfigs[i].hasFormula) {
                hasSensors = true;
                tft.setCursor(0, y);
                tft.setTextColor(TFT_YELLOW);
                tft.println(gpioConfigs[i].name.c_str());
                y += 10;

                tft.setCursor(0, y);
                tft.setTextColor(TFT_WHITE);
                y += 20;

                // Guardar valor inicial
                tftLast.analogValues[i] = gpioConfigs[i].convertedValue;
            }
        }

        if(!hasSensors) {
            tft.setCursor(0, 40);
            tft.setTextColor(TFT_RED);
            tft.println("No hay sensores");
            tft.println("configurados");
        }
    }

    // Actualizar solo valores que cambiaron
    int y = 20;

    // Actualizar sensores DHT
    for(int i = 0; i < MAX_DHT_SENSORS; i++) {
        if(dhtSensors[i].active) {
            y += 10;  // Nombre (no cambia)

            // Temperatura
            if(dhtSensors[i].lastReadOk) {
                if(abs(dhtSensors[i].temperature - tftLast.dhtTemp[i]) > 0.1 ||
                   tftLast.dhtStatus[i] != dhtSensors[i].lastReadOk || tftFirstDraw) {
                    TFT_UpdateValue(y, 36, String(dhtSensors[i].temperature, 1) + " C", TFT_ORANGE);
                    tftLast.dhtTemp[i] = dhtSensors[i].temperature;
                }
            }
            y += 10;

            // Humedad
            if(dhtSensors[i].lastReadOk) {
                if(abs(dhtSensors[i].humidity - tftLast.dhtHum[i]) > 0.1 ||
                   tftLast.dhtStatus[i] != dhtSensors[i].lastReadOk || tftFirstDraw) {
                    TFT_UpdateValue(y, 36, String(dhtSensors[i].humidity, 1) + " %", TFT_BLUE);
                    tftLast.dhtHum[i] = dhtSensors[i].humidity;
                }
            } else {
                // Error de lectura
                if(tftLast.dhtStatus[i] != dhtSensors[i].lastReadOk || tftFirstDraw) {
                    TFT_UpdateValue(y - 10, 36, "Error lectura", TFT_RED);
                }
            }

            tftLast.dhtStatus[i] = dhtSensors[i].lastReadOk;
            y += 20;
        }
    }

    // Actualizar sensores analógicos
    for(int i = 0; i < MAX_GPIOS; i++) {
        if(gpioConfigs[i].active &&
           (gpioConfigs[i].mode == GPIO_INPUT || gpioConfigs[i].mode == GPIO_INPUT_PULLUP) &&
           gpioConfigs[i].hasFormula) {
            y += 10;  // Nombre (no cambia)

            // Valor convertido
            if(abs(gpioConfigs[i].convertedValue - tftLast.analogValues[i]) > 0.01 || tftFirstDraw) {
                String valueStr = String(gpioConfigs[i].convertedValue, 2) + " " + gpioConfigs[i].unit;
                TFT_UpdateValue(y, 0, valueStr, TFT_GREENYELLOW);
                tftLast.analogValues[i] = gpioConfigs[i].convertedValue;
            }
            y += 20;
        }
    }
}

// Actualizar pantalla según modo
void TFT_Update() {
    if(!tftInitialized || !tftEnabled) return;

    switch(tftDisplayMode) {
        case 1:
            TFT_ShowSystemInfo();
            break;
        case 2:
            TFT_ShowSensors();
            break;
        default:
            break;
    }

    // Después del primer dibujado, desactivar flag
    if(tftFirstDraw) {
        tftFirstDraw = false;
    }
}

// Tarea periódica para actualizar pantalla
void taskTFTUpdate() {
    if(tftEnabled && tftDisplayMode > 0) {
        TFT_Update();
    }
}

// ============================================
// SISTEMA WIFI
// ============================================

// Guardar credenciales WiFi.
// Cada acceso abre y cierra su namespace: antes WiFi_Init dejaba "minios" abierto
// para siempre, el primer GPIO_SaveConfig fallaba al abrir "gpio" (Preferences
// devuelve false si ya hay uno abierto), escribia las claves de GPIO dentro de
// "minios" y al cerrar dejaba el objeto inutilizable, con lo que guardar el WiFi
// desde la web no hacia nada aunque dijera "Configuración guardada".
void WiFi_SaveCredentials(const String &ssid, const String &pass) {
    preferences.begin("minios", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", pass);
    preferences.end();
}

void WiFi_LoadCredentials() {
    preferences.begin("minios", true);
    wifiSSID = preferences.getString("ssid", "");
    wifiPassword = preferences.getString("password", "");
    preferences.end();
}

void WiFi_Init() {
    // Cargar configuración guardada
    WiFi_LoadCredentials();
    
    if(wifiSSID.length() > 0) {
        Serial.printf("[WiFi] Conectando a: %s\n", wifiSSID.c_str());
        WiFi.mode(WIFI_STA);
        WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
        
        // Intentar conectar por 10 segundos
        int attempts = 0;
        while(WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        
        if(WiFi.status() == WL_CONNECTED) {
            wifiConnected = true;
            localIP = WiFi.localIP();
            Serial.printf("\n[WiFi] ✅ Conectado! IP: %s\n", localIP.toString().c_str());
            WiFi_StartWebServer();
        } else {
            Serial.println("\n[WiFi] ❌ No se pudo conectar");
            WiFi_StartAP();
        }
    } else {
        Serial.println("[WiFi] Sin configuración guardada");
        WiFi_StartAP();
    }
}

void WiFi_StartAP() {
    Serial.println("[WiFi] Iniciando Punto de Acceso...");

    // AP_STA en vez de AP: con AP a secas la parte estacion queda apagada y el
    // equipo no volvia a intentar conectarse a la red nunca mas
    WiFi.mode(wifiSSID.length() > 0 ? WIFI_AP_STA : WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    
    localIP = WiFi.softAPIP();
    apMode = true;
    
    Serial.println("╔════════════════════════════════╗");
    Serial.printf("║ AP SSID: %-21s ║\n", AP_SSID);
    Serial.printf("║ AP Pass: %-21s ║\n", AP_PASS);
    Serial.printf("║ AP IP: %-23s ║\n", localIP.toString().c_str());
    Serial.println("║ Web: http://192.168.4.1       ║");
    Serial.println("╚════════════════════════════════╝");
    
    WiFi_StartWebServer();
}

// Comprueba la autenticación de la interfaz web.
// Con WEB_PASSWORD vacía no pide nada, igual que antes.
bool webAuthOK() {
    if(strlen(WEB_PASSWORD) == 0) return true;

    if(!server.authenticate(WEB_USER, WEB_PASSWORD)) {
        server.requestAuthentication();
        return false;
    }

    return true;
}

void WiFi_StartWebServer() {
    // Si ya está arrancado no se vuelve a registrar nada: WiFi_StartAP() llama
    // aquí en cada caída de la red y cada llamada añadía otra copia de los ~20
    // handlers a la lista interna de WebServer (fuga de memoria y despacho
    // duplicado).
    if(webServerStarted) {
        Serial.println("[Web] Servidor ya iniciado");
        return;
    }

    // Página principal
    server.on("/", []() {
        if(!webAuthOK()) return;
        String html = "<!DOCTYPE html><html><head>";
        html += "<meta charset='UTF-8'>";
        html += "<title>MiniOS ESP32-S3</title>";
        html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
        html += "<style>";
        html += "body{font-family:Arial;margin:20px;background:#1a1a1a;color:#fff;}";
        html += ".container{max-width:600px;margin:0 auto;padding:20px;background:#2a2a2a;border-radius:10px;}";
        html += "h1{color:#4CAF50;text-align:center;margin:10px 0;}";
        html += "h3{color:#4CAF50;margin:15px 0 10px 0;border-bottom:1px solid #4CAF50;padding-bottom:5px;}";
        html += "input,select{width:100%;padding:10px;margin:10px 0;box-sizing:border-box;background:#3a3a3a;color:#fff;border:1px solid #4CAF50;}";
        html += "button{background:#4CAF50;color:white;padding:12px;border:none;width:100%;cursor:pointer;border-radius:5px;font-size:16px;margin:5px 0;}";
        html += "button:hover{background:#45a049;}";
        html += ".btn-small{padding:8px;font-size:14px;width:auto;display:inline-block;margin:2px;}";
        html += ".btn-danger{background:#f44336;}";
        html += ".btn-danger:hover{background:#da190b;}";
        html += ".info{background:#3a3a3a;padding:10px;border-radius:5px;margin:10px 0;}";
        html += ".status{color:#4CAF50;font-weight:bold;}";
        html += ".gpio-item{background:#3a3a3a;padding:10px;margin:5px 0;border-radius:5px;display:flex;justify-content:space-between;align-items:center;}";
        html += ".gpio-controls{display:flex;gap:5px;}";
        html += ".slider{width:100%;height:25px;}";
        html += ".tab{overflow:hidden;border-bottom:2px solid #4CAF50;}";
        html += ".tab button{background:#2a2a2a;border:none;outline:none;cursor:pointer;padding:14px 16px;transition:0.3s;color:#fff;border-radius:0;}";
        html += ".tab button:hover{background:#3a3a3a;}";
        html += ".tab button.active{background:#4CAF50;}";
        html += ".tabcontent{display:none;padding:15px 0;}";
        html += ".tabcontent.active{display:block;}";
        html += "</style></head><body>";
        html += "<div class='container'>";
        html += "<h1>🖥️ MiniOS WiFi</h1>";

        // Tabs
        html += "<div class='tab'>";
        html += "<button class='tablinks active' onclick='openTab(event,\"System\")'>Sistema</button>";
        html += "<button class='tablinks' onclick='openTab(event,\"GPIO\")'>GPIO</button>";
        html += "<button class='tablinks' onclick='openTab(event,\"DHT\")'>Sensores</button>";
        html += "<button class='tablinks' onclick='openTab(event,\"TFT\")'>Pantalla</button>";
        html += "<button class='tablinks' onclick='openTab(event,\"WiFi\")'>WiFi</button>";
        html += "</div>";

        // Tab Sistema
        html += "<div id='System' class='tabcontent active'>";
        html += "<div class='info'>";
        html += "<p>📊 <span class='status'>Estado del Sistema</span></p>";
        html += "<p>⚡ CPU: " + String(ESP.getCpuFreqMHz()) + " MHz</p>";
        html += "<p>💾 RAM Libre: " + String(ESP.getFreeHeap()) + " bytes</p>";
        html += "<p>🌡️ Temperatura: " + String(temperatureRead(), 1) + "°C</p>";
        html += "<p>⏱️ Uptime: " + String(millis()/1000) + " segundos</p>";

        if(wifiConnected) {
            html += "<p>📶 WiFi: <span class='status'>Conectado</span></p>";
            html += "<p>🌐 SSID: " + WiFi.SSID() + "</p>";
            html += "<p>📍 IP: " + WiFi.localIP().toString() + "</p>";
            html += "<p>📊 Señal: " + String(WiFi.RSSI()) + " dBm</p>";
        } else {
            html += "<p>📶 WiFi: Modo AP</p>";
            html += "<p>📍 IP: " + WiFi.softAPIP().toString() + "</p>";
        }
        html += "</div>";
        html += "<h3>🎮 Control</h3>";
        html += "<form action='/led' method='POST'>";
        html += "<button name='state' value='toggle'>💡 Toggle LED</button>";
        html += "</form>";
        html += "<form action='/reboot' method='POST'>";
        html += "<button class='btn-danger' onclick='return confirm(\"¿Reiniciar?\")'>🔄 Reiniciar</button>";
        html += "</form>";
        html += "</div>";

        // Tab GPIO
        html += "<div id='GPIO' class='tabcontent'>";
        html += "<h3>⚡ Control de GPIOs</h3>";
        html += "<div id='gpioList'>Cargando...</div>";
        html += "<h3>➕ Configurar Nuevo GPIO</h3>";
        html += "<select id='newMode' onchange='updatePinsByMode()'>";
        html += "<option value='1'>OUTPUT</option>";
        html += "<option value='2'>INPUT</option>";
        html += "<option value='3'>INPUT_PULLUP</option>";
        html += "<option value='4'>PWM</option>";
        html += "</select>";
        html += "<select id='newPin'></select>";
        html += "<input type='text' id='newName' placeholder='Nombre (opcional)'>";
        html += "<button onclick='configGPIO()'>💾 Configurar</button>";
        html += "<p style='font-size:12px;margin-top:10px;color:#888;'>";
        html += "ℹ️ Pines analógicos (1-7): solo INPUT | Pines digitales: cualquier modo<br>";
        html += "⚙️ Click en ⚙️ en pines analógicos para configurar fórmulas (4-20mA, 0-10V, etc.)</p>";
        html += "</div>";

        // Tab Sensores DHT
        html += "<div id='DHT' class='tabcontent'>";
        html += "<h3>🌡️ Sensores DHT11</h3>";
        html += "<div id='dhtList'>Cargando...</div>";
        html += "<h3>➕ Configurar Nuevo Sensor</h3>";
        html += "<select id='dhtPin'></select>";
        html += "<input type='text' id='dhtName' placeholder='Nombre (opcional)'>";
        html += "<button onclick='configDHT()'>💾 Configurar DHT11</button>";
        html += "<p style='font-size:12px;margin-top:10px;color:#888;'>";
        html += "ℹ️ Solo pines digitales disponibles para sensores DHT</p>";
        html += "</div>";

        // Tab Pantalla TFT
        html += "<div id='TFT' class='tabcontent'>";
        html += "<h3>📺 Pantalla TFT 128x160</h3>";
        html += "<div id='tftStatus' class='info'>Cargando...</div>";
        html += "<h3>⚙️ Control de Pantalla</h3>";
        html += "<button onclick='initTFT()'>🔌 Inicializar Pantalla</button>";
        html += "<button class='btn-danger' onclick='setTFTMode(0)'>⏹️ Apagar</button>";
        html += "<h3>📊 Modo de Visualización</h3>";
        html += "<button onclick='setTFTMode(1)'>🖥️ Info Sistema</button>";
        html += "<button onclick='setTFTMode(2)'>🌡️ Sensores</button>";
        html += "<p style='font-size:12px;margin-top:10px;color:#888;'>";
        html += "ℹ️ Pines SPI: CS=10, DC=8, RST=9, MOSI=11, SCLK=12<br>";
        html += "📌 ST7735 compatible (128x160 RGB)</p>";
        html += "</div>";

        // Tab WiFi
        html += "<div id='WiFi' class='tabcontent'>";
        html += "<form action='/config' method='POST'>";
        html += "<h3>⚙️ Configurar WiFi</h3>";
        // Si se llega desde el escaneo con ?ssid=..., se rellena esa red
        String presetSSID = server.hasArg("ssid") ? server.arg("ssid") : wifiSSID;
        html += "<input type=\"text\" id=\"ssidInput\" name=\"ssid\" placeholder=\"SSID\" value=\"" + htmlEscape(presetSSID) + "\">";
        html += "<input type='password' name='pass' placeholder='Contraseña'>";
        html += "<button type='submit'>💾 Guardar y Conectar</button>";
        html += "</form>";
        html += "<form action='/scan' method='POST'>";
        html += "<button type='submit'>📡 Escanear Redes</button>";
        html += "</form>";
        html += "</div>";

        // JavaScript
        html += "<script>";
        html += "function openTab(e,t){var a=document.getElementsByClassName('tabcontent');";
        html += "for(var i=0;i<a.length;i++)a[i].className=a[i].className.replace(' active','');";
        html += "var b=document.getElementsByClassName('tablinks');";
        html += "for(var i=0;i<b.length;i++)b[i].className=b[i].className.replace(' active','');";
        html += "document.getElementById(t).className+=' active';e.currentTarget.className+=' active';}";

        html += "function loadGPIOs(){fetch('/api/gpio').then(r=>r.json()).then(d=>{";
        html += "var h='';if(d.length==0)h='<p>No hay pines configurados</p>';";
        html += "else{d.forEach(g=>{h+='<div class=\"gpio-item\">';";
        html += "h+='<div><b>'+g.name+'</b><br>GPIO '+g.pin+' - ';";
        html += "var m=['','OUTPUT','INPUT','INPUT_PU','PWM'][g.mode];h+=m;";
        html += "if(g.mode==1&&g.loop)h+=' (Loop: '+g.interval+'ms)';h+='</div>';";
        html += "h+='<div class=\"gpio-controls\">';";
        html += "if(g.mode==1){";
        html += "if(!g.loop){h+='<button class=\"btn-small\" onclick=\"setGPIO('+g.pin+',1)\">ON</button>';";
        html += "h+='<button class=\"btn-small\" onclick=\"setGPIO('+g.pin+',0)\">OFF</button>';";
        html += "h+='<button class=\"btn-small\" onclick=\"toggleLoop('+g.pin+')\">LOOP</button>';}";
        html += "else{h+='<button class=\"btn-small btn-danger\" onclick=\"stopLoop('+g.pin+')\">STOP</button>';}";
        html += "}";
        html += "else if(g.mode==4){h+='<input type=\"range\" class=\"slider\" min=\"0\" max=\"255\" value=\"'+g.value+'\" ';";
        html += "h+='onchange=\"setPWM('+g.pin+',this.value)\"><span id=\"pwm'+g.pin+'\">'+g.value+'</span>';}";
        html += "else if(g.mode==2||g.mode==3){";
        html += "var isAnalog=(g.pin>=1&&g.pin<=7);";
        html += "if(isAnalog){";
        html += "if(g.hasFormula){h+='<span>'+g.convertedValue.toFixed(3)+' '+g.unit+'</span><br>';";
        html += "h+='<span style=\"font-size:11px;color:#888\">RAW: '+g.value+'</span>';}";
        html += "else{h+='<span>RAW: '+g.value+'</span>';}";
        html += "h+='<button class=\"btn-small\" onclick=\"configFormula('+g.pin+')\">⚙️</button>';}";
        html += "else{h+='<span>Valor: '+g.value+'</span>';}";
        html += "}";
        html += "h+='<button class=\"btn-small btn-danger\" onclick=\"removeGPIO('+g.pin+')\">X</button>';";
        html += "h+='</div></div>';});};document.getElementById('gpioList').innerHTML=h;});}";

        html += "function loadSafePins(mode){";
        html += "var url='/api/gpio/safe';if(mode)url+='?mode='+mode;";
        html += "fetch(url).then(r=>r.json()).then(d=>{";
        html += "var s=document.getElementById('newPin');s.innerHTML='';";
        html += "d.forEach(p=>{var o=document.createElement('option');o.value=p;";
        html += "var isAnalog=(p>=1&&p<=7);";
        html += "o.text='GPIO '+p+(isAnalog?' (A)':'');";
        html += "s.add(o);});});}";

        html += "function updatePinsByMode(){var m=document.getElementById('newMode').value;loadSafePins(m);}";

        html += "function setGPIO(p,v){fetch('/api/gpio/set?pin='+p+'&value='+v,{method:'POST'}).then(()=>loadGPIOs());}";
        html += "function setPWM(p,v){document.getElementById('pwm'+p).innerText=v;";
        html += "fetch('/api/gpio/pwm?pin='+p+'&value='+v,{method:'POST'});}";
        html += "function removeGPIO(p){if(confirm('¿Eliminar GPIO '+p+'?'))";
        html += "fetch('/api/gpio/remove?pin='+p,{method:'POST'}).then(()=>loadGPIOs());}";
        html += "function toggleLoop(p){var i=prompt('Intervalo de parpadeo (ms):','500');";
        html += "if(i!=null&&i>=50)fetch('/api/gpio/loop?pin='+p+'&enabled=true&interval='+i,{method:'POST'}).then(()=>loadGPIOs());}";
        html += "function stopLoop(p){fetch('/api/gpio/loop?pin='+p+'&enabled=false',{method:'POST'}).then(()=>loadGPIOs());}";
        html += "function configGPIO(){var p=document.getElementById('newPin').value;";
        html += "var m=document.getElementById('newMode').value;var n=document.getElementById('newName').value;";
        html += "fetch('/api/gpio/config?pin='+p+'&mode='+m+'&name='+encodeURIComponent(n),{method:'POST'})";
        html += ".then(()=>{loadGPIOs();document.getElementById('newName').value='';});}";

        html += "function configFormula(pin){";
        html += "var type=prompt('Tipo de sensor:\\n1=4-20mA (250Ω)\\n2=0-10V (div 1:4)\\n3=0-3.3V\\n4=Custom','1');";
        html += "if(!type)return;";
        html += "var mult=1,offs=0,unit='',ftype='';";
        html += "if(type=='1'){mult=0.00322;offs=-12.88;unit='mA';ftype='4-20mA';}";
        html += "else if(type=='2'){mult=0.01287;unit='V';ftype='0-10V';}";
        html += "else if(type=='3'){mult=0.000806;unit='V';ftype='0-3.3V';}";
        html += "else if(type=='4'){";
        html += "mult=parseFloat(prompt('Multiplicador:','1'));";
        html += "offs=parseFloat(prompt('Offset:','0'));";
        html += "unit=prompt('Unidad:','');ftype='custom';}";
        html += "else{alert('Tipo inválido');return;}";
        html += "fetch('/api/gpio/formula?pin='+pin+'&enabled=true&multiplier='+mult+'&offset='+offs";
        html += "+'&unit='+encodeURIComponent(unit)+'&type='+encodeURIComponent(ftype),{method:'POST'})";
        html += ".then(()=>{loadGPIOs();alert('Fórmula configurada');});}";

        // JavaScript para DHT
        html += "function loadDHTs(){fetch('/api/dht').then(r=>r.json()).then(d=>{";
        html += "var h='';if(d.length==0)h='<p>No hay sensores configurados</p>';";
        html += "else{d.forEach(s=>{h+='<div class=\"gpio-item\">';";
        html += "h+='<div><b>'+s.name+'</b><br>GPIO '+s.pin+'<br>';";
        html += "if(s.status=='ok'){";
        html += "h+='🌡️ Temp: <b>'+s.temperature.toFixed(1)+'°C</b><br>';";
        html += "h+='💧 Humedad: <b>'+s.humidity.toFixed(1)+'%</b>';";
        html += "}else{h+='⚠️ Error de lectura';}";
        html += "h+='</div>';";
        html += "h+='<div class=\"gpio-controls\">';";
        html += "h+='<button class=\"btn-small\" onclick=\"readDHT('+s.pin+')\">🔄</button>';";
        html += "h+='<button class=\"btn-small btn-danger\" onclick=\"removeDHT('+s.pin+')\">X</button>';";
        html += "h+='</div></div>';});};document.getElementById('dhtList').innerHTML=h;});}";

        html += "function loadAvailablePins(){fetch('/api/gpio/safe?mode=1').then(r=>r.json()).then(d=>{";
        html += "var s=document.getElementById('dhtPin');s.innerHTML='';";
        html += "d.forEach(p=>{var o=document.createElement('option');o.value=p;o.text='GPIO '+p;s.add(o);});});}";

        html += "function configDHT(){var p=document.getElementById('dhtPin').value;";
        html += "var n=document.getElementById('dhtName').value;";
        html += "fetch('/api/dht/config?pin='+p+'&name='+encodeURIComponent(n),{method:'POST'})";
        html += ".then(r=>r.json()).then(d=>{if(d.success){loadDHTs();document.getElementById('dhtName').value='';";
        html += "alert('Sensor configurado. Espere unos segundos para la primera lectura.');}";
        html += "else{alert('Error: '+d.error);}});}";

        html += "function removeDHT(p){if(confirm('¿Eliminar sensor DHT en GPIO '+p+'?'))";
        html += "fetch('/api/dht/remove?pin='+p,{method:'POST'}).then(()=>loadDHTs());}";

        html += "function readDHT(p){fetch('/api/dht/read?pin='+p).then(()=>{";
        html += "setTimeout(loadDHTs,2000);});}";

        // JavaScript para pantalla TFT
        html += "function loadTFTStatus(){fetch('/api/tft/status').then(r=>r.json()).then(d=>{";
        html += "var h='<p><b>Estado:</b> ';";
        html += "if(d.initialized){h+='<span style=\"color:#4CAF50\">Inicializada</span>';}";
        html += "else{h+='<span style=\"color:#f44336\">No inicializada</span>';}";
        html += "h+='</p><p><b>Modo:</b> ';";
        html += "var modes=['Apagada','Info Sistema','Sensores'];";
        html += "h+=modes[d.mode]+'</p>';";
        html += "document.getElementById('tftStatus').innerHTML=h;});}";

        html += "function initTFT(){fetch('/api/tft/init',{method:'POST'}).then(r=>r.json()).then(d=>{";
        html += "if(d.success){alert('Pantalla inicializada');loadTFTStatus();}";
        html += "else{alert('Error: '+d.error);}});}";

        html += "function setTFTMode(m){fetch('/api/tft/mode?mode='+m,{method:'POST'}).then(r=>r.json()).then(d=>{";
        html += "if(d.success){loadTFTStatus();}";
        html += "else{alert('Error: '+d.error);}});}";

        html += "loadGPIOs();loadSafePins(1);loadDHTs();loadAvailablePins();loadTFTStatus();";
        html += "setInterval(loadGPIOs,3000);setInterval(loadDHTs,5000);setInterval(loadTFTStatus,3000);";

        // Al llegar desde el escaneo de redes se abre directamente la pestaña WiFi.
        // Antes el clic redirigía a /?ssid=... y la página ignoraba el parámetro:
        // seleccionar una red no hacía absolutamente nada.
        if(server.hasArg("ssid")) {
            html += "document.querySelectorAll('.tablinks').forEach(function(b){";
            html += "if(b.textContent.trim()=='WiFi')b.click();});";
        }

        html += "</script>";

        html += "</div></body></html>";
        server.send(200, "text/html", html);
    });
    
    // Configurar WiFi
    server.on("/config", HTTP_POST, []() {
        if(!webAuthOK()) return;
        String newSSID = server.arg("ssid");
        String newPass = server.arg("pass");
        
        if(newSSID.length() > 0) {
            wifiSSID = newSSID;
            wifiPassword = newPass;
            
            // Guardar en memoria flash (abre y cierra su propio namespace)
            WiFi_SaveCredentials(wifiSSID, wifiPassword);
            
            String html = "<html><head><meta charset='UTF-8'></head>";
            html += "<body style='background:#1a1a1a;color:#fff;font-family:Arial;text-align:center;padding:50px;'>";
            html += "<h2>✅ Configuración guardada</h2>";
            html += "<p>Conectando a: " + htmlEscape(wifiSSID) + "</p>";
            html += "<p>El sistema se reiniciará...</p>";
            html += "<script>setTimeout(function(){window.location='/'}, 3000);</script>";
            html += "</body></html>";
            server.send(200, "text/html", html);
            
            delay(1000);
            ESP.restart();
        } else {
            // Antes no se respondia nada y el navegador se quedaba colgado
            server.send(400, "text/html",
                "<html><head><meta charset='UTF-8'></head><body style='background:#1a1a1a;color:#fff;"
                "font-family:Arial;text-align:center;padding:50px;'><h2>El SSID no puede estar vacio</h2>"
                "<p><a style='color:#4CAF50' href='/'>Volver</a></p></body></html>");
        }
    });
    
    // Escanear redes
    server.on("/scan", HTTP_POST, []() {
        if(!webAuthOK()) return;
        String html = "<html><head>";
        html += "<meta charset='UTF-8'>";
        html += "<style>body{background:#1a1a1a;color:#fff;font-family:Arial;margin:20px;}";
        html += ".container{max-width:400px;margin:0 auto;padding:20px;background:#2a2a2a;border-radius:10px;}";
        html += ".network{background:#3a3a3a;padding:10px;margin:5px 0;border-radius:5px;cursor:pointer;}";
        html += ".network:hover{background:#4a4a4a;}";
        html += "button{background:#4CAF50;color:white;padding:10px;border:none;width:100%;cursor:pointer;border-radius:5px;}";
        html += "</style></head><body>";
        html += "<div class='container'>";
        html += "<h2>📡 Redes WiFi Encontradas</h2>";
        
        int n = WiFi.scanNetworks();
        if(n == 0) {
            html += "<p>No se encontraron redes</p>";
        } else {
            for(int i = 0; i < n; i++) {
                // jsEscape primero (cadena JS) y htmlEscape despues (atributo HTML):
                // un SSID con comilla simple rompia la pagina entera
                html += "<div class='network' onclick=\"selectNetwork('" + htmlEscape(jsEscape(WiFi.SSID(i))) + "')\">";
                html += "📶 " + htmlEscape(WiFi.SSID(i));
                html += " (" + String(WiFi.RSSI(i)) + " dBm)";
                html += WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? " 🔓" : " 🔐";
                html += "</div>";
            }
        }
        
        html += "<br><button onclick='window.location=\"/\"'>🔙 Volver</button>";
        html += "<script>function selectNetwork(ssid){";
        html += "if(confirm('¿Conectar a ' + ssid + '?')){";
        html += "window.location='/?ssid=' + encodeURIComponent(ssid);}}";
        html += "</script>";
        html += "</div></body></html>";
        
        server.send(200, "text/html", html);
    });
    
    // Control del LED
    server.on("/led", HTTP_POST, []() {
        if(!webAuthOK()) return;
        static bool ledState = false;
        ledState = !ledState;
        ledWrite(ledState);
        server.sendHeader("Location", "/");
        server.send(303);
    });
    
    // Reiniciar sistema
    server.on("/reboot", HTTP_POST, []() {
        if(!webAuthOK()) return;
        String html = "<html><head><meta charset='UTF-8'></head>";
        html += "<body style='background:#1a1a1a;color:#fff;text-align:center;padding:50px;'>";
        html += "<h2>🔄 Reiniciando...</h2>";
        html += "</body></html>";
        server.send(200, "text/html", html);
        delay(1000);
        ESP.restart();
    });
    
    // API JSON para estado
    server.on("/api/status", []() {
        if(!webAuthOK()) return;
        String json = "{";
        json += "\"uptime\":" + String(millis()/1000) + ",";
        json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
        json += "\"temperature\":" + String(temperatureRead()) + ",";
        json += "\"tasks\":" + String(taskCount) + ",";
        json += "\"wifi\":\"" + String(wifiConnected ? "connected" : "ap_mode") + "\",";
        json += "\"ip\":\"" + localIP.toString() + "\",";
        json += "\"rssi\":" + String(WiFi.RSSI());
        json += "}";

        server.send(200, "application/json", json);
    });

    // ============ API GPIO ============

    // Obtener estado de todos los GPIOs
    server.on("/api/gpio", HTTP_GET, []() {
        if(!webAuthOK()) return;
        String json = GPIO_GetStatusJSON();
        server.send(200, "application/json", json);
    });

    // Configurar un GPIO
    server.on("/api/gpio/config", HTTP_POST, []() {
        if(!webAuthOK()) return;
        if(!server.hasArg("pin") || !server.hasArg("mode")) {
            server.send(400, "application/json", "{\"error\":\"Missing parameters\"}");
            return;
        }

        int pin = server.arg("pin").toInt();
        int mode = server.arg("mode").toInt();
        String name = server.hasArg("name") ? server.arg("name") : "";

        if(mode < 1 || mode > 4) {
            server.send(400, "application/json", "{\"error\":\"Invalid mode\"}");
            return;
        }

        int result = GPIO_Configure(pin, (GPIOMode)mode, name);

        if(result >= 0) {
            server.send(200, "application/json", "{\"success\":true,\"pin\":" + String(pin) + "}");
        } else {
            server.send(400, "application/json", "{\"error\":\"Configuration failed\"}");
        }
    });

    // Escribir valor digital
    server.on("/api/gpio/set", HTTP_POST, []() {
        if(!webAuthOK()) return;
        if(!server.hasArg("pin") || !server.hasArg("value")) {
            server.send(400, "application/json", "{\"error\":\"Missing parameters\"}");
            return;
        }

        int pin = server.arg("pin").toInt();
        int value = server.arg("value").toInt();

        if(GPIO_DigitalWrite(pin, value)) {
            server.send(200, "application/json", "{\"success\":true,\"pin\":" + String(pin) + ",\"value\":" + String(value) + "}");
        } else {
            server.send(400, "application/json", "{\"error\":\"Pin not configured as OUTPUT\"}");
        }
    });

    // Escribir valor PWM
    server.on("/api/gpio/pwm", HTTP_POST, []() {
        if(!webAuthOK()) return;
        if(!server.hasArg("pin") || !server.hasArg("value")) {
            server.send(400, "application/json", "{\"error\":\"Missing parameters\"}");
            return;
        }

        int pin = server.arg("pin").toInt();
        int value = server.arg("value").toInt();

        if(GPIO_PWMWrite(pin, value)) {
            server.send(200, "application/json", "{\"success\":true,\"pin\":" + String(pin) + ",\"value\":" + String(value) + "}");
        } else {
            server.send(400, "application/json", "{\"error\":\"Pin not configured as PWM\"}");
        }
    });

    // Leer valor digital
    server.on("/api/gpio/read", HTTP_GET, []() {
        if(!webAuthOK()) return;
        if(!server.hasArg("pin")) {
            server.send(400, "application/json", "{\"error\":\"Missing pin parameter\"}");
            return;
        }

        int pin = server.arg("pin").toInt();
        int value = GPIO_DigitalRead(pin);

        if(value >= 0) {
            server.send(200, "application/json", "{\"success\":true,\"pin\":" + String(pin) + ",\"value\":" + String(value) + "}");
        } else {
            server.send(400, "application/json", "{\"error\":\"Pin not configured as INPUT\"}");
        }
    });

    // Eliminar GPIO
    server.on("/api/gpio/remove", HTTP_POST, []() {
        if(!webAuthOK()) return;
        if(!server.hasArg("pin")) {
            server.send(400, "application/json", "{\"error\":\"Missing pin parameter\"}");
            return;
        }

        int pin = server.arg("pin").toInt();

        if(GPIO_Remove(pin)) {
            server.send(200, "application/json", "{\"success\":true,\"pin\":" + String(pin) + "}");
        } else {
            server.send(400, "application/json", "{\"error\":\"Pin not found\"}");
        }
    });

    // Obtener lista de GPIOs disponibles según modo (excluye los ocupados)
    server.on("/api/gpio/safe", HTTP_GET, []() {
        if(!webAuthOK()) return;
        String modeStr = server.hasArg("mode") ? server.arg("mode") : "";
        int mode = modeStr.toInt();

        // Obtener pines en uso
        int inUseList[50];
        int inUseCount = 0;
        GPIO_GetInUse(inUseList, &inUseCount, 50);

        String json = "[";
        bool first = true;

        // Si no se especifica modo o es INPUT, mostrar analógicas y digitales
        if(mode == 0 || mode == GPIO_INPUT || mode == GPIO_INPUT_PULLUP) {
            // Agregar GPIOs analógicas (si no están en uso)
            for(int i = 0; i < ANALOG_GPIOS_COUNT; i++) {
                bool inUse = false;
                for(int j = 0; j < inUseCount; j++) {
                    if(ANALOG_GPIOS[i] == inUseList[j]) {
                        inUse = true;
                        break;
                    }
                }
                if(!inUse) {
                    if(!first) json += ",";
                    first = false;
                    json += String(ANALOG_GPIOS[i]);
                }
            }
            // Agregar GPIOs digitales (si no están en uso)
            for(int i = 0; i < DIGITAL_GPIOS_COUNT; i++) {
                bool inUse = false;
                for(int j = 0; j < inUseCount; j++) {
                    if(DIGITAL_GPIOS[i] == inUseList[j]) {
                        inUse = true;
                        break;
                    }
                }
                if(!inUse) {
                    if(!first) json += ",";
                    first = false;
                    json += String(DIGITAL_GPIOS[i]);
                }
            }
        }
        // Si es OUTPUT o PWM, solo mostrar digitales (si no están en uso)
        else if(mode == GPIO_OUTPUT || mode == GPIO_PWM) {
            for(int i = 0; i < DIGITAL_GPIOS_COUNT; i++) {
                bool inUse = false;
                for(int j = 0; j < inUseCount; j++) {
                    if(DIGITAL_GPIOS[i] == inUseList[j]) {
                        inUse = true;
                        break;
                    }
                }
                if(!inUse) {
                    if(!first) json += ",";
                    first = false;
                    json += String(DIGITAL_GPIOS[i]);
                }
            }
        }

        json += "]";
        server.send(200, "application/json", json);
    });

    // Activar/desactivar modo loop
    server.on("/api/gpio/loop", HTTP_POST, []() {
        if(!webAuthOK()) return;
        if(!server.hasArg("pin")) {
            server.send(400, "application/json", "{\"error\":\"Missing pin parameter\"}");
            return;
        }

        int pin = server.arg("pin").toInt();
        bool enabled = server.hasArg("enabled") ? (server.arg("enabled") == "true" || server.arg("enabled") == "1") : true;
        int interval = server.hasArg("interval") ? server.arg("interval").toInt() : 500;

        if(interval < 50) interval = 50;  // Mínimo 50ms

        if(GPIO_SetLoop(pin, enabled, interval)) {
            String json = "{\"success\":true,\"pin\":" + String(pin) + ",\"enabled\":" + String(enabled ? "true" : "false") + ",\"interval\":" + String(interval) + "}";
            server.send(200, "application/json", json);
        } else {
            server.send(400, "application/json", "{\"error\":\"Pin not configured as OUTPUT\"}");
        }
    });

    // Configurar fórmula de conversión para pin analógico
    server.on("/api/gpio/formula", HTTP_POST, []() {
        if(!webAuthOK()) return;
        if(!server.hasArg("pin")) {
            server.send(400, "application/json", "{\"error\":\"Missing pin parameter\"}");
            return;
        }

        int pin = server.arg("pin").toInt();

        // Verificar que sea un pin analógico configurado como INPUT
        bool found = false;
        int index = -1;

        for(int i = 0; i < MAX_GPIOS; i++) {
            if(gpioConfigs[i].active && gpioConfigs[i].pin == pin) {
                found = true;
                index = i;
                break;
            }
        }

        if(!found) {
            server.send(400, "application/json", "{\"error\":\"Pin not configured\"}");
            return;
        }

        if(!GPIO_InList(pin, ANALOG_GPIOS, ANALOG_GPIOS_COUNT)) {
            server.send(400, "application/json", "{\"error\":\"Pin is not analog\"}");
            return;
        }

        if(gpioConfigs[index].mode != GPIO_INPUT && gpioConfigs[index].mode != GPIO_INPUT_PULLUP) {
            server.send(400, "application/json", "{\"error\":\"Pin must be configured as INPUT\"}");
            return;
        }

        // Configurar fórmula
        gpioConfigs[index].hasFormula = server.hasArg("enabled") ? (server.arg("enabled") == "true" || server.arg("enabled") == "1") : true;

        if(gpioConfigs[index].hasFormula) {
            gpioConfigs[index].multiplier = server.hasArg("multiplier") ? server.arg("multiplier").toFloat() : 1.0;
            gpioConfigs[index].offset = server.hasArg("offset") ? server.arg("offset").toFloat() : 0.0;
            gpioConfigs[index].unit = server.hasArg("unit") ? server.arg("unit") : "";
            gpioConfigs[index].formulaType = server.hasArg("type") ? server.arg("type") : "custom";
        }

        GPIO_SaveConfig();

        String json = "{\"success\":true,\"pin\":" + String(pin) + "}";
        server.send(200, "application/json", json);
    });

    // ============ API DHT ============

    // Obtener estado de todos los sensores DHT
    server.on("/api/dht", HTTP_GET, []() {
        if(!webAuthOK()) return;
        String json = DHT_GetStatusJSON();
        server.send(200, "application/json", json);
    });

    // Configurar un nuevo sensor DHT
    server.on("/api/dht/config", HTTP_POST, []() {
        if(!webAuthOK()) return;
        if(!server.hasArg("pin")) {
            server.send(400, "application/json", "{\"error\":\"Missing pin parameter\"}");
            return;
        }

        int pin = server.arg("pin").toInt();
        String name = server.hasArg("name") ? server.arg("name") : "";

        int result = DHT_Configure(pin, name);

        if(result >= 0) {
            server.send(200, "application/json", "{\"success\":true,\"pin\":" + String(pin) + "}");
        } else {
            server.send(400, "application/json", "{\"error\":\"Configuration failed\"}");
        }
    });

    // Eliminar sensor DHT
    server.on("/api/dht/remove", HTTP_POST, []() {
        if(!webAuthOK()) return;
        if(!server.hasArg("pin")) {
            server.send(400, "application/json", "{\"error\":\"Missing pin parameter\"}");
            return;
        }

        int pin = server.arg("pin").toInt();

        if(DHT_Remove(pin)) {
            server.send(200, "application/json", "{\"success\":true,\"pin\":" + String(pin) + "}");
        } else {
            server.send(400, "application/json", "{\"error\":\"Sensor not found\"}");
        }
    });

    // Leer un sensor DHT específico
    server.on("/api/dht/read", HTTP_GET, []() {
        if(!webAuthOK()) return;
        if(!server.hasArg("pin")) {
            server.send(400, "application/json", "{\"error\":\"Missing pin parameter\"}");
            return;
        }

        int pin = server.arg("pin").toInt();

        // Buscar el sensor
        for(int i = 0; i < MAX_DHT_SENSORS; i++) {
            if(dhtSensors[i].active && dhtSensors[i].pin == pin) {
                DHT_Read(i);
                String json = "{";
                json += "\"pin\":" + String(pin) + ",";
                json += "\"temperature\":" + String(dhtSensors[i].temperature, 1) + ",";
                json += "\"humidity\":" + String(dhtSensors[i].humidity, 1) + ",";
                json += "\"status\":\"" + String(dhtSensors[i].lastReadOk ? "ok" : "error") + "\"";
                json += "}";
                server.send(200, "application/json", json);
                return;
            }
        }

        server.send(404, "application/json", "{\"error\":\"Sensor not found\"}");
    });

    // ============ API PANTALLA TFT ============

    // Inicializar pantalla
    server.on("/api/tft/init", HTTP_POST, []() {
        if(!webAuthOK()) return;
        if(TFT_Init()) {
            server.send(200, "application/json", "{\"success\":true,\"message\":\"Pantalla inicializada\"}");
        } else {
            server.send(500, "application/json", "{\"error\":\"Error al inicializar pantalla\"}");
        }
    });

    // Apagar pantalla
    server.on("/api/tft/off", HTTP_POST, []() {
        if(!webAuthOK()) return;
        TFT_Off();
        server.send(200, "application/json", "{\"success\":true,\"message\":\"Pantalla apagada\"}");
    });

    // Configurar modo de visualización
    server.on("/api/tft/mode", HTTP_POST, []() {
        if(!webAuthOK()) return;
        if(!server.hasArg("mode")) {
            server.send(400, "application/json", "{\"error\":\"Missing mode parameter\"}");
            return;
        }

        int mode = server.arg("mode").toInt();

        if(mode < 0 || mode > 2) {
            server.send(400, "application/json", "{\"error\":\"Invalid mode (0=Off, 1=System, 2=Sensors)\"}");
            return;
        }

        if(!tftInitialized && mode > 0) {
            if(!TFT_Init()) {
                server.send(500, "application/json", "{\"error\":\"Failed to initialize display\"}");
                return;
            }
        }

        // Si cambia el modo, marcar para redibujo completo
        if(tftDisplayMode != mode) {
            tftFirstDraw = true;
        }

        tftDisplayMode = mode;
        tftEnabled = (mode > 0);

        if(mode == 0) {
            TFT_Off();
        } else {
            TFT_Update();
        }

        String json = "{\"success\":true,\"mode\":" + String(mode) + "}";
        server.send(200, "application/json", json);
    });

    // Obtener estado de pantalla
    server.on("/api/tft/status", HTTP_GET, []() {
        if(!webAuthOK()) return;
        String json = "{";
        json += "\"initialized\":" + String(tftInitialized ? "true" : "false") + ",";
        json += "\"enabled\":" + String(tftEnabled ? "true" : "false") + ",";
        json += "\"mode\":" + String(tftDisplayMode);
        json += "}";
        server.send(200, "application/json", json);
    });

    server.begin();
    webServerStarted = true;
    Serial.printf("[Web] Servidor iniciado en puerto %d%s\n", WEB_PORT,
                  strlen(WEB_PASSWORD) > 0 ? " (con autenticación)" : "");
}

// ============================================
// COMANDOS DEL SISTEMA
// ============================================
String inputBuffer = "";

void OS_ProcessCommand() {
    if(Serial.available()) {
        char c = Serial.read();
        
        if(c == '\n' || c == '\r') {
            if(inputBuffer.length() > 0) {
                OS_ExecuteCommand(inputBuffer);
                inputBuffer = "";
                Serial.print("> ");
            }
        } else if(c == 8 || c == 127) { // Backspace
            if(inputBuffer.length() > 0) {
                inputBuffer.remove(inputBuffer.length() - 1);
                Serial.print("\b \b");
            }
        } else {
            inputBuffer += c;
            Serial.print(c);
        }
    }
}

void OS_ExecuteCommand(String cmd) {
    cmd.trim();
    
    if(cmd == "help") {
        Serial.println("\n╔═══════════════════════════════╗");
        Serial.println("║         COMANDOS              ║");
        Serial.println("╠═══════════════════════════════╣");
        Serial.println("║ ps        - Ver tareas        ║");
        Serial.println("║ free      - Memoria libre     ║");
        Serial.println("║ temp      - Temperatura CPU   ║");
        Serial.println("║ ip        - Mostrar IP        ║");
        Serial.println("║ wifi scan - Escanear redes    ║");
        Serial.println("║ wifi set  - Config. WiFi      ║");
        Serial.println("║ wifi info - Estado WiFi       ║");
        Serial.println("║ wifi ap   - Modo AP           ║");
        Serial.println("║ led on/off- Control LED       ║");
        Serial.println("║ reboot    - Reiniciar         ║");
        Serial.println("╠═══════════════════════════════╣");
        Serial.println("║ 💡 GPIO y DHT: usar Web       ║");
        Serial.println("╚═══════════════════════════════╝\n");
    }
    else if(cmd == "ps") {
        Serial.println("\n=== TAREAS ACTIVAS ===");
        Serial.println("ID | Nombre      | Pri | Intervalo");
        Serial.println("---+--------------+-----+----------");
        for(int i = 0; i < MAX_TASKS; i++) {
            if(tasks[i].active) {
                Serial.printf("%2d | %-12s | %d   | %lu ms\n", 
                    i, tasks[i].name, tasks[i].priority, tasks[i].interval);
            }
        }
        Serial.printf("Total: %d/%d tareas\n\n", taskCount, MAX_TASKS);
    }
    else if(cmd == "free") {
        Serial.printf("\n💾 RAM Total: %lu bytes\n", (unsigned long)ESP.getHeapSize());
        Serial.printf("💾 RAM Libre: %lu bytes\n", (unsigned long)ESP.getFreeHeap());
        Serial.printf("💾 RAM Usada: %lu bytes\n\n", (unsigned long)(ESP.getHeapSize() - ESP.getFreeHeap()));
    }
    else if(cmd == "temp") {
        Serial.printf("\n🌡️ Temperatura CPU: %.1f°C\n\n", temperatureRead());
    }
    else if(cmd == "ip") {
        Serial.printf("\n📍 IP: %s\n", localIP.toString().c_str());
        if(wifiConnected) {
            Serial.println("📡 Modo: Cliente WiFi");
            Serial.printf("📶 SSID: %s\n", WiFi.SSID().c_str());
            Serial.printf("📊 Señal: %d dBm\n", (int)WiFi.RSSI());
            if(apMode) Serial.printf("📡 AP también activo: %s\n", AP_SSID);
        } else if(apMode) {
            Serial.println("📡 Modo: Punto de Acceso");
            Serial.printf("📶 SSID: %s\n", AP_SSID);
        }
        Serial.printf("🌐 Web: http://%s\n\n", localIP.toString().c_str());
    }
    else if(cmd == "wifi scan") {
        Serial.println("\n📡 Escaneando redes WiFi...");
        int n = WiFi.scanNetworks();
        
        if(n == 0) {
            Serial.println("No se encontraron redes\n");
        } else {
            Serial.printf("Se encontraron %d redes:\n", n);
            Serial.println("-----------------------------------");
            for(int i = 0; i < n; i++) {
                Serial.printf("%2d | %-20s | %3d dBm | %s\n",
                    i+1,
                    WiFi.SSID(i).c_str(),
                    (int)WiFi.RSSI(i),
                    WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "Abierta" : "Segura");
            }
            Serial.println();
        }
    }
    else if(cmd == "wifi set") {
        Serial.println("\nConfiguración WiFi:");
        Serial.print("SSID: ");
        
        // Con espera acotada: antes se quedaba bloqueado indefinidamente y
        // congelaba el servidor web y todas las tareas
        unsigned long waitStart = millis();
        while(!Serial.available()) {
            if(millis() - waitStart > 30000) {
                Serial.println("\n⏱️ Tiempo agotado, cancelado");
                return;
            }
            delay(10);
        }
        String ssid = Serial.readStringUntil('\n');
        ssid.trim();

        if(ssid.length() == 0) {
            Serial.println("\n❌ SSID vacío, cancelado");
            return;
        }
        
        Serial.print("Password: ");
        waitStart = millis();
        while(!Serial.available()) {
            if(millis() - waitStart > 30000) {
                Serial.println("\n⏱️ Tiempo agotado, cancelado");
                return;
            }
            delay(10);
        }
        String pass = Serial.readStringUntil('\n');
        pass.trim();
        
        // Guardar configuración
        WiFi_SaveCredentials(ssid, pass);
        
        Serial.println("\n✅ Configuración guardada. Reiniciando...");
        delay(1000);
        ESP.restart();
    }
    else if(cmd == "wifi info") {
        Serial.println("\n=== INFORMACIÓN WiFi ===");
        if(wifiConnected) {
            Serial.println("Estado: Conectado ✅");
            Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
            Serial.printf("IP Local: %s\n", WiFi.localIP().toString().c_str());
            Serial.printf("Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
            Serial.printf("DNS: %s\n", WiFi.dnsIP().toString().c_str());
            Serial.printf("MAC: %s\n", WiFi.macAddress().c_str());
            Serial.printf("Señal: %d dBm\n", (int)WiFi.RSSI());
            Serial.printf("Canal: %d\n", (int)WiFi.channel());
        } else if(apMode) {
            Serial.println("Estado: Modo AP 📡");
            Serial.printf("SSID: %s\n", AP_SSID);
            Serial.printf("IP: %s\n", WiFi.softAPIP().toString().c_str());
            Serial.printf("MAC: %s\n", WiFi.softAPmacAddress().c_str());
            Serial.printf("Clientes: %d\n", WiFi.softAPgetStationNum());
        } else {
            Serial.println("Estado: Desconectado ❌");
        }
        Serial.println();
    }
    else if(cmd == "wifi ap") {
        Serial.println("\n🔄 Cambiando a modo AP...");
        WiFi_SaveCredentials("", "");
        delay(500);
        ESP.restart();
    }
    else if(cmd == "led on") {
        ledWrite(true);
        Serial.println("\n💡 LED encendido\n");
    }
    else if(cmd == "led off") {
        ledWrite(false);
        Serial.println("\n💡 LED apagado\n");
    }
    else if(cmd == "reboot") {
        Serial.println("\n🔄 Reiniciando...");
        delay(500);
        ESP.restart();
    }
    else {
        Serial.println("\n❌ Comando no reconocido. Use 'help'\n");
    }
}

// ============================================
// TAREAS DE EJEMPLO
// ============================================

// Parpadeo del LED
void taskBlink() {
    static bool ledState = false;
    ledState = !ledState;
    ledWrite(ledState);
}

// Monitor del sistema
void taskMonitor() {
    static int count = 0;
    if(++count >= 60) { // Cada 30 segundos (60 * 500ms)
        Serial.printf("\n[Monitor] Up:%lus | RAM:%lu | T:%.1f°C | WiFi:%s\n> ", 
            systemTime/1000, 
            (unsigned long)ESP.getFreeHeap(), 
            temperatureRead(),
            wifiConnected ? "OK" : (apMode ? "AP" : "OFF"));
        count = 0;
    }
}

// Tarea del servidor web
void taskWebServer() {
    server.handleClient();
}

// Verificar conexión WiFi
void taskWiFiCheck() {
    static int failCount = 0;
    static unsigned long lastRetry = 0;

    bool connected = (WiFi.status() == WL_CONNECTED);

    if(connected) {
        if(!wifiConnected) {
            wifiConnected = true;
            localIP = WiFi.localIP();
            Serial.printf("\n[WiFi] ✅ Conectado! IP: %s\n> ", localIP.toString().c_str());
        }
        failCount = 0;
        return;
    }

    if(wifiConnected) {
        wifiConnected = false;
        Serial.println("\n[WiFi] ❌ Conexión perdida\n> ");
    }

    // Sin credenciales guardadas solo tiene sentido el modo AP
    if(wifiSSID.length() == 0) return;

    failCount++;

    // Reintentar la conexión periódicamente, también estando en modo AP.
    // Antes, una vez que apMode pasaba a true no se volvía a intentar jamás y
    // el equipo se quedaba en AP hasta un reinicio manual.
    if(millis() - lastRetry >= WIFI_RETRY_INTERVAL) {
        lastRetry = millis();
        Serial.printf("\n[WiFi] 🔄 Reintentando conexión a %s...\n> ", wifiSSID.c_str());
        WiFi.disconnect();
        WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    }

    // Tras 20 s sin red se levanta el AP para no quedarse incomunicado
    if(!apMode && failCount > 20) {
        Serial.println("\n[WiFi] ⚠️ Sin conexión, levantando modo AP...");
        WiFi_StartAP();
    }
}

// ============================================
// PROGRAMA PRINCIPAL
// ============================================

void setup() {
    pinMode(LED_PIN, OUTPUT);
    ledWrite(false);  // Apagar LED al inicio

    // Inicializar OS
    OS_Init();
    
    // Añadir tareas del sistema
    // OS_AddTask(taskBlink, 500, "Blink", 0);           // Desactivado - LED no parpadea
    OS_AddTask(taskMonitor, 500, "Monitor", 1);       // Prioridad normal
    OS_AddTask(taskWebServer, 10, "WebServer", 3);    // Alta prioridad
    OS_AddTask(taskWiFiCheck, 1000, "WiFiCheck", 2);  // Prioridad media
    OS_AddTask(taskGPIOBlink, 50, "GPIOBlink", 2);    // Parpadeo de GPIOs
    OS_AddTask(taskDHTRead, 5000, "DHTRead", 1);      // Lectura de sensores DHT cada 5s
    OS_AddTask(taskTFTUpdate, 2000, "TFTUpdate", 1);  // Actualizar pantalla TFT cada 2s
    
    Serial.print("> ");
}

void loop() {
    // Procesar comandos
    OS_ProcessCommand();
    
    // Ejecutar scheduler
    OS_Run();
    
    // Pequeña pausa
    delay(1);
}