# MiniOS WiFi — ESP32-S3

Firmware para ESP32-S3 con un mini planificador cooperativo, control de GPIO,
sensores DHT11, pantalla TFT ST7735 e interfaz web de configuración.

Se compila directamente en el Arduino IDE (un único sketch, sin PlatformIO).

## Requisitos

| Componente | Versión probada |
|------------|-----------------|
| ESP32 Arduino Core | 3.3.x (usa la API `ledcAttach`/`ledcDetach`, no la de canales de 2.x) |
| Placa | ESP32-S3 Dev Module |
| Librería DHT sensor library | Adafruit |
| Adafruit GFX Library | Adafruit |
| Adafruit ST7735 and ST7789 Library | Adafruit |

## Primer arranque

Sin credenciales guardadas el equipo levanta un punto de acceso:

- SSID: `MiniOS-ESP32`
- Contraseña: `12345678` — **cambiar en `AP_PASS` antes de desplegar**
- Interfaz web: <http://192.168.4.1>

Desde la pestaña **WiFi** se escanea la red, se elige una y se guardan las
credenciales. El equipo reinicia y se conecta como estación.

## Configuración en el código

Al principio del sketch:

| Define | Por defecto | Para qué sirve |
|--------|-------------|----------------|
| `AP_SSID` / `AP_PASS` | `MiniOS-ESP32` / `12345678` | Punto de acceso de respaldo |
| `WEB_PORT` | 80 | Puerto del servidor web |
| `WIFI_RETRY_INTERVAL` | 30000 | Cada cuánto se reintenta la red perdida (ms) |
| `WEB_USER` / `WEB_PASSWORD` | `admin` / *(vacía)* | Con la contraseña vacía la web no pide nada. Al ponerle valor, **todas** las rutas piden HTTP Basic |
| `MAX_GPIOS` / `MAX_DHT_SENSORS` / `MAX_TASKS` | 10 / 4 / 8 | Límites de las tablas estáticas |

> La interfaz web no tiene autenticación por defecto: cualquiera en la misma red
> puede cambiar GPIOs y reiniciar el equipo. Si el dispositivo está en una red
> compartida, define `WEB_PASSWORD`.

## Pines

Ver también `gpio.txt`.

| Función | GPIO |
|---------|------|
| Analógicos (ADC, solo entrada) | 1, 2, 4, 5, 6, 7 |
| Digitales (cualquier modo) | 0, 3, 14–21, 36–42, 45, 46 |
| I2C (reservados) | 8 (SDA), 9 (SCL) |
| SPI / TFT | 10 (CS), 11 (MOSI), 12 (SCLK), 8 (DC), 9 (RST) |

**Cuidado con estos pines**, aunque la interfaz los ofrezca:

- **0, 3, 45, 46** son pines de *strapping* del ESP32-S3. Configurar el GPIO 0
  como salida puede impedir que la placa arranque.
- **19 y 20** son USB D− / D+. Usarlos rompe el USB nativo.

## Comandos por puerto serie (115200 baudios)

```
help        ps          free        temp        ip
wifi scan   wifi set    wifi info   wifi ap
led on/off  reboot
```

`wifi set` pide SSID y contraseña con un tiempo de espera de 30 s por campo.
La configuración de GPIO y sensores se hace desde la interfaz web.

## API HTTP

| Método | Ruta | Descripción |
|--------|------|-------------|
| GET | `/` | Interfaz web (acepta `?ssid=` para prerrellenar la red) |
| POST | `/config` | Guardar credenciales WiFi y reiniciar |
| POST | `/scan` | Escanear redes |
| POST | `/led`, `/reboot` | LED integrado y reinicio |
| GET | `/api/status` | Estado del sistema en JSON |
| GET | `/api/gpio` | Estado de los GPIO configurados |
| GET | `/api/gpio/safe?mode=N` | Pines disponibles para ese modo |
| POST | `/api/gpio/config`, `/set`, `/pwm`, `/loop`, `/formula`, `/remove` | Gestión de GPIO |
| GET | `/api/dht` · POST `/api/dht/config`, `/remove` · GET `/api/dht/read` | Sensores DHT11 |
| POST | `/api/tft/init`, `/mode`, `/off` · GET `/api/tft/status` | Pantalla |

Modos de GPIO: `1`=OUTPUT, `2`=INPUT, `3`=INPUT_PULLUP, `4`=PWM.

## Almacenamiento en NVS

Tres espacios de nombres, cada uno abierto y cerrado en su propia operación:

| Namespace | Contenido |
|-----------|-----------|
| `minios` | SSID y contraseña WiFi |
| `gpio` | `count` + ranuras consecutivas `g0_…`, `g1_…` |
| `dht` | `count` + ranuras consecutivas `d0_…`, `d1_…` |

Las ranuras se **compactan** al guardar: los pines activos se escriben siempre
desde el índice 0, independientemente de la posición que ocupen en el array en
memoria.

> **Al actualizar desde una versión anterior**: revisa la pestaña GPIO en el
> primer arranque. Si el guardado antiguo había quedado inconsistente (pasaba al
> eliminar un pin que no fuera el último), puede que falte alguna entrada.
> Volver a configurarla y guardar deja el almacenamiento correcto de forma
> definitiva.

## Comportamiento del WiFi

- Al arrancar intenta conectarse durante 10 s con las credenciales guardadas.
- Si falla, levanta el AP y **sigue reintentando** la red cada 30 s en modo
  `WIFI_AP_STA`: al volver la red se reconecta solo, sin reiniciar.
- El servidor web se registra una sola vez, aunque el AP se levante después.
