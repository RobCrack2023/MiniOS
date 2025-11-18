# MiniOS WiFi Client

Firmware ligero para ESP32 que se conecta al backend centralizado MiniOS.

## Diferencias con MiniOS_WiFi Original

| Característica | Original | Client |
|----------------|----------|--------|
| Servidor Web | Sí (HTML embebido) | No |
| Tamaño Flash | ~800KB | ~300KB |
| Configuración | Local via web | Remota via backend |
| Acceso | Solo red local | Desde cualquier lugar |

## Librerías Requeridas

Instalar desde el Library Manager de Arduino IDE:

1. **WebSockets** by Markus Sattler (v2.4.0+)
2. **ArduinoJson** by Benoit Blanchon (v6.21.0+)
3. **DHT sensor library** by Adafruit (v1.4.4+)

## Configuración Inicial

### Via Serial (115200 baud)

1. Configurar WiFi:
```
wifi MiRed MiPassword
```

2. Configurar backend:
```
server 192.168.1.100 3000
```

O si tienes dominio:
```
server mi-servidor.com 3000
```

3. Ver estado:
```
status
```

4. Reiniciar:
```
reboot
```

## Funcionamiento

1. El ESP32 se conecta al WiFi configurado
2. Establece conexión WebSocket con el backend
3. Se registra automáticamente usando su MAC Address
4. Recibe configuraciones (GPIO, DHT) del backend
5. Envía datos de sensores cada 5 segundos
6. Ejecuta comandos recibidos (set GPIO, reiniciar, OTA)

## Actualizaciones OTA

1. Subir archivo .bin al backend desde el dashboard
2. Seleccionar "Enviar a todos" o actualizar dispositivo específico
3. El ESP32 descarga e instala automáticamente
4. Se reinicia con el nuevo firmware

## Estructura de Mensajes

### Registro (ESP32 → Backend)
```json
{
  "type": "register",
  "mac_address": "AA:BB:CC:DD:EE:FF",
  "firmware_version": "1.0.0",
  "ip_address": "192.168.1.50"
}
```

### Datos de Sensores (ESP32 → Backend)
```json
{
  "type": "data",
  "mac_address": "AA:BB:CC:DD:EE:FF",
  "payload": {
    "temperature": 25.5,
    "humidity": 60,
    "gpio": [
      {"pin": 2, "value": 1},
      {"pin": 4, "value": 0}
    ]
  }
}
```

### Comando GPIO (Backend → ESP32)
```json
{
  "type": "command",
  "action": "set_gpio",
  "pin": 2,
  "value": 1
}
```

### Comando OTA (Backend → ESP32)
```json
{
  "type": "command",
  "action": "ota_update",
  "ota_id": 1,
  "filename": "firmware_1.0.1.bin",
  "filesize": 512000,
  "checksum": "abc123..."
}
```

## Compilación

### Arduino IDE

1. Board: "ESP32S3 Dev Module"
2. Partition Scheme: "Default 4MB with spiffs"
3. Upload Speed: 921600

### PlatformIO

```ini
[env:esp32s3]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
lib_deps =
    links2004/WebSockets@^2.4.0
    bblanchon/ArduinoJson@^6.21.0
    adafruit/DHT sensor library@^1.4.4
```

## Troubleshooting

### No conecta al WiFi
- Verificar SSID y contraseña
- El ESP32 debe estar cerca del router

### No conecta al backend
- Verificar IP/puerto del servidor
- El backend debe estar corriendo
- Verificar firewall del VPS

### OTA falla
- Verificar espacio en flash
- El archivo .bin debe ser válido
- Conexión estable durante descarga

## Pines por Defecto

- **LED Status**: GPIO 2 (LED integrado)
- Los demás GPIOs se configuran desde el dashboard
