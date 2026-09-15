# Firmware SCADA local por Wi-Fi

Esta variante conserva la adquisicion ADS1263 y reemplaza la conectividad de
produccion por una red local creada por el ESP32.

```text
ADS1263 -> ESP32 SoftAP SCADA-EV -> navegador
```

## Acceso

1. Encender el ESP32.
2. Conectar la laptop o telefono a `SCADA-EV`.
3. Abrir `http://192.168.4.1/`.

Endpoints:

- `GET /`: dashboard.
- `GET /api/latest`: ultimo JSON version 3.
- `GET /health`: estado del ADC, medicion, clientes y uptime.

La primera llamada a `/api/latest` devuelve HTTP 503 hasta que termine la
primera ventana de medicion. La pagina vuelve a consultar cada cinco segundos.

## Configuracion

El firmware compila usando `secrets.example.h`. Para cambiar el SSID o proteger
la red, copiarlo como `secrets.h` y editar solamente:

```cpp
#define SCADA_AP_SSID "SCADA-EV"
#define SCADA_AP_PASSWORD ""
```

Una clave WPA2 debe tener al menos ocho caracteres. Sin clave, la red queda
abierta y debe usarse solo para la prueba local prevista.

## Limites

- No se conecta a la red universitaria.
- No publica MQTT ni sincroniza GitHub.
- `timestamp_unix` permanece en cero porque no existe NTP.
- El navegador obtiene la hora local de recepcion.
- El ESP32 conserva solamente el JSON mas reciente.
