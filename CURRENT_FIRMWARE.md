# Firmware actual: adquisicion ADS1263 y publicacion MQTT

El firmware operativo nuevo esta en:

```text
firmware/SCADA_ADC_MQTT/SCADA_ADC_MQTT.ino
```

La ruta de datos es:

```text
SCT-013 -> burden 22 ohm + bias 2.5 V -> ADS1263 IN9/COM -> ESP32 -> MQTT -> servidor Ubuntu
```

## Preparacion

1. Copiar `firmware/SCADA_ADC_MQTT/secrets.example.h` como `secrets.h`.
2. Completar WiFi, IP privada del servidor Mosquitto y credenciales del dispositivo.
3. Mantener `SCADA_MQTT_USE_TLS=0` solamente durante la primera prueba LAN.
4. Compilar `SCADA_ADC_MQTT.ino` para `ESP32 Dev Module`.
5. Abrir el monitor serie a 115200 baud.

`secrets.h` esta excluido de Git y no debe subirse.

## Topic y mensaje

El dispositivo BR1 publica cada cinco segundos en:

```text
scada/ev/br1/medicion
```

Campos principales:

- `corriente_rms_a`: corriente primaria calculada para SCT-013-000 100 A / 50 mA y burden de 22 ohm.
- `senal_rms_v`: RMS AC medido en el ADS1263.
- `bias_v`: nivel DC de la entrada acondicionada.
- `frecuencia_hz`: frecuencia detectada.
- `valido`: indica que no existen rieles, referencia flotante ni frecuencia incompatible.
- `cargando`: corriente valida igual o superior a 1 A.
- `secuencia`: contador monotono desde el ultimo arranque.
- `timestamp_unix`: cero hasta que NTP sincroniza.

## Independencia del broker

El cliente implementa MQTT 3.1.1 estandar y no usa funciones de HiveMQ. El broker puede ser Mosquitto, EMQX, HiveMQ u otro compatible cambiando solamente `secrets.h`.

Las publicaciones usan QoS 1 y solamente salen de la cola cuando el broker responde con `PUBACK`. Durante una interrupcion conserva las ultimas 16 publicaciones en RAM. Si la cola se llena, elimina primero la mas antigua y aumenta `descartados`.

## Cableado ADS1263

| ADS1263 | ESP32 |
|---|---:|
| CS | GPIO27 |
| SCLK | GPIO18 |
| DOUT/MISO | GPIO19 |
| DIN/MOSI | GPIO23 |
| DRDY | GPIO25 |
| RESET | GPIO26 |
| VCC digital | 3V3 |
| AVDD | 5V |
| AVSS/GND | GND |

Se mide `IN9` respecto a `COM`. El jumper `COM-AVSS` debe permanecer instalado. La entrada que llega a IN9 debe estar acondicionada alrededor de 2.5 V; no se debe conectar el SCT directamente entre IN9 y tierra sin el burden y el bias. El firmware rechaza una lectura cuya media no este entre 2.0 V y 3.0 V.
