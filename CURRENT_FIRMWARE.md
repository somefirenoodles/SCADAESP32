# Firmware actual: adquisicion ADS1263 y publicacion MQTT

El firmware operativo nuevo esta en:

```text
firmware/SCADA_ADC_MQTT/SCADA_ADC_MQTT.ino
```

La ruta de datos es:

```text
SCT-013 -> burden y acondicionamiento -> ADS1263 IN9-IN1 -> ESP32 -> MQTT -> servidor Ubuntu
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
- `bias_v`: media de la tension diferencial IN9-IN1, conservada con ese nombre por compatibilidad del mensaje.
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

Se mide diferencialmente `IN9` respecto a `IN1` (`IN9-IN1`). Una tension diferencial negativa es valida y ya no se interpreta como saturacion. El firmware conserva la medicion de AVDD-AVSS para determinar su escala real.

## Recalibracion IN9-IN1

El firmware se entrega temporalmente con `CALIBRATION_MODE=true`. En ese modo desactiva WiFi/MQTT y espera en el generador:

```text
Sine, 60 Hz, High=3.000 V, Low=2.000 V, Phase=0
```

Conecte la salida del generador a IN9 y su retorno a IN1. El monitor serie a 115200 baud muestra `[RAW IN9-IN1]` y calcula `[CONST SUGERIDAS]` usando la media y el RMS de la senoide, que son mas resistentes al ruido que tomar solamente dos muestras extremas. Las constantes se sustituyen en el codigo despues de observar varias ventanas estables; posteriormente se cambia `CALIBRATION_MODE=false` para habilitar MQTT.
