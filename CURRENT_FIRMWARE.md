# Firmware actual: adquisicion ADS1263 y publicacion MQTT

El firmware operativo nuevo esta en:

```text
firmware/SCADA_ADC_MQTT/SCADA_ADC_MQTT.ino
```

La ruta de datos es:

```text
SCT-013 -> burden y acondicionamiento -> ADS1263 IN1-IN0
Sensor de voltaje aislado -> ADS1263 IN3-IN2
ADS1263 -> ESP32 -> MQTT -> servidor Ubuntu
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
- `bias_v`: media de la tension diferencial IN1-IN0, conservada con ese nombre por compatibilidad del mensaje.
- `frecuencia_hz`: frecuencia detectada.
- `valido`: indica que no existen rieles, referencia flotante ni frecuencia incompatible.
- `cargando`: corriente valida igual o superior a 1 A.
- `voltaje_sensor_rms_v`: RMS de la salida acondicionada medida en IN3-IN2.
- `voltaje_rms_v`: RMS de red; permanece `null` hasta calibrar el sensor.
- `voltaje_bias_v`, `voltaje_vpp_v`, `voltaje_frecuencia_hz`: diagnostico del canal de voltaje.
- `voltaje_valido`: indica que IN3-IN2 no esta saturado y su frecuencia es compatible.
- `voltaje_calibrado`: confirma si ya se habilito la conversion a voltios de red.
- `secuencia`: contador monotono desde el ultimo arranque.
- `timestamp_unix`: cero hasta que NTP sincroniza.

## Independencia del broker

El cliente implementa MQTT 3.1.1 estandar y no usa funciones de HiveMQ. El broker puede ser Mosquitto, EMQX, HiveMQ u otro compatible cambiando solamente `secrets.h`.

Las publicaciones periodicas usan QoS 0 sobre TCP para mantener una sesion estable y evitar bloquear la adquisicion esperando confirmaciones. Durante una interrupcion de WiFi o del broker conserva las ultimas 16 publicaciones en RAM; al reconectarse intenta enviarlas en orden. Si la cola se llena, elimina primero la mas antigua y aumenta `descartados`.

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

La corriente se mide diferencialmente en `IN1-IN0`. El voltaje acondicionado se mide en `IN3-IN2`, con IN3 positivo e IN2 negativo. Una tension diferencial negativa es valida y no se interpreta como saturacion. El firmware conserva la medicion de AVDD-AVSS para determinar su escala real. Nunca se conecta la red electrica directamente a una entrada del ADS1263: el canal IN3-IN2 requiere aislamiento y acondicionamiento.

## Calibracion del canal de voltaje

La ganancia de un modulo de voltaje ajustable depende del circuito y de la posicion de su potenciometro. La version inicial publica `voltaje_sensor_rms_v`, pero mantiene `voltaje_rms_v=null` y `voltaje_calibrado=false` para no reportar un valor de red inventado.

Con una fuente AC segura o durante una ventana de trabajo autorizada, se compara `voltaje_sensor_rms_v` contra un multimetro True RMS. Entonces se configura:

```text
VOLTAGE_MAINS_PER_SENSOR_V = voltaje_referencia_rms / voltaje_sensor_rms_v
VOLTAGE_SENSOR_CALIBRATED = true
```

## Recalibracion IN1-IN0

Para repetir la calibracion se cambia temporalmente a `CALIBRATION_MODE=true`. En ese modo desactiva WiFi/MQTT y espera en el generador:

```text
Sine, 60 Hz, High=3.000 V, Low=1.500 V, Phase=0
```

Conecte la salida del generador a IN1 y su retorno a IN0. El monitor serie a 115200 baud muestra `[RAW IN1-IN0]` y calcula `[CONST SUGERIDAS]` usando la media y el RMS de la senoide, que son mas resistentes al ruido que tomar solamente dos muestras extremas. Las constantes se sustituyen en el codigo despues de observar varias ventanas estables; posteriormente se cambia `CALIBRATION_MODE=false` para habilitar MQTT.

La calibracion obtenida despues de corregir la polaridad fisica es:

```text
INPUT_CAL_GAIN=0.9412289
INPUT_CAL_OFFSET_V=-0.0512069 V
```

Estas constantes provienen de 21 ventanas estables. Su promedio crudo fue media=2.444896 V, AC_RMS=0.563444 V y Vpp=1.595965 V; despues de calibrar se obtiene media=2.250000 V, AC_RMS=0.530330 V y Vpp=1.502168 V. El modo de calibracion tambien verifica la relacion Vpp/RMS de la senoide y rechaza ventanas de conexion, saturacion o deformacion antes de proponer constantes. La version operacional usa `CALIBRATION_MODE=false` para conectarse a WiFi y publicar por MQTT.
