# SCADAESP32

Firmware ESP-IDF para validar una interfaz ESP32 + TTL-RS485 y dejar listo un maestro Modbus RTU para leer un dispositivo RS485 real.

El objetivo inmediato es que el ESP32 envie solicitudes Modbus RTU por UART2 y descarte ruido mediante validacion de trama y CRC16. Con un esclavo Modbus real conectado a `A/B`, el firmware imprime registros leidos en consola.

## Estado del proyecto

Implementado:

- Proyecto ESP-IDF completo.
- UART2 en `GPIO17` TX y `GPIO16` RX.
- Maestro Modbus RTU.
- Lectura por funcion `0x03` o `0x04`.
- Validacion de `slave_id`, `function_code`, longitud y `CRC16`.
- Configuracion por `idf.py menuconfig`.
- Scripts PowerShell para build, flash, monitor y limpieza.

No implementado todavia:

- MQTT.
- Telegraf / TimescaleDB.
- Dashboard Superset.
- Lectura directa del cargador FoxESS por CAN o WiFi.

## Cableado base

| ESP32 | Modulo TTL-RS485 |
|---|---|
| GPIO17 / TX2 | RXD |
| GPIO16 / RX2 | TXD |
| GND | GND |
| 3V3 o 5V | VCC |

| Modulo TTL-RS485 | Dispositivo RS485 |
|---|---|
| A | A / D+ / RS485+ |
| B | B / D- / RS485- |

Nota tecnica: si el modulo esta alimentado a 5 V, verifica que `TXD` del modulo no entregue 5 V al `GPIO16` del ESP32. El ESP32 usa logica de 3.3 V.

## Build

Desde ESP-IDF PowerShell:

```powershell
idf.py build
```

O con script:

```powershell
.\scripts\build.ps1
```

## Flash + monitor

```powershell
idf.py -p COM17 flash monitor
```

O con script:

```powershell
.\scripts\flash_monitor.ps1 -Port COM17
```

Salir del monitor:

```text
Ctrl + ]
```

## Configuracion

Abrir menuconfig:

```powershell
idf.py menuconfig
```

Ruta:

```text
SCADAESP32 Modbus RTU
```

Parametros configurables:

- `UART2 TX GPIO`: default `17`.
- `UART2 RX GPIO`: default `16`.
- `Modbus RTU baudrate`: default `9600`.
- `Parity`: `8N1`, `8E1` o `8O1`.
- `Slave ID`: default `1`.
- `Function code`: `0x03` holding registers o `0x04` input registers.
- `Start register`: default `0x0000`.
- `Register count`: default `2`.
- `Polling interval`: default `2000 ms`.

## Salida esperada sin dispositivo RS485

Con `A/B` desconectado, es normal ver:

```text
RX: timeout. No hay respuesta valida.
```

Tambien puede aparecer ruido y ser descartado como:

```text
Error: CRC invalido
```

Eso indica que RX recibe actividad electrica, pero no una trama Modbus valida.

## Salida esperada con esclavo Modbus RTU

```text
TX Modbus RTU: 01 03 00 00 00 02 C4 0B
RX bruto: 01 03 04 00 E6 00 00 9B 3A
Trama Modbus valida.
Registro 0x0000 = 230 / 0x00E6
Registro 0x0001 = 0 / 0x0000
Lectura OK.
```

## Diagnostico rapido

Si no hay respuesta:

1. Intercambia `A` y `B`.
2. Verifica `slave_id`.
3. Verifica baudrate.
4. Verifica paridad: `8N1`, `8E1` o `8O1`.
5. Verifica si el dispositivo usa funcion `0x03` o `0x04`.
6. Verifica la direccion inicial del registro.
7. Usa resistencia de 120 ohm entre `A` y `B` solo si corresponde a extremo de bus.

## Advertencia sobre FoxESS

No conectes este modulo MAX485 a terminales marcados `CANH` y `CANL`. CAN bus no es RS485. Para CAN se requiere un transceptor CAN y el protocolo CAN del fabricante.

El protocolo FoxESS EV Charger disponible es Modbus TCP por red IP. Este firmware es para Modbus RTU por RS485, util para medidores o dispositivos externos RS485.
