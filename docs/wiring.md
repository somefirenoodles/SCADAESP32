# Cableado ESP32 + TTL-RS485

## UART del ESP32 al modulo RS485

| ESP32 | Modulo TTL-RS485 |
|---|---|
| GPIO17 / TX2 | RXD |
| GPIO16 / RX2 | TXD |
| GND | GND |
| 3V3 o 5V | VCC |

Usa 3V3 si el modulo funciona estable a 3.3 V. Si el modulo solo funciona a 5 V, no conectes TXD del modulo directo al GPIO16 sin adaptacion de nivel.

## Bus RS485

| Modulo TTL-RS485 | Dispositivo RS485 |
|---|---|
| A | A / D+ / RS485+ |
| B | B / D- / RS485- |
| GND opcional | GND comun |

Si no hay respuesta, intercambia A y B.

## Prueba esperada

Con A/B sin conectar, el firmware puede mostrar `timeout` o `CRC invalido`. Eso no es fallo de firmware; significa que no hay esclavo Modbus RTU respondiendo o que el bus esta flotando.

Para lectura valida se necesita un dispositivo Modbus RTU real: medidor electrico RS485, sensor RS485, adaptador USB-RS485 con software esclavo, o segundo microcontrolador actuando como esclavo.

## No conectar al cargador FoxESS CAN

No conectes el MAX485 a terminales marcados `CANH` y `CANL`. CAN y RS485 no son equivalentes. Para CAN se requiere transceptor CAN y protocolo CAN del fabricante.
