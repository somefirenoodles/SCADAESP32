# Testing - Pin configuration RS485

Este folder contiene codigo de prueba para validar que la configuracion de pines del ESP32 y los modulos TTL-RS485 esten correctos antes de conectar el sistema al cargador FoxESS.

## Archivo

- `pin_config_test.c`: firmware temporal para probar comunicacion entre dos modulos TTL-RS485 usando dos UART del mismo ESP32.

## Objetivo de la prueba

Validar que estos pines funcionen correctamente:

| Funcion | ESP32 | Modulo TTL-RS485 |
|---|---:|---|
| UART maestro TX | GPIO17 | RXD modulo A |
| UART maestro RX | GPIO16 | TXD modulo A |
| UART prueba TX | GPIO26 | RXD modulo B |
| UART prueba RX | GPIO27 | TXD modulo B |
| Alimentacion | 3V3 o 5V | VCC de ambos modulos |
| Tierra comun | GND | GND de ambos modulos |

## Conexion entre modulos

Despues de alimentar ambos modulos y conectar TX/RX:

| Modulo A | Modulo B |
|---|---|
| A | A |
| B | B |

Si no hay lectura, invertir solo A/B en uno de los modulos:

| Modulo A | Modulo B |
|---|---|
| A | B |
| B | A |

## Uso

1. Respaldar el firmware principal:

```powershell
copy main\my_esp32_project.c main\my_esp32_project.backup.c
```

2. Copiar el test como firmware principal temporal:

```powershell
copy testing\pin_config_test.c main\my_esp32_project.c
```

3. Compilar, grabar y abrir monitor:

```powershell
idf.py build flash monitor
```

4. Resultado esperado:

```text
Resultado: PASS - bytes recibidos correctamente.
```

5. Restaurar el firmware principal:

```powershell
copy main\my_esp32_project.backup.c main\my_esp32_project.c
```

## Interpretacion de resultados

### PASS

La configuracion de pines, TX/RX, alimentacion, GND comun y enlace A/B funcionan correctamente.

### FAIL - timeout

No se recibio ningun byte. Revisar:

- VCC de ambos modulos.
- GND comun entre ESP32 y ambos modulos.
- TX/RX cruzados correctamente.
- A/B conectados entre ambos modulos.
- A/B invertidos.

### FAIL - bytes recibidos no coinciden

Hay comunicacion electrica, pero los datos llegan corruptos. Revisar:

- A/B invertidos.
- Alimentacion inestable.
- Ruido o cables flojos.
- Baudrate distinto.

## Nota tecnica

Los modulos TTL-RS485 no responden solos. Esta prueba usa dos UART del mismo ESP32: un lado transmite y el otro lado recibe. Por eso permite aislar si el problema esta en el cableado, pines, modulos RS485 o firmware base antes de volver a probar con el cargador.
