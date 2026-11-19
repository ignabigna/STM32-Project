# HMI Banco de Pruebas · BP_A_Master_F

Interfaz web de telemetría y control para el proyecto `BP_A_Master_F` (STM32F103 + CAN).

```
STM32 (RAM) ──SWD── ST-Link clon ── OpenOCD ─┬─ :3333 GDB ──► STM32CubeIDE (debug)
                                             └─ :6666 TCL ──► Puente Python ──WebSocket :8765──► HMI (este directorio)
```

## Uso

1. Abrí `index.html` en Chrome o Edge (doble clic; no hace falta un servidor).
2. Elegí la **fuente de datos**:
   - **Simulador (demo)**: emula el firmware, la placa B y el debugger. Sirve para probar la HMI sin hardware.
   - **Puente Python**: se conecta por WebSocket a `ws://localhost:8765`, configurable. Si se corta, reintenta solo.
3. ⚙ **Ajustes**: pulsos por vuelta (para ver vueltas, grados y RPM) y timeout de la alarma CAN.

Atajo: **Esc** = STOP (solo en modo REMOTO).

## Protocolo WebSocket (JSON)

### Puente → HMI

| `type` | Cuándo | Campos |
|---|---|---|
| `status` | Al conectar y cuando cambia algo | `openocd` (bool), `target`, `elf`, `rate_hz`, `vars_meta` (opcional) |
| `telemetry` | En cada muestreo (10–50 Hz) | `t` (ms), `target`, `vars` |
| `target` | Al cambiar el estado del núcleo | `state`: `running` \| `halted` \| `reset` \| `unknown` |
| `ack` | Respuesta a cada `write` | `id`, `ok` (bool), `error` (si falla) |
| `log` / `error` | Mensajes informativos | `message`, `level` (`info` \| `ok` \| `warn` \| `err`) |

```json
{"type":"status","openocd":true,"target":"running","elf":"BP_A_Master_F.elf","rate_hz":20,
 "vars_meta":{"live_position":{"type":"int32_t","addr":536870932,"writable":false,"desc":"..."},
              "system_mode":{"type":"OperatingMode_t","addr":536870920,"writable":true}}}

{"type":"telemetry","t":123456,"target":"running",
 "vars":{"live_position":-1520,"live_motor_state":1,"telemetry_counter":8812,
         "system_mode":1,"remote_cmd":0,"last_system_mode":1,"current_led_state":1}}

{"type":"ack","id":7,"ok":true}
```

### HMI → Puente

| `type` | Campos | Acción esperada en el puente |
|---|---|---|
| `hello` | `client`, `version` | Responder con `status` |
| `write` | `id`, `var`, `value` | Escribir la variable en RAM (con OpenOCD `write_memory`, usando dirección y tamaño del `.elf`) y responder con `ack` |

```json
{"type":"write","id":7,"var":"remote_cmd","value":1}
```

## Notas de comportamiento

- **Comandos** (`remote_cmd`): la HMI solo los envía en modo REMOTO. El comando se da por ejecutado cuando el `ack` es OK **y** una muestra posterior muestra `remote_cmd == 0`, es decir, cuando el firmware lo tomó.
- **Micro en pausa (`halted`)**: los valores quedan congelados, el gráfico deja de avanzar y los comandos aparecen como "pendientes" hasta que reanudes. Mientras está en pausa no corre el timeout del comando.
- **Alarma CAN**: si el micro está `running` y `telemetry_counter` no aumenta durante el timeout configurado, aparece "Sin telemetría CAN".
- **CSV**: se graba una fila por cada muestra recibida, con `;` como separador (se abre directo en Excel con configuración regional en español).
- El STOP de la HMI **no** reemplaza la parada física (BTN4).
