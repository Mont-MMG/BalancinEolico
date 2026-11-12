# Balancín eólico – Identificación de sistemas dinámicos

Balancín de 1 GDL con dos motores brushless A2212 1000KV, ESC de 30 A y potenciómetro como sensor de ángulo. Microcontrolador: ESP32-S3 (PlatformIO + Arduino).

## Estructura
```
include/app_config.h   pines, muestreo, PWM, calibración y PID (todo lo ajustable)
include/control.h      interfaz del PID
src/control.cpp        PID (derivada sobre la medición + anti-windup)
src/main.cpp           sensor, ESC, señales de excitación, comandos y lazo principal
logger.py              guarda en CSV lo que manda el ESP32 (pip install pyserial)
matlab/ident_balancin.m  identificación ARX / ARMAX / OE, diferencial y MISO
```

## Comandos por serial
| Comando | Acción |
|---|---|
| `c` | calibración del potenciómetro (motores en mínimo) |
| `a` | armar ESC |
| `o` | punto de operación (buscar θ ≈ 0) |
| `p L R` | cambiar punto de operación, ej. `p 1200 1250` |
| `e` | Exp. 1: PRBS diferencial (identificación) |
| `m` | Exp. 2: MISO, una PRBS por motor |
| `v` | Exp. 3: señal multinivel (validación) |
| `r X` | referencia del PID en grados |
| `k P I D` | ganancias del PID |
| `g` | control PID en lazo cerrado |
| `s` | STOP |

CSV: `t,pwmL,pwmR,adcPot,theta,thetaDot` (en control se agregan `ref,u`).

## Orden de pruebas
1. Calibrar potenciómetro (`c`, sin hélices) y poner valores en `app_config.h`.
2. Encontrar punto de operación (`o` + `p`) y guardarlo en `app_config.h`.
3. `python logger.py COM5 e identificacion_diferencial.csv`
4. `python logger.py COM5 m identificacion_miso.csv`
5. `python logger.py COM5 v validacion.csv`
6. Correr `matlab/ident_balancin.m` y elegir el modelo con mejor FIT en validación.
7. Diseñar el PID con el modelo y probar con `k`, `r`, `g`.
