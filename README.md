# Balancín eólico de dos motores: identificación y control PID

Identificación de sistemas dinámicos · UNAQ

Este repositorio contiene el firmware del ESP32-S3, el registro de datos y los scripts de MATLAB usados para **identificar el modelo dinámico** de un balancín eólico de 1 grado de libertad y **diseñar un controlador PID** a partir del modelo identificado.

---

## Contenido

1. [Descripción del sistema](#1-descripción-del-sistema)
2. [Modelo físico y linealización](#2-modelo-físico-y-linealización)
3. [Firmware](#3-firmware)
4. [Señal de excitación PRBS](#4-señal-de-excitación-prbs)
5. [Experimento de identificación](#5-experimento-de-identificación)
6. [Identificación del modelo](#6-identificación-del-modelo)
7. [Diseño del controlador PID](#7-diseño-del-controlador-pid)
8. [Resultados del control](#8-resultados-del-control)
9. [Cómo reproducir](#9-cómo-reproducir)
10. [Estructura del repositorio](#10-estructura-del-repositorio)
11. [Conclusiones](#11-conclusiones)

---

## 1. Descripción del sistema

El balancín es una barra que gira sobre un eje central, con un motor y una hélice en cada extremo. La diferencia de empuje entre los motores produce un torque que inclina la barra. El ángulo se mide con un potenciómetro acoplado al eje.

| Componente | Modelo / valor |
|---|---|
| Microcontrolador | ESP32-S3 (PlatformIO + framework Arduino) |
| Motores | Brushless A2212/13T, 1000 KV |
| Controladores de velocidad | ESC de 30 A, pulso de 1000 a 2000 µs a 50 Hz |
| Sensor de ángulo | Potenciómetro de 10 vueltas en el ADC1 (12 bits) |
| Pines | Potenciómetro: GPIO4 · ESC izquierdo: GPIO5 · ESC derecho: GPIO6 |

**Convención de signos:** θ es positivo cuando el balancín se inclina hacia la derecha y negativo hacia la izquierda.

---

## 2. Modelo físico y linealización

La ecuación de movimiento del balancín es:

```
J·θ̈ + b·θ̇ + τ_g(θ) = l_R·T_R − l_L·T_L
```

- `J·θ̈`: inercia de la barra
- `b·θ̇`: fricción en el eje
- `τ_g(θ) = m·g·L·sen θ`: torque de la gravedad
- `l·T`: torque de cada motor

La gravedad y el empuje de las hélices son **no lineales**. Alrededor del equilibrio (θ ≈ 0) se linealizan:

- `sen θ ≈ θ`, por lo que `τ_g ≈ k_g·θ`
- Cerca del punto de operación, el torque es proporcional a la entrada diferencial: `τ ≈ K_u·u_Δ`

Con esto queda un sistema de **segundo orden**:

```
G(s) = Θ(s) / U_Δ(s) = K_u / (J·s² + b·s + k_g)
```

Por eso todo el trabajo se hace **en desviación respecto al punto de operación** y con señales pequeñas.

### Punto de operación y entradas

| Variable | Definición |
|---|---|
| Punto de operación | `PWM_L0 = 1205 µs`, `PWM_R0 = 1200 µs` (balancín horizontal) |
| Desviaciones | `u_L = PWM_L − PWM_L0`, `u_R = PWM_R − PWM_R0` |
| Entrada diferencial | `u_Δ = u_R − u_L` (produce torque) |
| Entrada común | `u_Σ = u_R + u_L` (no produce torque; en el experimento vale 0) |

Los motores se mueven en espejo:

```
PWM_L = PWM_L0 − u_Δ/2
PWM_R = PWM_R0 + u_Δ/2
```

---

## 3. Firmware

El firmware ejecuta un ciclo fijo cada **Ts = 20 ms (50 Hz)**:

1. **Medir:** lee el potenciómetro (promedio de 32 lecturas), lo convierte a grados y calcula la velocidad angular con un filtro de primer orden (β = 0.85).
2. **Calcular el mando:** según el modo activo (PRBS, validación o PID).
3. **Seguridad:** detiene los motores si el ángulo se sale de rango o si el potenciómetro se desconecta.
4. **Aplicar:** envía los PWM a los ESC.
5. **Registrar:** envía por serial una línea CSV con los PWM **realmente aplicados**.

### Archivos

| Archivo | Contenido |
|---|---|
| `include/app_config.h` | Pines, muestreo, amplitud de la PRBS, punto de operación, calibración y ganancias del PID |
| `src/main.cpp` | Lectura del sensor, generación de la PRBS, modos de operación, comandos y ciclo principal |
| `src/control.cpp`, `include/control.h` | Controlador PID (derivada sobre la medición y antiwindup) |
| `monitor_csv.py` | Monitor serial en la PC: envía comandos y guarda cada prueba en un CSV dentro de `datos/` |

### Comandos por serial

| Comando | Acción |
|---|---|
| `z` | Fija el cero del ángulo (balancín horizontal) |
| `l` | Lectura única del ángulo (promedio de 1 s) |
| `a` | Arma los ESC |
| `o` | Motores en el punto de operación |
| `p L R` | Cambia el punto de operación, por ejemplo `p 1205 1200` |
| `e` | **Experimento de identificación con PRBS** (60 s) |
| `v` | Experimento de validación con señal multinivel (60 s) |
| `r X` | Referencia del control en grados, por ejemplo `r 10` |
| `k P I D` | Ganancias del PID, por ejemplo `k 4 4 1.5` |
| `g` | Arranca el control PID |
| `s` | Detiene los motores |

Formato del CSV:

```
Identificación:  t, pwmL, pwmR, adcPot, theta, thetaDot
Control:         t, pwmL, pwmR, adcPot, theta, thetaDot, ref, u
```

---

## 4. Señal de excitación PRBS

Para identificar el sistema se aplica una **PRBS** (*Pseudo-Random Binary Sequence*) en la entrada diferencial. Es una señal de dos niveles que cambia de forma pseudoaleatoria.

**Generación en el ESP32:**

```cpp
uint32_t rnd() { lcg = lcg * 1664525UL + 1013904223UL; return lcg >> 8; }   // generador congruencial lineal

uDelta = (rnd() & 1) ? A_US : -A_US;           // un bit: +A o −A
holdSamples = (int)roundf(TSWITCH / TS) - 1;   // se mantiene T_switch
```

1. Un generador congruencial lineal con **semilla 1** produce una secuencia que parece aleatoria pero **se puede repetir**.
2. En cada intervalo se toma un bit: 1 da `+A` y 0 da `−A`.
3. El valor se mantiene durante `T_switch`; después se toma un bit nuevo.

| Parámetro | Valor | Justificación |
|---|---|---|
| Amplitud A | ±75 µs | Con ±50 µs el balancín casi no vencía la fricción; con ±100 µs llegaba al corte de seguridad de 35° |
| T_switch | 0.4 s | El espectro cubre la dinámica del balancín (≈ 0.27 Hz); el primer nulo está en 1/T_switch = 2.5 Hz |
| Duración | 60 s | 3000 muestras a 50 Hz |

Propiedades: su autocorrelación cae a ~0 después de T_switch (se comporta como ruido blanco), por lo que excita muchas frecuencias a la vez y permite estimar los parámetros con buen condicionamiento.

---

## 5. Experimento de identificación

Archivo: [`datos/identificacion_diferencial_20261123_164432.csv`](datos/identificacion_diferencial_20261123_164432.csv)

| Dato | Valor |
|---|---|
| Muestras | 3001 |
| Periodo de muestreo | 0.020 s (50 Hz) |
| Duración | 60 s |
| Valores de u_Δ | ±74 µs (75/2 = 37 µs por motor en aritmética entera) |
| Valores de u_Σ | 0 µs |
| Rango del ángulo | −27.6° a +22.1°, sin tocar los topes mecánicos |

Antes de excitar, el firmware mantiene 3 s el punto de operación sin grabar, para que el balancín empiece cerca de 0°.

---

## 6. Identificación del modelo

Script: [`matlab/balancin_bj.m`](matlab/balancin_bj.m)

### Procedimiento

1. `u = pwmR − pwmL` y `y = theta`, en un objeto `iddata`.
2. 70 % de los datos para estimar y 30 % para validar.
3. `detrend` para quitar la media, que equivale a trabajar en desviación respecto al punto de operación.
4. Estimar y comparar cuatro estructuras con el mismo orden de planta (2,2) y retardo `nk = 5` (100 ms).
5. Validar con `compare` (FIT en los datos de validación) y `resid` (blancura de los residuos).

### Estructuras comparadas

| Modelo | Planta G | Ruido H | Familia |
|---|---|---|---|
| ARX(2,2,5) | B/A | 1/A | Error de ecuación |
| ARMAX(2,2,2,5) | B/A | C/A | Error de ecuación |
| OE(2,2,5) | B/F | 1 | Error de salida |
| BJ(2,2,2,2,5) | B/F | C/D | Error de salida |

Resultados aproximados en validación (los valores exactos los imprime `compare` al correr el script):

| Modelo | FIT en validación | Residuos | Comentario |
|---|---|---|---|
| ARX | ≈ 12 % | Correlacionados | Sesgado: polo en 0.998 y otro en −0.2 sin sentido físico |
| ARMAX | ≈ 50 % | Casi blancos | Mejora mucho al modelar el ruido con C |
| OE | Bajo | Correlacionados | El ruido no es blanco: tiene dinámica propia |
| **BJ** | **≈ 50 %** | **Blancos** | **Modelo elegido** |

**Por qué Box-Jenkins:** los residuos del OE muestran correlación temporal, o sea que la perturbación tiene memoria (fricción, vibración, cuantización del potenciómetro). BJ modela el ruido con su propia dinámica `C/D`, independiente de la planta `B/F`. Sus residuos quedan blancos y los polos del ruido (~0.876) son distintos de los de la planta (~0.98), así que la planta y la perturbación sí son fenómenos diferentes.

### Modelo identificado (Box-Jenkins)

```
B(z) = 0.0003452 z⁻⁵ − 0.0006412 z⁻⁶
F(z) = 1 − 1.96 z⁻¹ + 0.9611 z⁻²
C(z) = 1 − 1.191 z⁻¹ + 0.4143 z⁻²
D(z) = 1 − 1.752 z⁻¹ + 0.7554 z⁻²
```

**Planta:**

```
          0.0003452 z⁻⁵ − 0.0006412 z⁻⁶
G(z) = ---------------------------------      Ts = 0.02 s
            1 − 1.96 z⁻¹ + 0.9611 z⁻²
```

| Propiedad | Valor |
|---|---|
| Polos de la planta | 0.980 ± 0.0275j (dentro del círculo unitario: estable) |
| Frecuencia natural | ωn ≈ 1.72 rad/s |
| Amortiguamiento | ζ ≈ 0.58 (subamortiguado) |
| Retardo | 5 muestras = 100 ms |

El resultado es coherente con el modelo físico: un sistema de segundo orden subamortiguado, como un péndulo con fricción.

---

## 7. Diseño del controlador PID

Las ganancias se calculan con `pidtune` sobre la planta BJ:

```matlab
P   = -Gd;                                   % ver nota de signo
C   = pidtune(P, 'PIDF', 1.5, pidtuneOptions('PhaseMargin', 70));
```

| Criterio | Valor |
|---|---|
| Ancho de banda del lazo | 1.5 rad/s |
| Margen de fase | 70° |

**Nota de signo:** con la convención del balancín, al darle más PWM al motor derecho el ángulo baja, así que la planta tiene ganancia negativa. Se sintoniza sobre `P = −G` y en el firmware se compensa con `CTRL_SIGNO = −1`.

### Ley de control en el ESP32

```
u = Kp·e + Ki·∫e dt − Kd·θ̇
```

- **Derivada sobre la medición** (θ̇ filtrada), para evitar picos al cambiar la referencia.
- **Antiwindup:** la integral solo acumula cuando la salida no está saturada.
- **Saturación:** |u_Δ| ≤ 150 µs.
- **Arranque suave:** la referencia va del ángulo inicial a la deseada a 10 °/s.

### Ganancias finales

| Kp | Ki | Kd |
|---|---|---|
| 4.0 µs/° | 4.0 µs/(°·s) | 1.5 µs/(°/s) |

Las ganancias que entrega `pidtune` con el modelo BJ son muy parecidas a estas. Las finales se ajustaron y comprobaron en el balancín real.

---

## 8. Resultados del control

Archivo: [`datos/control_pid_20261123_174624.csv`](datos/control_pid_20261123_174624.csv) · Script: [`matlab/graficar_control.m`](matlab/graficar_control.m)

| Tramo | Referencia | Resultado |
|---|---|---|
| 0 – 32 s | 0° | Mantiene 0°. Se aplicaron dos perturbaciones fuertes (hasta −62°) y en ambas regresó a la referencia en unos 6–8 s |
| 32 – 49 s | +10° | Sigue la referencia y rechaza otra perturbación |
| 49 – 67 s | −30° | Sigue la referencia; tras una perturbación de +63° regresa a −29° |

El controlador sigue cambios de referencia y rechaza perturbaciones grandes. Durante los empujones el mando llega a su límite, que es el control usando todo el esfuerzo disponible para recuperar el balancín.

---

## 9. Cómo reproducir

### Requisitos

- VS Code con la extensión PlatformIO
- Python con `pyserial` (ya viene incluido en PlatformIO)
- MATLAB con System Identification Toolbox y Control System Toolbox

### Pasos

1. **Cargar el firmware:** abrir la carpeta en VS Code y dar **Upload** en PlatformIO.
2. **Abrir el monitor serial** en la terminal de PlatformIO:
   ```
   python monitor_csv.py COM14
   ```
   Cambiar `COM14` por el puerto de la placa (`pio device list`).
3. **Calibrar y armar:** balancín horizontal → `z` → conectar la batería → `a`.
4. **Experimento de identificación:** `e` y esperar `# FIN del experimento`. El CSV se guarda en `datos/`.
5. **Identificación y ganancias:** en MATLAB, con la carpeta `matlab/` como *Current Folder*, correr `balancin_bj.m` (actualizar el nombre del CSV en `readtable` si se graba uno nuevo).
6. **Control:** `r 0` → `g` → `k Kp Ki Kd` → cambiar la referencia con `r 10`, `r -10` → `s` para terminar.
7. **Gráfica del control:** correr `graficar_control.m`.

---

## 10. Estructura del repositorio

```
balancin_esp32s3/
├── platformio.ini                 configuración de PlatformIO (ESP32-S3)
├── include/
│   ├── app_config.h               parámetros del sistema
│   └── control.h                  interfaz del PID
├── src/
│   ├── main.cpp                   firmware principal
│   └── control.cpp                PID
├── monitor_csv.py                 monitor serial y registro en CSV
├── datos/
│   ├── identificacion_diferencial_20261123_164432.csv   experimento con PRBS
│   └── control_pid_20261123_174624.csv                  prueba del control
└── matlab/
    ├── balancin_bj.m              identificación (ARX, ARMAX, OE, BJ) y ganancias del PID
    └── graficar_control.m         gráfica de la prueba de control
```

---

## 11. Conclusiones

- La PRBS de ±75 µs con T_switch = 0.4 s excitó el balancín de forma suficiente y dentro de la zona de operación segura.
- El ARX resultó sesgado porque supone que el ruido comparte la dinámica de la planta. ARMAX y BJ mejoraron el ajuste al modelar el ruido.
- Se eligió **Box-Jenkins** porque separa la dinámica de la planta (B/F) de la del ruido (C/D) y deja residuos blancos.
- La planta identificada es de segundo orden subamortiguada (ωn ≈ 1.72 rad/s, ζ ≈ 0.58), coherente con el modelo físico del péndulo con fricción.
- El PID diseñado a partir del modelo mantiene la referencia, sigue sus cambios y rechaza perturbaciones en el balancín real.
- El FIT está limitado por no linealidades que un modelo lineal no captura: fricción estática en el eje, resolución del potenciómetro (~0.8° por cuenta) y empuje no lineal de las hélices.
