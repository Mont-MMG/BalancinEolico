
#pragma once
#include <Arduino.h>

#define PIN_POT        4     
#define PIN_ESCL     5   // ESC izquierdo
#define PIN_ESCR       6      //  ESC derecho


#define SERIAL_BAUD    115200

// Muestreo y exper
#define TS             0.02f  
#define TS_US          20000UL
#define TEXP           60.0f  // [s] duracion de experimentos
#define SETTLE_S       3.0f   // [s] estabilizacion en el punto de operacion antes de excitar (no se graba)
#define TSWITCH        0.4f   // [s] cambio de la PRBS
#define A_US           75     // [us] amplitud de u_delta (50 casi no movia; 100 llegaba al corte de 35 deg)
#define SEED_IDENT     1      // semilla PRBS (identificacion)
#define SEED_VALID     2     
#define A_MISO_US      25     // [us] amplitud por motor (uR-uL queda en +/-50 como maximo)
#define SEED_MISO_L    3      // semilla PRBS motor izquierdo
#define SEED_MISO_R    4      // semilla PRBS motor derecho

//  PWM ESC
#define ESC_FREQ_HZ    50
#define PWM_MIN        1000
#define PWM_MAX        1600
#define PWM_SAFE_LO    1100   // saturacion durante experimentos/control
#define PWM_SAFE_HI    1800
// Punto de operacion: 19/nov era 1180/1232; 22/nov (ESC reconectado + moneda a la derecha) = 1205/1200
#define PWM_L0_DEF     1205   // punto de operacion por defecto (ajustar con 'o' y 'p')
#define PWM_R0_DEF     1200
#define ARM_TIME_MS    3000

//  Potenciometro (calibracion de 2 puntos)

// Calibracion 17/nov: tope izq 1640, +35 = 1599, 0 = 1558, -35 = 1510, tope der 1475
// Pendiente 1.27 cuentas/grado (~0.79 grados por cuenta), centrada en 0 = 1558.3
// 21/nov: el cero se mueve (1558 -> 1546 -> 1541): por defecto 0 deg = 1541.
// Antes de cada prueba: balancin horizontal + comando 'z' para fijar el cero real.
#define ADC_A          1496.6f
#define TH_A           (-35.0f)
#define ADC_B          1585.4f
#define TH_B           35.0f
#define THETA_SIGNO    (-1.0f) // 22/nov: izquierda = negativo, derecha = positivo (como se ve el balancin)
#define N_OVERSAMPLE   32    
#define BETA           0.85f 
#define THETA_LIMIT    35.0f  
#define RAMPA_DEG_S    10.0f  // [deg/s] arranque suave del control: la referencia va del angulo inicial a la ref
#define ERR_LIMIT      45.0f  // [deg] en control: STOP si |ref - theta| supera esto durante ERR_TIEMPO
#define ERR_TIEMPO     1.5f   // [s] tiempo seguido fuera de ERR_LIMIT antes de cortar (tolera perturbaciones)

// Controlador PID
// Ganancias de ARRANQUE: se sintonizan con el modelo identificado.
// Ganancias: 19/nov 6/6/0.5 (ARMAX). 22/nov 4/4/1.5: robustas a +300 ms de retardo extra (la prueba real oscilaba con 6/6/0.5)
#define KP_DEF         4.0f   
#define KI_DEF         4.0f  
#define KD_DEF         1.5f   
#define U_MAX          150.0f 
#define CTRL_SIGNO     -1.0f   
