/*
 * app_config.h
 * Configuracion general del balancin: pines, muestreo, PWM, calibracion y PID.
 * Todo lo que se ajusta en el laboratorio esta aqui.
 */
#pragma once
#include <Arduino.h>

// ---------------- Pines (ajustalos a tu cableado) ----------------
#define PIN_POT        4      // Debe ser ADC1: GPIO1..GPIO10 en el S3
#define PIN_ESCL       5      // Senal ESC izquierdo
#define PIN_ESCR       6      // Senal ESC derecho

// ---------------- Serial ----------------
#define SERIAL_BAUD    921600

// ---------------- Muestreo / experimento ----------------
#define TS             0.02f  // [s] 50 Hz (teoria)
#define TS_US          20000UL
#define TEXP           60.0f  // [s] duracion de experimentos
#define TSWITCH        0.4f   // [s] cambio de la PRBS
#define A_US           50     // [us] amplitud de u_delta
#define SEED_IDENT     1      // semilla PRBS (identificacion)
#define SEED_VALID     2      // semilla multinivel (validacion)
// Experimento MISO: una PRBS independiente para cada motor
#define A_MISO_US      25     // [us] amplitud por motor (uR-uL queda en +/-50 como maximo)
#define SEED_MISO_L    3      // semilla PRBS motor izquierdo
#define SEED_MISO_R    4      // semilla PRBS motor derecho

// ---------------- PWM ESC ----------------
#define ESC_FREQ_HZ    50
#define PWM_MIN        1000
#define PWM_MAX        2000
#define PWM_SAFE_LO    1100   // saturacion durante experimentos/control
#define PWM_SAFE_HI    1800
#define PWM_L0_DEF     1200   // punto de operacion por defecto (ajustar con 'o' y 'p')
#define PWM_R0_DEF     1250
#define ARM_TIME_MS    3000

// ---------------- Potenciometro (calibracion de 2 puntos) ----------------
// Coloca el balancin en dos angulos conocidos, anota el ADC del modo 'c'
// y pon aqui los valores.
#define ADC_A          1800.0f
#define TH_A           (-20.0f)
#define ADC_B          2300.0f
#define TH_B           20.0f
#define N_OVERSAMPLE   32     // lecturas promediadas por muestra
#define BETA           0.85f  // filtro de la derivada (teoria)
#define THETA_LIMIT    35.0f  // [deg] corte de seguridad

// ---------------- Controlador PID ----------------
// Ganancias de ARRANQUE: se sintonizan con el modelo identificado.
#define KP_DEF         2.0f   // [us/deg]
#define KI_DEF         0.5f   // [us/(deg*s)]
#define KD_DEF         0.8f   // [us/(deg/s)]
#define U_MAX          150.0f // [us] limite de u_delta del controlador
#define CTRL_SIGNO     1.0f   // si al subir u_delta el angulo BAJA, pon -1.0f
