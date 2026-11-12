/*
 * control.h
 * Controlador PID del angulo del balancin.
 * Salida: u_delta [us] (diferencia de mando entre motores).
 */
#pragma once

struct PID {
  float kp, ki, kd;
  float uMax;
  float integ;
};

// Inicializa ganancias y limite
void pidInit(PID &pid, float kp, float ki, float kd, float uMax);

// Borra el integrador (llamar al iniciar el lazo)
void pidReset(PID &pid);

// Cambia ganancias en caliente
void pidSetGains(PID &pid, float kp, float ki, float kd);

// Calcula u_delta [us].
//   ref, theta en [deg]; thetaDot en [deg/s] (ya filtrada); ts en [s]
// Derivada sobre la medicion y anti-windup por integracion condicional.
float pidUpdate(PID &pid, float ref, float theta, float thetaDot, float ts);
