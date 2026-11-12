/*
 * control.cpp
 * Implementacion del PID:
 *   u = Kp*e + Ki*integral(e) - Kd*thetaDot
 */
#include "control.h"
#include <Arduino.h>

void pidInit(PID &pid, float kp, float ki, float kd, float uMax) {
  pid.kp = kp; pid.ki = ki; pid.kd = kd;
  pid.uMax = uMax;
  pid.integ = 0.0f;
}

void pidReset(PID &pid) { pid.integ = 0.0f; }

void pidSetGains(PID &pid, float kp, float ki, float kd) {
  pid.kp = kp; pid.ki = ki; pid.kd = kd;
}

float pidUpdate(PID &pid, float ref, float theta, float thetaDot, float ts) {
  float e  = ref - theta;
  float uP = pid.kp * e;
  float uD = -pid.kd * thetaDot;        // derivada sobre la medicion (sin "patada")

  // Anti-windup: solo integra si no satura, o si integrar ayuda a salir de saturacion
  float uTry = uP + pid.ki * (pid.integ + e * ts) + uD;
  if (fabsf(uTry) < pid.uMax || (uTry * e) < 0.0f) {
    pid.integ += e * ts;
  }

  float u = uP + pid.ki * pid.integ + uD;
  return constrain(u, -pid.uMax, pid.uMax);
}
