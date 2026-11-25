
#pragma once

struct PID {
  float kp, ki, kd;
  float uMax;
  float integ;
};

void pidInit(PID &pid, float kp, float ki, float kd, float uMax);

void pidReset(PID &pid);

void pidSetGains(PID &pid, float kp, float ki, float kd);


float pidUpdate(PID &pid, float ref, float theta, float thetaDot, float ts);
