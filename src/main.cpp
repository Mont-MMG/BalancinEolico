/*
 * main.cpp
 * Balancin eolico de 2 motores - ESP32-S3
 * Sensor: potenciometro | Actuadores: 2 ESC 30A + A2212 1000KV
 *
 * CSV por serial:
 *   identificacion/validacion: t,pwmL,pwmR,adcPot,theta,thetaDot
 *   control:                   t,pwmL,pwmR,adcPot,theta,thetaDot,ref,u
 *
 * Comandos por serial (terminar con Enter):
 *   c        -> calibracion: motores en minimo, imprime ADC y angulo
 *   a        -> armar ESC
 *   o        -> punto de operacion (buscar theta ~ 0)
 *   p L R    -> cambia el punto de operacion, ej: "p 1200 1252"
 *   e        -> experimento 1: PRBS diferencial (identificacion)
 *   m        -> experimento 2: MISO, una PRBS independiente por motor
 *   v        -> experimento 3: senal multinivel (validacion)
 *   r X      -> referencia del controlador [deg], ej: "r 10"
 *   k P I D  -> ganancias del PID, ej: "k 2.0 0.5 0.8"
 *   g        -> control PID en lazo cerrado
 *   s        -> STOP
 */
#include <Arduino.h>
#include <ESP32Servo.h>
#include "app_config.h"
#include "control.h"

Servo escL, escR;
PID pid;

enum Mode { IDLE, CALIB, OPPOINT, EXP_PRBS, EXP_MISO, EXP_VALID, CONTROL };
Mode mode = IDLE;

int   pwmL0 = PWM_L0_DEF, pwmR0 = PWM_R0_DEF;
float thetaRef = 0.0f;

uint32_t tNext = 0, k = 0;
float thetaPrev = 0, thetaDotF = 0;
int   uDelta = 0, holdSamples = 0;

// ---------- Generador pseudoaleatorio reproducible (como rng(semilla)) ----------
uint32_t lcg = 1;
uint32_t rnd() { lcg = lcg * 1664525UL + 1013904223UL; return lcg >> 8; }

// Dos generadores independientes para el experimento MISO
uint32_t lcgL = SEED_MISO_L, lcgR = SEED_MISO_R;
int uMisoL = 0, uMisoR = 0, holdL = 0, holdR = 0;
bool rndBit(uint32_t &st) { st = st * 1664525UL + 1013904223UL; return (st >> 31) & 1; }  // bit alto: mas aleatorio

// ---------- Sensor ----------
int readPotRaw() {
  uint32_t acc = 0;
  for (int i = 0; i < N_OVERSAMPLE; i++) acc += analogRead(PIN_POT);
  return acc / N_OVERSAMPLE;
}

float adcToTheta(float adc) {
  return TH_A + (adc - ADC_A) * (TH_B - TH_A) / (ADC_B - ADC_A);
}

// ---------- Actuadores ----------
void writeMotors(int pl, int pr) {
  escL.writeMicroseconds(constrain(pl, PWM_MIN, PWM_MAX));
  escR.writeMicroseconds(constrain(pr, PWM_MIN, PWM_MAX));
}

void stopMotors() { mode = IDLE; writeMotors(PWM_MIN, PWM_MIN); }

// Reparte u_delta entre motores (u_sigma = 0)
void applyDelta(int u, int &pl, int &pr) {
  pl = constrain(pwmL0 - u / 2, PWM_SAFE_LO, PWM_SAFE_HI);
  pr = constrain(pwmR0 + u / 2, PWM_SAFE_LO, PWM_SAFE_HI);
}

// ---------- Senales de excitacion ----------
int nextInput() {
  if (holdSamples > 0) { holdSamples--; return uDelta; }
  if (mode == EXP_PRBS) {                         // PRBS +/-A
    uDelta = (rnd() & 1) ? A_US : -A_US;
    holdSamples = (int)roundf(TSWITCH / TS) - 1;
  } else if (mode == EXP_VALID) {                 // multinivel, 0.3..1.0 s
    const float lv[5] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};
    uDelta = (int)(lv[rnd() % 5] * A_US);
    float dur = 0.3f + (rnd() % 1000) / 1000.0f * 0.7f;
    holdSamples = (int)roundf(dur / TS) - 1;
  } else {
    uDelta = 0;
  }
  return uDelta;
}

// MISO: cada motor tiene su propia PRBS +/-A_MISO_US, sin relacion entre ellas
void nextMisoInputs(int &uL, int &uR) {
  const int hold = (int)roundf(TSWITCH / TS) - 1;
  if (holdL > 0) holdL--; else { uMisoL = rndBit(lcgL) ? A_MISO_US : -A_MISO_US; holdL = hold; }
  if (holdR > 0) holdR--; else { uMisoR = rndBit(lcgR) ? A_MISO_US : -A_MISO_US; holdR = hold; }
  uL = uMisoL; uR = uMisoR;
}

// ---------- Modos ----------
void startMode(Mode m, uint32_t seed) {
  mode = m; k = 0; holdSamples = 0; uDelta = 0; lcg = seed;
  lcgL = SEED_MISO_L; lcgR = SEED_MISO_R; holdL = holdR = 0; uMisoL = uMisoR = 0;
  thetaPrev = adcToTheta(readPotRaw()); thetaDotF = 0;
  pidReset(pid);
  tNext = micros();
  if (m == CONTROL) Serial.println("t,pwmL,pwmR,adcPot,theta,thetaDot,ref,u");
  else              Serial.println("t,pwmL,pwmR,adcPot,theta,thetaDot");
}

void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n'); cmd.trim();
  if (cmd.length() == 0) return;
  char c = cmd[0];

  if (c == 's') { stopMotors(); Serial.println("# STOP"); }
  else if (c == 'a') {
    Serial.println("# Armando ESC...");
    writeMotors(PWM_MIN, PWM_MIN); delay(ARM_TIME_MS);
    Serial.println("# ESC armados");
  }
  else if (c == 'c') startMode(CALIB, SEED_IDENT);
  else if (c == 'o') startMode(OPPOINT, SEED_IDENT);
  else if (c == 'e') startMode(EXP_PRBS, SEED_IDENT);
  else if (c == 'm') startMode(EXP_MISO, SEED_IDENT);
  else if (c == 'v') startMode(EXP_VALID, SEED_VALID);
  else if (c == 'g') startMode(CONTROL, SEED_IDENT);
  else if (c == 'p') {
    int l, r;
    if (sscanf(cmd.c_str(), "p %d %d", &l, &r) == 2) { pwmL0 = l; pwmR0 = r; }
    Serial.printf("# PWM_L0=%d PWM_R0=%d\n", pwmL0, pwmR0);
  }
  else if (c == 'r') {
    float r;
    if (sscanf(cmd.c_str(), "r %f", &r) == 1) thetaRef = r;
    Serial.printf("# ref=%.2f deg\n", thetaRef);
  }
  else if (c == 'k') {
    float p, i, d;
    if (sscanf(cmd.c_str(), "k %f %f %f", &p, &i, &d) == 3) pidSetGains(pid, p, i, d);
    Serial.printf("# Kp=%.3f Ki=%.3f Kd=%.3f\n", pid.kp, pid.ki, pid.kd);
  }
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(SERIAL_BAUD);
  analogReadResolution(12);                       // 0..4095
  analogSetPinAttenuation(PIN_POT, ADC_11db);     // rango ~0..3.1 V

  ESP32PWM::allocateTimer(0);
  escL.setPeriodHertz(ESC_FREQ_HZ); escR.setPeriodHertz(ESC_FREQ_HZ);
  escL.attach(PIN_ESCL, PWM_MIN, PWM_MAX);
  escR.attach(PIN_ESCR, PWM_MIN, PWM_MAX);
  writeMotors(PWM_MIN, PWM_MIN);

  pidInit(pid, KP_DEF, KI_DEF, KD_DEF, U_MAX);

  delay(1500);
  Serial.println("# Balancin listo. Comandos: c a o p e m v r k g s");
}

void loop() {
  handleSerial();
  if (mode == IDLE) return;
  if ((int32_t)(micros() - tNext) < 0) return;    // espera al siguiente periodo
  tNext += TS_US;

  // 1) Medir
  int adc = readPotRaw();
  float theta = adcToTheta(adc);
  float thetaDotRaw = (theta - thetaPrev) / TS;
  thetaDotF = BETA * thetaDotF + (1.0f - BETA) * thetaDotRaw;
  thetaPrev = theta;

  // 2) Calcular mando
  int pl = PWM_MIN, pr = PWM_MIN;
  float uCtrl = 0.0f;
  switch (mode) {
    case OPPOINT:   pl = pwmL0; pr = pwmR0; break;
    case EXP_PRBS:
    case EXP_VALID: applyDelta(nextInput(), pl, pr); break;
    case EXP_MISO: {
      int uL, uR;
      nextMisoInputs(uL, uR);
      pl = constrain(pwmL0 + uL, PWM_SAFE_LO, PWM_SAFE_HI);
      pr = constrain(pwmR0 + uR, PWM_SAFE_LO, PWM_SAFE_HI);
      break;
    }
    case CONTROL:
      uCtrl = pidUpdate(pid, thetaRef, theta, thetaDotF, TS);
      applyDelta((int)lroundf(CTRL_SIGNO * uCtrl), pl, pr);
      break;
    default: break;                               // CALIB: motores en minimo
  }

  // 3) Seguridad
  if (mode != CALIB && fabsf(theta) > THETA_LIMIT) {
    stopMotors(); Serial.println("# STOP: angulo fuera de limite"); return;
  }

  // 4) Aplicar (se registra lo REALMENTE aplicado)
  writeMotors(pl, pr);

  // 5) Registrar
  if (mode == CONTROL)
    Serial.printf("%.2f,%d,%d,%d,%.3f,%.3f,%.2f,%.1f\n",
                  k * TS, pl, pr, adc, theta, thetaDotF, thetaRef, uCtrl);
  else
    Serial.printf("%.2f,%d,%d,%d,%.3f,%.3f\n", k * TS, pl, pr, adc, theta, thetaDotF);

  k++;
  if ((mode == EXP_PRBS || mode == EXP_MISO || mode == EXP_VALID) && k * TS > TEXP) {
    stopMotors(); Serial.println("# FIN del experimento");
  }
}
