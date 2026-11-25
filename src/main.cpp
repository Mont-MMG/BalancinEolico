/*
 * Comandos por serial
 *   c calibracion: motores en minimo, imprime ADC y angulo (continuo)
 *   l lectura unica: promedio de 1 s del ADC y del angulo
 *   a armar ESC
 *   opunto de operacion (buscar theta ~ 0)
 *   p L R cambia el punto de operacion, ej: "p 1200 1252"
 *   eexperimento 1: PRBS diferencial (identificacion)
 *   m experimento 2: MISO, una PRBS independiente por motor
 *   vexperimento 3: senal multinivel (validacion)
 *   r X referencia del controlador [deg]
 *   k ganancias del PID
 * 
 *   g control PID en lazo cerrado
 *   s STOP
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
int settleSamples = 0;          
float refActual = 0.0f;        
bool  rampa = false;         
int   errSamples = 0;        
float thetaPrev = 0, thetaDotF = 0;
int   uDelta = 0, holdSamples = 0;

uint32_t lcg = 1;
uint32_t rnd() { lcg = lcg * 1664525UL + 1013904223UL; return lcg >> 8; }

uint32_t lcgL = SEED_MISO_L, lcgR = SEED_MISO_R;
int uMisoL = 0, uMisoR = 0, holdL = 0, holdR = 0;
bool rndBit(uint32_t &st) { st = st * 1664525UL + 1013904223UL; return (st >> 31) & 1; }  // bit alto: mas aleatorio

int readPotRaw() {
  uint32_t acc = 0;
  for (int i = 0; i < N_OVERSAMPLE; i++) acc += analogRead(PIN_POT);
  return acc / N_OVERSAMPLE;
}

// Pendiente fija de la calibracion y cero ajustable con el comando 'z'
const float PENDIENTE = (TH_B - TH_A) / (ADC_B - ADC_A);          // [deg/cuenta]
float adcCero = ADC_A + (0.0f - TH_A) / PENDIENTE;                // ADC que corresponde a 0 deg

float adcToTheta(float adc) {
  return THETA_SIGNO * (adc - adcCero) * PENDIENTE;
}

void writeMotors(int pl, int pr) {
  escL.writeMicroseconds(constrain(pl, PWM_MIN, PWM_MAX));
  escR.writeMicroseconds(constrain(pr, PWM_MIN, PWM_MAX));
}

void stopMotors() { mode = IDLE; writeMotors(PWM_MIN, PWM_MIN); }

void applyDelta(int u, int &pl, int &pr) {
  pl = constrain(pwmL0 - u / 2, PWM_SAFE_LO, PWM_SAFE_HI);
  pr = constrain(pwmR0 + u / 2, PWM_SAFE_LO, PWM_SAFE_HI);
}

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

void nextMisoInputs(int &uL, int &uR) {
  const int hold = (int)roundf(TSWITCH / TS) - 1;
  if (holdL > 0) holdL--; else { uMisoL = rndBit(lcgL) ? A_MISO_US : -A_MISO_US; holdL = hold; }
  if (holdR > 0) holdR--; else { uMisoR = rndBit(lcgR) ? A_MISO_US : -A_MISO_US; holdR = hold; }
  uL = uMisoL; uR = uMisoR;
}

void startMode(Mode m, uint32_t seed) {
  mode = m; k = 0; holdSamples = 0; uDelta = 0; lcg = seed;
  settleSamples = (m == EXP_PRBS || m == EXP_MISO || m == EXP_VALID) ? (int)(SETTLE_S / TS) : 0;
  if (settleSamples > 0) Serial.println("# Estabilizando en el punto de operacion...");
  lcgL = SEED_MISO_L; lcgR = SEED_MISO_R; holdL = holdR = 0; uMisoL = uMisoR = 0;
  thetaPrev = adcToTheta(readPotRaw()); thetaDotF = 0;
  pidReset(pid);
  tNext = micros();
  if (m == CONTROL) {
    refActual = thetaPrev;                        // la rampa empieza donde esta el balancin
    errSamples = 0;
    rampa = true;
    Serial.printf("# Inicia control PID: rampa de %.1f a %.1f deg\n", refActual, thetaRef);
    Serial.println("t,pwmL,pwmR,adcPot,theta,thetaDot,ref,u");
  }
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
  else if (c == 'z') {                            // fija el cero: balancin horizontal
    stopMotors();
    uint32_t acc = 0;
    for (int i = 0; i < 50; i++) { acc += readPotRaw(); delay(20); }
    float nuevo = acc / 50.0f;
    if (nuevo < 20 || nuevo > 4075) {
      Serial.printf("# ERROR: ADC = %.0f, el potenciometro parece desconectado. Cero sin cambios\n", nuevo);
    } else {
      Serial.printf("# Cero ajustado: antes ADC %.1f, ahora ADC %.1f = 0 deg (cambio de %.1f deg)\n",
                    adcCero, nuevo, THETA_SIGNO * (nuevo - adcCero) * PENDIENTE);
      adcCero = nuevo;
    }
  }
  else if (c == 'l') {                            // lectura unica promediada (1 s)
    stopMotors();
    uint32_t acc = 0; int amin = 4095, amax = 0;
    for (int i = 0; i < 50; i++) {
      int a = readPotRaw(); acc += a;
      amin = min(amin, a); amax = max(amax, a);
      delay(20);
    }
    float aProm = acc / 50.0f;
    Serial.printf("# ADC promedio = %.1f  (min %d, max %d)  ->  theta = %.2f deg\n",
                  aProm, amin, amax, adcToTheta(aProm));
  }
  else if (c == 'o') startMode(OPPOINT, SEED_IDENT);
  else if (c == 'e') startMode(s, SEED_IDENT);
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
  Serial.println("# Balancin listo. Comandos: c l z a o p e m v r k g s");
}

void loop() {
  handleSerial();
  if (mode == IDLE) return;
  if ((int32_t)(micros() - tNext) < 0) return;    // espera al siguiente periodo
  tNext += TS_US;

  // 1 Medir
  int adc = readPotRaw();
  float theta = adcToTheta(adc);
  float thetaDotRaw = (theta - thetaPrev) / TS;
  thetaDotF = BETA * thetaDotF + (1.0f - BETA) * thetaDotRaw;
  thetaPrev = theta;

  // 1.2 Sensor desconectado: el ADC pegado a 0 o a 4095 no es una lectura real
  if (mode != CALIB && (adc < 20 || adc > 4075)) {
    stopMotors();
    Serial.printf("# STOP: potenciometro desconectado (ADC = %d). Revisa sus cables 3.3V, GND y GPIO%d\n", adc, PIN_POT);
    return;
  }

  // 1.5 Estabilizacion: motores en el punto de operacion, sin grabar ni cortar
  if (settleSamples > 0) {
    writeMotors(pwmL0, pwmR0);
    settleSamples--;
    if (settleSamples == 0) {
      pidReset(pid);                              // el PID arranca limpio despues de estabilizar
      if (mode == CONTROL) Serial.printf("# Inicia control PID (theta = %.1f deg, ref = %.1f deg)\n", theta, thetaRef);
      else                 Serial.printf("# Inicia excitacion (theta = %.1f deg)\n", theta);
    }
    return;
  }

  // 2  Calcular mando
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
      if (rampa) {                                // arranque suave hacia la referencia
        float paso = RAMPA_DEG_S * TS;
        float dif = thetaRef - refActual;
        if (fabsf(dif) <= paso) { refActual = thetaRef; rampa = false; }
        else refActual += (dif > 0 ? paso : -paso);
      } else {
        refActual = thetaRef;                     // despues: los cambios con 'r' son escalones
      }
      uCtrl = pidUpdate(pid, refActual, theta, thetaDotF, TS);
      applyDelta((int)lroundf(CTRL_SIGNO * uCtrl), pl, pr);
      break;
    default: break;                               
  }

  // 3 Seguridad
  if (mode == CONTROL && fabsf(refActual - theta) > ERR_LIMIT) errSamples++;
  else errSamples = 0;
  if (mode == CONTROL && errSamples > (int)(ERR_TIEMPO / TS)) {
    stopMotors();
    Serial.printf("# STOP: el control se alejo de la referencia (theta = %.1f, ref = %.1f deg)\n", theta, refActual);
    return;
  }
  if (mode != CALIB && mode != OPPOINT && mode != CONTROL && fabsf(theta) > THETA_LIMIT) {
    stopMotors();
    Serial.printf("# STOP: angulo fuera de limite (theta = %.1f deg)\n", theta);
    return;
  }

  // 4 Aplicar 
  writeMotors(pl, pr);

  // 5 Registrar
  if (mode == CONTROL)
    Serial.printf("%.2f,%d,%d,%d,%.3f,%.3f,%.2f,%.1f\n",
                  k * TS, pl, pr, adc, theta, thetaDotF, refActual, uCtrl);
  else
    Serial.printf("%.2f,%d,%d,%d,%.3f,%.3f\n", k * TS, pl, pr, adc, theta, thetaDotF);

  k++;
  if ((mode == EXP_PRBS || mode == EXP_MISO || mode == EXP_VALID) && k * TS > TEXP) {
    stopMotors(); Serial.println("# FIN del experimento");
  }
}
