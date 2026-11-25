%% Identificacion del balancin con Box-Jenkins y diseno del PID
clear; clc; close all;

datos = readtable('../datos/identificacion_diferencial_20261123_164432.csv');

Ts   = 0.02;
t    = datos.t;
pwmL = datos.pwmL;
pwmR = datos.pwmR;
y    = datos.theta;

u = pwmR - pwmL;     % entrada diferencial (PRBS)

figure
subplot(2,1,1)
plot(t, pwmL, t, pwmR, 'LineWidth', 1.2); grid on
ylabel('PWM [us]'); legend('PWM_L','PWM_R')
title('Senales de identificacion diferencial')
subplot(2,1,2)
plot(t, y, 'LineWidth', 1.2); grid on
xlabel('Tiempo [s]'); ylabel('\theta [deg]')
title('Angulo medido')

% Estimacion y validacion
data = iddata(y, u, Ts);

N  = length(y);
Ne = round(0.7*N);                     % 70% estimar, 30% validar
ze = data(1:Ne);
zv = data(Ne+1:end);

[ze, T] = detrend(ze, 0);              % quita la media (misma T para validacion)
zv = detrend(zv, T);

% Modelos
nk = 5;                                % retardo de 100 ms

% Punto de partida de la planta: segundo orden subamortiguado (pendulo con friccion)
F0 = [1 -1.95 0.955];
B0 = [zeros(1,nk) -1e-3 -1e-3];

m_arx = arx(ze, [2 2 nk]);
m_armax = armax(ze, idpoly(F0, B0, [1 0 0], 1, 1, [], Ts));
m_oe  = oe(ze, idpoly(1, B0, 1, 1, F0, [], Ts));
m_bj  = bj(ze, idpoly(1, B0, [1 0 0], [1 0 0], F0, [], Ts));

m_bj

figure
compare(zv, m_arx, m_armax, m_oe, m_bj)

figure
resid(zv, m_armax)

figure
resid(zv, m_oe)

figure
resid(zv, m_bj)

%Planta identificada (Box-Jenkins)
Gd = tf(m_bj.B, m_bj.F, Ts, 'Variable', 'z^-1')
polos = roots(m_bj.F);                 % polos de la planta (raices de F)
s     = log(polos)/Ts;        % polos equivalentes en continuo
fprintf('Polos de la planta: %s\n', mat2str(polos', 4));
fprintf('wn = %.2f rad/s   zeta = %.2f\n', abs(s(1)), -real(s(1))/abs(s(1)));

figure
pzmap(Gd); grid on
title('Polos y ceros de la planta (BJ)')

% Ganancias del PID
P   = -Gd; 
wc  = 1.5;                             
opt = pidtuneOptions('PhaseMargin', 70);
C   = pidtune(P, 'PIDF', wc, opt);
[Kp, Ki, Kd, Tf] = piddata(C);
fprintf('\nKp = %.4f  Ki = %.4f  Kd = %.4f  Tf = %.4f s\n', Kp, Ki, Kd, Tf);
fprintf('En el balancin:  k %.2f %.2f %.2f\n', Kp, Ki, Kd);

figure
step(10*feedback(C*P, 1), 15); grid on
ylabel('\theta [deg]')
title('Lazo cerrado con el PID: escalon de 10 deg')
