%% IDENTIFICACION DEL BALANCIN EOLICO - modelo diferencial (SISO) y MISO
% Archivos (salen de logger.py), columnas: t,pwmL,pwmR,adcPot,theta,thetaDot
%   identificacion_diferencial.csv  -> comando 'e'
%   identificacion_miso.csv         -> comando 'm'
%   validacion.csv                  -> comando 'v'
% Requiere System Identification Toolbox (arx, armax, oe, compare, resid)
clear; clc; close all

Ts = 0.02;

%% 1) Cargar datos
D = readtable('identificacion_diferencial.csv');
M = readtable('identificacion_miso.csv');
V = readtable('validacion.csv');

% Punto de operacion: promedio del mando (la PRBS tiene media ~0)
PWM_L0 = mean(D.pwmL);
PWM_R0 = mean(D.pwmR);
fprintf('Punto de operacion: PWM_L0 = %.1f us, PWM_R0 = %.1f us\n', PWM_L0, PWM_R0);

% Funcion para pasar cada CSV a desviaciones respecto al punto de operacion
prep = @(T) deal(T.pwmL - PWM_L0, T.pwmR - PWM_R0, T.theta - mean(T.theta));

[uL_d, uR_d, y_d] = prep(D);
[uL_m, uR_m, y_m] = prep(M);
[uL_v, uR_v, y_v] = prep(V);

%% 2) Graficar los datos crudos
figure('Name','Datos');
subplot(3,1,1); plot(D.t, D.pwmL, D.t, D.pwmR); grid on; title('Exp. 1: diferencial'); legend('PWM_L','PWM_R')
subplot(3,1,2); plot(M.t, M.pwmL, M.t, M.pwmR); grid on; title('Exp. 2: MISO');       legend('PWM_L','PWM_R')
subplot(3,1,3); plot(D.t, D.theta, M.t, M.theta, V.t, V.theta); grid on
title('\theta [deg]'); legend('diferencial','MISO','validacion'); xlabel('t [s]')

%% 3) Objetos iddata
% Diferencial (SISO): u_delta = uR - uL
zd   = iddata(y_d, uR_d - uL_d, Ts, 'InputName','uDelta', 'OutputName','theta');
zv_d = iddata(y_v, uR_v - uL_v, Ts, 'InputName','uDelta', 'OutputName','theta');

% MISO: dos entradas separadas [uL uR]
zm   = iddata(y_m, [uL_m uR_m], Ts, 'InputName',{'uL','uR'}, 'OutputName','theta');
zv_m = iddata(y_v, [uL_v uR_v], Ts, 'InputName',{'uL','uR'}, 'OutputName','theta');

%% 4) Barrido de retardo nk (como en la presentacion)
fprintf('\n nk | FIT_val ARX dif [%%] | FIT_val ARX MISO [%%]\n');
best = struct('fitD',-Inf,'fitM',-Inf);
for nk = 1:5
    mD = arx(zd, [2 2 nk]);
    mM = arx(zm, [2 [2 2] [nk nk]]);
    [~, fD] = compare(zv_d, mD);
    [~, fM] = compare(zv_m, mM);
    fprintf(' %d  |       %6.2f        |        %6.2f\n', nk, fD, fM);
    if fD > best.fitD, best.fitD = fD; best.nkD = nk; best.mD = mD; end
    if fM > best.fitM, best.fitM = fM; best.nkM = nk; best.mM = mM; end
end
nk = best.nkD;

%% 5) Comparar estructuras con el mejor nk (diferencial)
m_arx   = arx(zd,   [2 2 nk]);
m_armax = armax(zd, [2 2 2 nk]);      % [na nb nc nk]
m_oe    = oe(zd,    [2 2 nk]);        % [nb nf nk]

figure('Name','Validacion diferencial');
compare(zv_d, m_arx, m_armax, m_oe); grid on

figure('Name','Residuos ARMAX'); resid(zv_d, m_armax)

%% 6) Resultado MISO: ¿los motores son simetricos?
mM = best.mM;
GL = tf(mM(1,1));   GR = tf(mM(1,2));
kL = dcgain(GL);    kR = dcgain(GR);
fprintf('\nMISO (nk = %d): ganancia DC  GL = %.4f   GR = %.4f deg/us\n', best.nkM, kL, kR);
fprintf('Se espera GR ~ -GL  ->  GR/GL = %.2f (ideal -1)\n', kR/kL);

figure('Name','Validacion MISO vs diferencial');
compare(zv_m, mM); hold on; grid on; title('Modelo MISO en validacion')

%% 7) Modelo final para control
fprintf('\nFIT validacion: diferencial = %.2f %%, MISO = %.2f %%\n', best.fitD, best.fitM);
fprintf('Si el MISO no mejora mucho, usar el diferencial para disenar el PID.\n');
Gz = tf(m_arx)            % G(z) = theta/uDelta  [deg/us]
polos = pole(m_arx)       % deben estar dentro del circulo unitario
Gs = d2c(m_arx)           % equivalente continuo para disenar el PID
