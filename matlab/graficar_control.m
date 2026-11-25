%% Grafica la ultima prueba de control del balancin
clear; clc; close all;

d = dir('../datos/control_pid_*.csv');
[~, i] = max([d.datenum]);
datos = readtable(fullfile(d(i).folder, d(i).name));
fprintf('Prueba: %s\n', d(i).name);

t     = datos.t;
theta = datos.theta;
ref   = datos.ref;
pwmL  = datos.pwmL;
pwmR  = datos.pwmR;

figure
subplot(2,1,1)
plot(t, ref, 'k--', 'LineWidth', 1.3); hold on
plot(t, theta, 'LineWidth', 1.3); grid on
ylabel('\theta [deg]'); legend('Referencia', '\theta medido')
title('Control PID del balancin')

subplot(2,1,2)
plot(t, pwmL, t, pwmR, 'LineWidth', 1.2); grid on
xlabel('Tiempo [s]'); ylabel('PWM [us]'); legend('PWM_L', 'PWM_R')
title('Mando aplicado a los motores')
