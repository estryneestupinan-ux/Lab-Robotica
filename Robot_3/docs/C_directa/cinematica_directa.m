%% Cinematica directa - Robot 3 SCARA (RRP)
clc; clear; close all;

% Medidas del robot (m)
L1 = 0.1704;   % altura de la base
L2 = 0.130;    % eslabon 1
L3 = 0.1476;   % eslabon 2

%% Matrices DH simbolicas
syms th1 th2 d3

% Eslabon 1: theta = 0,   d = L1,  a = 0,  alpha = 0
A1 = [1 0 0 0;
      0 1 0 0;
      0 0 1 L1;
      0 0 0 1];

% Eslabon 2: theta = th1, d = 0,   a = L2, alpha = 0
A2 = [cos(th1) -sin(th1) 0 L2*cos(th1);
      sin(th1)  cos(th1) 0 L2*sin(th1);
      0         0        1 0;
      0         0        0 1];

% Eslabon 3: theta = th2, d = 0,   a = L3, alpha = 0
A3 = [cos(th2) -sin(th2) 0 L3*cos(th2);
      sin(th2)  cos(th2) 0 L3*sin(th2);
      0         0        1 0;
      0         0        0 1];

% Eslabon 4: theta = 0,   d = -d3, a = 0,  alpha = 0
A4 = [1 0 0 0;
      0 1 0 0;
      0 0 1 -d3;
      0 0 0 1];

T = simplify(A1*A2*A3*A4)

x = T(1,4)
y = T(2,4)
z = T(3,4)

%% Evaluar en los puntos de la trayectoria
% [th1 th2] en grados y d3 en metros
puntos = {'HOME', 'PLL', 'AGARRE PLL', 'PS', 'SOLTAR PS'};
q = [  92.41  99.28  0;
       68.08  41.11  0;
       68.08  41.11  0.08;
     -111.92  41.11  0;
     -111.92  41.11  0.08];

for i = 1:5
    Ti = double(subs(T, [th1 th2 d3], [deg2rad(q(i,1)) deg2rad(q(i,2)) q(i,3)]));
    fprintf('%s: x = %.2f cm, y = %.2f cm, z = %.2f cm\n', ...
        puntos{i}, Ti(1,4)*100, Ti(2,4)*100, Ti(3,4)*100);
end

%% Grafica del robot en PLL y PS (vista superior)
figure; hold on; grid on; axis equal;
for i = [2 4]
    t1 = deg2rad(q(i,1));
    t2 = deg2rad(q(i,2));
    xc = L2*cos(t1);            yc = L2*sin(t1);
    xe = xc + L3*cos(t1+t2);    ye = yc + L3*sin(t1+t2);
    plot([0 xc xe]*100, [0 yc ye]*100, '-o', 'LineWidth', 3);
end
xlabel('x (cm)'); ylabel('y (cm)');
title('Robot 3 en PLL y PS');
legend('PLL', 'PS');