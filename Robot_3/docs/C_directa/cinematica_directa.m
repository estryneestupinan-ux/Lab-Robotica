clc; clear; close all;

%% ================= ROBOT [cm] =================
L1  = 13;        % eslabon 1 [cm]
L2  = 15;        % eslabon 2 [cm]
G   = 11;        % gripper (colineal con eslabon 2) [cm]
L2e = L2 + G;    % eslabon 2 + gripper [cm]
H   = 0;         % altura de la base sobre la mesa [cm]  <-- poner la real

OFF1 = deg2rad(90);
OFF2 = deg2rad(-120);

%% ================= MATRICES DH SIMBOLICAS =================
syms th1 th2 d3 real
A1 = dh(0,          H,   0,   0);
A2 = dh(th1 + OFF1, 0,   L1,  0);
A3 = dh(th2 + OFF2, 0,   L2e, 0);
A4 = dh(0,          -d3, 0,   0);

T = simplify(A1*A2*A3*A4)
x = T(1,4)
y = T(2,4)
z = T(3,4)

%% ================= PUNTOS DE LA TRAYECTORIA =================
puntos = {'HOME', 'PLL', 'AGARRE PLL', 'PS', 'SOLTAR PS'};
q = [   0.0000  0.0000  0;
      -14.4775 15.5225  0;
      -14.4775 15.5225  8;
      165.5225 15.5225  0;
      165.5225 15.5225  8];

fprintf('\n');
for i = 1:size(q,1)
    Ti = double(subs(T, [th1 th2 d3], [deg2rad(q(i,1)) deg2rad(q(i,2)) q(i,3)]));
    fprintf('%-11s: th1 = %+8.2f deg, th2 = %+7.2f deg, d3 = %4.1f cm  ->  x = %7.2f cm, y = %7.2f cm, z = %7.2f cm\n', ...
        puntos{i}, q(i,1), q(i,2), q(i,3), Ti(1,4), Ti(2,4), Ti(3,4));
end

%% ================= GRAFICA (vista superior) =================
figure('Color','w'); hold on; grid on; axis equal;
idx     = [2 4];
colores = [0.85 0.1 0.1; 0.1 0.4 0.85];
verde   = [0.1 0.65 0.2];
h = gobjects(numel(idx),1);

for k = 1:numel(idx)
    i  = idx(k);
    a1 = OFF1 + deg2rad(q(i,1));
    a2 = a1 + OFF2 + deg2rad(q(i,2));
    u2 = [cos(a2); sin(a2)];
    P1 = L1*[cos(a1); sin(a1)];   % codo
    P2 = P1 + L2*u2;              % muneca
    P3 = P2 + G*u2;               % punta del gripper

    h(k) = plot([0 P1(2) P2(2)], [0 P1(1) P2(1)], '-o', 'LineWidth', 3, ...
                'Color', colores(k,:), 'MarkerFaceColor', colores(k,:));
    plot([P2(2) P3(2)], [P2(1) P3(1)], '-', 'LineWidth', 3, 'Color', verde);
    plot(P3(2), P3(1), 'kx', 'MarkerSize', 14, 'LineWidth', 2);
end
plot(0, 0, 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k');

lim = L1 + L2e + 5; xlim([-lim lim]); ylim([-lim lim]);
xlabel('Y [cm] (horizontal)'); ylabel('X [cm] (vertical)');
title(sprintf('Robot 3 en PLL y PS  (L1=%g, L2=%g, G=%g cm, gripper en verde)', L1, L2, G));
legend(h, puntos(idx), 'Location', 'southoutside');

%% ================= FUNCIONES =================
function A = dh(theta, d, a, alpha)
    A = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
         sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
         0           sin(alpha)             cos(alpha)            d;
         0           0                      0                     1];
end