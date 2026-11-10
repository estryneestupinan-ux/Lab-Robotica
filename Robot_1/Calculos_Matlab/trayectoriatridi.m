
clc;
clear;
close all;

%% =========================================================
% PUNTOS A Y B (milimetros)
% ==========================================================

% Punto A: altura de 20 cm
A = [248.08, 201.48, 200];

% Punto B: altura de 25 cm
B = [-348.66, 0, 250];

% Convertir de milimetros a metros
A = A / 1000;
B = B / 1000;

%% =========================================================
% PARAMETROS DE LA TRAYECTORIA
% ==========================================================

n = 300;       % Numero de puntos
T = 15;        % Duracion total en segundos

tiempo = linspace(0, T, n);
u = tiempo / T;

% Interpolacion cubica suave
s = 3*u.^2 - 2*u.^3;

%% =========================================================
% TRAYECTORIA CURVA EN X-Y
% ==========================================================

% Distancias desde el origen
RA = hypot(A(1), A(2));
RB = hypot(B(1), B(2));

% Angulos de A y B
thetaA = atan2(A(2), A(1));
thetaB = atan2(B(2), B(1));

% Interpolacion del radio
R = RA + (RB - RA).*s;

% Interpolacion angular
theta = thetaA + (thetaB - thetaA).*s;

% Coordenadas X e Y
X = R .* cos(theta);
Y = R .* sin(theta);

%% =========================================================
% TRAYECTORIA VERTICAL Z
% ==========================================================

% Subir suavemente de 20 cm a 25 cm
Z = A(3) + (B(3) - A(3)).*s;

%% =========================================================
% GARANTIZAR PUNTOS EXACTOS
% ==========================================================

X(1) = A(1);
Y(1) = A(2);
Z(1) = A(3);

X(end) = B(1);
Y(end) = B(2);
Z(end) = B(3);

%% =========================================================
% GRAFICA 3D
% ==========================================================

figure('Color', 'w');

hold on;
grid on;
axis equal;
view(45, 30);

% Trayectoria
plot3(X, Y, Z, 'b', 'LineWidth', 3);

% Punto A
plot3(A(1), A(2), A(3), 'go', ...
    'MarkerSize', 10, ...
    'MarkerFaceColor', 'g');

% Punto B
plot3(B(1), B(2), B(3), 'ro', ...
    'MarkerSize', 10, ...
    'MarkerFaceColor', 'r');

% Etiquetas
text(A(1), A(2), A(3)+0.015, ...
    'PUNTO A - 20 cm', 'FontWeight', 'bold');

text(B(1), B(2), B(3)+0.015, ...
    'PUNTO B - 25 cm', 'FontWeight', 'bold');

% Base del robot
plot3(0, 0, 0, 'ks', ...
    'MarkerSize', 9, ...
    'MarkerFaceColor', 'k');

% Proyecciones verticales
plot3([A(1) A(1)], [A(2) A(2)], ...
    [0 A(3)], 'g--');

plot3([B(1) B(1)], [B(2) B(2)], ...
    [0 B(3)], 'r--');

% Plano del suelo
[Xsuelo, Ysuelo] = meshgrid(-0.5:0.1:0.5);

Zsuelo = zeros(size(Xsuelo));

surf(Xsuelo, Ysuelo, Zsuelo, ...
    'FaceAlpha', 0.15, ...
    'EdgeColor', 'none');

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

title('Trayectoria 3D: A (20 cm) a B (25 cm)');

legend('Trayectoria', 'Punto A', 'Punto B', ...
    'Base', 'Location', 'best');

%% =========================================================
% EXPORTAR CSV CARTESIANO
% ==========================================================

trayectoria = table( ...
    tiempo(:), X(:), Y(:), Z(:), ...
    'VariableNames', {'tiempo', 'X', 'Y', 'Z'});

writetable(trayectoria, 'trayectoria_cartesiana.csv');

disp('Trayectoria generada correctamente.');
disp('Archivo: trayectoria_cartesiana.csv');
