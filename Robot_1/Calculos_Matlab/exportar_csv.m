
%% 9. GRAFICAS DE VERIFICACION - TEMA OSCURO

% Colores
fondo = [0.07 0.07 0.09];
fondoEjes = [0.11 0.11 0.14];
texto = [1 1 1];
rejilla = [0.45 0.45 0.50];

% Crear figura
fig = figure( ...
    'Color', fondo, ...
    'Name', 'Trayectoria A-B V2 completa', ...
    'NumberTitle', 'off', ...
    'Position', [100 80 1100 850]);

% Distribucion de las tres graficas
t = tiledlayout(3,1, ...
    'TileSpacing', 'compact', ...
    'Padding', 'loose');

% Fondo general
t.TileSpacing = 'compact';

%% GRAFICA 1: POSICIONES

ax1 = nexttile;

plot(tiempo, rad2deg(Q), 'LineWidth', 1.8);

title('POSICIONES ARTICULARES', ...
    'Color', texto, 'FontWeight', 'bold');

ylabel('Posicion [grados]', 'Color', texto);

grid on;

legend('joint_1','joint_2','joint_3','joint_4', ...
    'Location', 'eastoutside', ...
    'TextColor', texto, ...
    'Color', fondoEjes, ...
    'EdgeColor', rejilla);

%% GRAFICA 2: VELOCIDADES

ax2 = nexttile;

plot(tiempo, rad2deg(V), 'LineWidth', 1.8);

title('VELOCIDADES ARTICULARES', ...
    'Color', texto, 'FontWeight', 'bold');

ylabel('Velocidad [grados/s]', 'Color', texto);

grid on;

legend('joint_1','joint_2','joint_3','joint_4', ...
    'Location', 'eastoutside', ...
    'TextColor', texto, ...
    'Color', fondoEjes, ...
    'EdgeColor', rejilla);

%% GRAFICA 3: ACELERACIONES

ax3 = nexttile;

plot(tiempo, rad2deg(A), 'LineWidth', 1.8);

title('ACELERACIONES ARTICULARES', ...
    'Color', texto, 'FontWeight', 'bold');

xlabel('Tiempo [s]', 'Color', texto);

ylabel('Aceleracion [grados/s^2]', ...
    'Color', texto);

grid on;

legend('joint_1','joint_2','joint_3','joint_4', ...
    'Location', 'eastoutside', ...
    'TextColor', texto, ...
    'Color', fondoEjes, ...
    'EdgeColor', rejilla);

%% APLICAR ESTILO A LOS TRES EJES

ejes = [ax1 ax2 ax3];

for k = 1:3

    ax = ejes(k);

    % Fondo negro oscuro
    ax.Color = fondoEjes;

    % Numeros y lineas de los ejes
    ax.XColor = texto;
    ax.YColor = texto;

    % Rejilla
    ax.GridColor = rejilla;
    ax.GridAlpha = 0.35;

    % Tipografia
    ax.FontSize = 11;
    ax.FontWeight = 'bold';

    % Bordes
    ax.LineWidth = 1.1;

    % Limites del tiempo
    xlim(ax, [tiempo(1) tiempo(end)]);

end

% Sincronizar el eje del tiempo
linkaxes(ejes, 'x');

% Titulo general
title(t, 'ROBOT2 - TRAYECTORIA A-B V2', ...
    'Color', texto, ...
    'FontSize', 16, ...
    'FontWeight', 'bold');
