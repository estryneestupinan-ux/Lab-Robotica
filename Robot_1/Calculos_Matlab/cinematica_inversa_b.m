
clc;
clear;
close all;

%% =========================================================
% ROBOT2 - CINEMATICA INVERSA PARA EL PUNTO B
%
% Objetivos:
% 1. Comprobar si el punto B es alcanzable.
% 2. Buscar 1500 configuraciones.
% 3. Verificar limites articulares.
% 4. Permitir hasta 10 mm de error para diagnostico.
% 5. Medir el menor error dentro de los limites.
% 6. Favorecer una pinza horizontal.
%
% IMPORTANTE:
% Esta prueba NO genera movimiento A -> B.
% ==========================================================

%% 1. IMPORTAR ROBOT

archivo = 'C:\Users\maria\Downloads\robot2.0 (2)\robot2.0\robot2\urdf\robot2_base_centrada.urdf';

if ~isfile(archivo)
    error('No se encontro el archivo URDF: %s', archivo);
end

robot = importrobot(archivo);
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

disp('Robot importado correctamente.');

%% 2. AGREGAR CENTRO DE AGARRE

tool = rigidBody('tool_tip');

jointTool = rigidBodyJoint('joint_tool_tip', 'fixed');

% Distancia aproximada desde la muneca
% hasta el centro de agarre: 92.67 mm.

setFixedTransform(jointTool, ...
    trvec2tform([0 0 -0.09267]));

tool.Joint = jointTool;

addBody(robot, tool, '4_Link');

disp('Centro de agarre agregado.');

%% 3. DEFINIR PUNTOS A Y B

% Punto A: recoger botella
pA = [0.28; 0.19; 0.16];

% Punto B: posicion para el vertido
pB = [-0.34; 0; 0.25];

fprintf('\n====================================\n');
fprintf('PUNTOS DE TRABAJO\n');
fprintf('====================================\n');

fprintf('Punto A [m]:\n');
disp(pA);

fprintf('Punto B [m]:\n');
disp(pB);

%% 4. CONFIGURAR CINEMATICA INVERSA

ik = inverseKinematics('RigidBodyTree', robot);

% Los tres primeros pesos corresponden
% a orientacion y los ultimos a posicion.
%
% Aqui resolvemos posicion y evaluamos
% la horizontalidad por separado.

pesos = [0 0 0 1 1 1];

Tobjetivo = eye(4);
Tobjetivo(1:3,4) = pB;

%% 5. LIMITES ARTICULARES PRELIMINARES

limites = deg2rad([
    -180  180;
     -90   90;
    -180    0;
     -90   90
]);

%% 6. PARAMETROS DE BUSQUEDA

numIntentos = 500;

% Tolerancia exploratoria: 10 mm
toleranciaExploratoria = 0.010;

% Precision preferida: 5 mm
toleranciaPreferida = 0.005;

rng(20);

%% 7. VARIABLES PARA GUARDAR RESULTADOS

mejorCosto = inf;

mejorQ = [];
mejorError = inf;

mejorAnguloPinza = inf;
mejorAnguloBrazo = inf;

numValidas = 0;
numDentroLimites = 0;
numPrecisos = 0;

% Mejor error sin importar los limites
menorErrorSinFiltros = inf;

% Mejor error respetando los limites
menorErrorDentroLimites = inf;

% Guardar tambien la postura que produce
% el menor error dentro de los limites
qMenorError = [];

fprintf('\nBuscando %d configuraciones para B...\n', ...
    numIntentos);

%% 8. BUSQUEDA DE CINEMATICA INVERSA

for k = 1:numIntentos

    %% 8.1 Elegir configuracion inicial

    if k == 1

        qInicial = homeConfiguration(robot);

    elseif k == 2

        % Postura encontrada anteriormente en A
        qInicial = deg2rad( ...
            [-146.92 88.09 -98.68 -0.89]);

    else

        qInicial = zeros(1,4);

        for j = 1:4

            qInicial(j) = limites(j,1) + ...
                rand() * ...
                (limites(j,2)-limites(j,1));

        end

    end

    %% 8.2 Resolver cinematica inversa

    [qPrueba, ~] = ik( ...
        'tool_tip', ...
        Tobjetivo, ...
        pesos, ...
        qInicial);

    % Normalizar angulos
    qPrueba = atan2( ...
        sin(qPrueba), cos(qPrueba));

    %% 8.3 Calcular posicion alcanzada

    Ttip = getTransform( ...
        robot, qPrueba, ...
        'tool_tip', 'base_centrada');

    pReal = Ttip(1:3,4);

    errorPos = norm(pReal - pB);

    %% 8.4 Guardar menor error sin filtros

    if errorPos < menorErrorSinFiltros

        menorErrorSinFiltros = errorPos;

    end

    %% 8.5 Verificar limites articulares

    dentro = all( ...
        qPrueba >= limites(:,1)' - 1e-6 & ...
        qPrueba <= limites(:,2)' + 1e-6);

    if ~dentro
        continue;
    end

    numDentroLimites = numDentroLimites + 1;

    %% 8.6 Guardar menor error dentro de limites

    if errorPos < menorErrorDentroLimites

        menorErrorDentroLimites = errorPos;
        qMenorError = qPrueba;

    end

    %% 8.7 Contar soluciones precisas

    if errorPos <= toleranciaPreferida

        numPrecisos = numPrecisos + 1;

    end

    %% 8.8 Aplicar tolerancia exploratoria

    if errorPos > toleranciaExploratoria
        continue;
    end

    numValidas = numValidas + 1;

    %% 8.9 Calcular inclinacion de pinza

    T4 = getTransform( ...
        robot, qPrueba, ...
        '4_Link', 'base_centrada');

    p4 = T4(1:3,4);

    direccionPinza = pReal - p4;

    anguloPinza = atan2( ...
        abs(direccionPinza(3)), ...
        norm(direccionPinza(1:2)));

    %% 8.10 Calcular inclinacion del brazo largo

    T2 = getTransform( ...
        robot, qPrueba, ...
        '2_Link', 'base_centrada');

    T3 = getTransform( ...
        robot, qPrueba, ...
        '3_Link', 'base_centrada');

    p2 = T2(1:3,4);
    p3 = T3(1:3,4);

    direccionBrazo = p3 - p2;

    anguloBrazo = atan2( ...
        abs(direccionBrazo(3)), ...
        norm(direccionBrazo(1:2)));

    %% 8.11 Seleccionar mejor postura

    % En esta prueba damos prioridad a:
    % 1. Menor error de posicion.
    % 2. Pinza mas horizontal.
    % 3. Brazo menos inclinado.

    costo = ...
        10 * (errorPos / toleranciaExploratoria) + ...
         5 * anguloPinza + ...
         1 * anguloBrazo + ...
         0.01 * norm(qPrueba);

    if costo < mejorCosto

        mejorCosto = costo;

        mejorQ = qPrueba;
        mejorError = errorPos;

        mejorAnguloPinza = anguloPinza;
        mejorAnguloBrazo = anguloBrazo;

    end

end

%% 9. MOSTRAR DIAGNOSTICO GENERAL

fprintf('\n========================================\n');
fprintf('DIAGNOSTICO DEL PUNTO B\n');
fprintf('========================================\n');

fprintf('Intentos realizados: %d\n', ...
    numIntentos);

fprintf('Posturas dentro de limites: %d\n', ...
    numDentroLimites);

fprintf('Soluciones con error <= 10 mm: %d\n', ...
    numValidas);

fprintf('Soluciones con error <= 5 mm: %d\n', ...
    numPrecisos);

fprintf('\nMenor error sin filtros: %.3f mm\n', ...
    menorErrorSinFiltros * 1000);

if isfinite(menorErrorDentroLimites)

    fprintf('Menor error dentro de limites: %.3f mm\n', ...
        menorErrorDentroLimites * 1000);

else

    fprintf('Menor error dentro de limites: NO ENCONTRADO\n');

end

%% 10. VERIFICAR SI EXISTE UNA POSTURA ACEPTABLE

if isempty(mejorQ)

    fprintf('\n========================================\n');
    fprintf('NO SE ENCONTRO SOLUCION EXPLORATORIA\n');
    fprintf('========================================\n');

    fprintf('No se encontro una postura con error\n');
    fprintf('menor o igual a 10 mm dentro de limites.\n');

    if ~isempty(qMenorError)

        fprintf('\nMejor postura dentro de limites:\n');

        qGrados = rad2deg(qMenorError);

        for j = 1:4

            fprintf('Joint %d: %.2f grados\n', ...
                j, qGrados(j));

        end

    end

    fprintf('\nNo se genera movimiento hacia B.\n');

    return;

end

%% 11. MOSTRAR MEJOR POSTURA EXPLORATORIA

fprintf('\n========================================\n');
fprintf('MEJOR POSTURA EXPLORATORIA EN B\n');
fprintf('========================================\n');

qBgrados = rad2deg(mejorQ);

for j = 1:4

    fprintf('Joint %d: %.2f grados\n', ...
        j, qBgrados(j));

end

fprintf('\nError de posicion: %.3f mm\n', ...
    mejorError * 1000);

fprintf('Inclinacion de pinza: %.2f grados\n', ...
    rad2deg(mejorAnguloPinza));

fprintf('Inclinacion de brazo: %.2f grados\n', ...
    rad2deg(mejorAnguloBrazo));

if mejorError <= toleranciaPreferida

    fprintf('\nPRECISION: menor o igual a 5 mm.\n');

else

    fprintf('\nPRECISION: entre 5 y 10 mm.\n');
    fprintf('Esta postura es solo exploratoria.\n');

end

%% 12. CALCULAR POSICION FINAL

Tfinal = getTransform( ...
    robot, mejorQ, ...
    'tool_tip', 'base_centrada');

pFinal = Tfinal(1:3,4);

fprintf('\nPunto B objetivo [m]:\n');
disp(pB);

fprintf('Punto realmente alcanzado [m]:\n');
disp(pFinal);

%% 13. VISUALIZAR ROBOT

figure( ...
    'Color', 'w', ...
    'Name', 'Robot2 - estudio del punto B');

show(robot, mejorQ, ...
    'Visuals', 'on', ...
    'Frames', 'off');

hold on;
grid on;
axis equal;

% Punto A
hA = plot3( ...
    pA(1), pA(2), pA(3), ...
    'r*', ...
    'MarkerSize', 15, ...
    'LineWidth', 2);

% Punto B objetivo
hB = plot3( ...
    pB(1), pB(2), pB(3), ...
    'bo', ...
    'MarkerSize', 12, ...
    'MarkerFaceColor', 'b');

% Punto realmente alcanzado
hReal = plot3( ...
    pFinal(1), pFinal(2), pFinal(3), ...
    'go', ...
    'MarkerSize', 11, ...
    'MarkerFaceColor', 'g');

% Centro de la base
plot3( ...
    0, 0, 0, ...
    'mp', ...
    'MarkerSize', 15, ...
    'MarkerFaceColor', 'm');

% Linea entre A y B: solo referencia visual,
% NO representa una trayectoria validada.
plot3( ...
    [pA(1), pB(1)], ...
    [pA(2), pB(2)], ...
    [pA(3), pB(3)], ...
    'k--', ...
    'LineWidth', 1.5, ...
    'HandleVisibility', 'off');

% Mostrar tramo del brazo largo
T2 = getTransform( ...
    robot, mejorQ, ...
    '2_Link', 'base_centrada');

T3 = getTransform( ...
    robot, mejorQ, ...
    '3_Link', 'base_centrada');

p2 = T2(1:3,4);
p3 = T3(1:3,4);

plot3( ...
    [p2(1), p3(1)], ...
    [p2(2), p3(2)], ...
    [p2(3), p3(3)], ...
    'b-', ...
    'LineWidth', 3, ...
    'HandleVisibility', 'off');

legend( ...
    [hA, hB, hReal], ...
    {'Punto A', 'Punto B', 'Posicion alcanzada'}, ...
    'Location', 'best');

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

title(sprintf( ...
    'Punto B - error %.2f mm / pinza %.1f grados', ...
    mejorError*1000, ...
    rad2deg(mejorAnguloPinza)));

view(45,25);
rotate3d on;

%% 14. GUARDAR RESULTADOS

% Guardamos la postura para analisis posterior.
% NO significa que sea segura para ejecutar.

qB = mejorQ;

save('resultado_punto_B.mat', ...
    'qB', ...
    'pA', ...
    'pB', ...
    'pFinal', ...
    'mejorError', ...
    'menorErrorDentroLimites', ...
    'mejorAnguloPinza', ...
    'mejorAnguloBrazo');

fprintf('\nResultado guardado en resultado_punto_B.mat\n');

%% 15. ADVERTENCIA FINAL

fprintf('\n========================================\n');
fprintf('IMPORTANTE\n');
fprintf('========================================\n');

fprintf('Esta es una prueba de alcance y postura.\n');
fprintf('Todavia NO se verificaron colisiones.\n');
fprintf('Todavia NO se verifico la trayectoria A-B.\n');
fprintf('NO enviar estos angulos a Gazebo todavia.\n');
