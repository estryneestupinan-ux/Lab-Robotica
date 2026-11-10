
clc;
clear;
close all;

%% =========================================================
% ROBOT2 - CINEMATICA INVERSA PARA RECOGER BOTELLA
%
% Objetivos:
% 1. Llevar el centro de agarre al punto A.
% 2. Altura del agarre: 15 cm.
% 3. Buscar una pinza aproximadamente horizontal.
% 4. Respetar limites articulares preliminares.
% ==========================================================

%% 1. IMPORTAR ROBOT

archivo = 'C:\Users\jandr\Downloads\robot2.0\robot2\urdf\robot2_base_centrada.urdf';

if ~isfile(archivo)
    error('No se encontro el archivo URDF: %s', archivo);
end

robot = importrobot(archivo);
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

disp('Robot importado correctamente.');

%% 2. AGREGAR EL CENTRO DE AGARRE

% El desplazamiento se verifico visualmente en MATLAB.
% Sigue siendo una aproximacion respecto al CAD.

tool = rigidBody('tool_tip');

jointTool = rigidBodyJoint('joint_tool_tip', 'fixed');

distancia = 0.09267;

setFixedTransform(jointTool, ...
    trvec2tform([0 0 -distancia]));

tool.Joint = jointTool;

addBody(robot, tool, '4_Link');

disp('Centro de agarre agregado.');

%% 3. DEFINIR EL PUNTO A

% X = 30 cm
% Y = 18 cm
% Z = 15 cm

pA = [0.28; 0.19; 0.16];

fprintf('\nPUNTO A OBJETIVO [m]:\n');
disp(pA);

%% 4. CONFIGURAR CINEMATICA INVERSA

ik = inverseKinematics('RigidBodyTree', robot);

% MATLAB:
% 3 primeros pesos: orientacion
% 3 ultimos pesos: posicion
%
% Aqui resolvemos posicion y evaluamos
% la horizontalidad por separado.

pesos = [0 0 0 1 1 1];

Tobjetivo = eye(4);
Tobjetivo(1:3,4) = pA;

%% 5. LIMITES ARTICULARES PRELIMINARES

% joint_1: -180 a 180 grados
% joint_2:  -90 a 90 grados
% joint_3: -180 a 0 grados
% joint_4:  -90 a 90 grados (provisional)

limites = deg2rad([
    -180  180;
     -90   90;
    -180    0;
     -90   90
]);

%% 6. BUSCAR MULTIPLES SOLUCIONES

numIntentos = 250;

rng(10);

mejorCosto = inf;
mejorQ = [];
mejorError = inf;
mejorAnguloPinza = inf;
mejorAnguloBrazo = inf;

numValidas = 0;

fprintf('\nBuscando %d configuraciones...\n', ...
    numIntentos);

for k = 1:numIntentos

    % Primer intento: robot en posicion inicial
    if k == 1
        qInicial = homeConfiguration(robot);
    else
        % Intentos adicionales dentro de limites
        qInicial = zeros(1,4);

        for j = 1:4
            qInicial(j) = limites(j,1) + ...
                rand()*(limites(j,2)-limites(j,1));
        end
    end

    %% Resolver cinematica inversa

    [qPrueba, ~] = ik( ...
        'tool_tip', ...
        Tobjetivo, ...
        pesos, ...
        qInicial);

    % Normalizar angulos para juntas revolutas
    qPrueba = atan2(sin(qPrueba), cos(qPrueba));

    %% Verificar limites

    dentro = all( ...
        qPrueba >= limites(:,1)' - 1e-6 & ...
        qPrueba <= limites(:,2)' + 1e-6);

    if ~dentro
        continue;
    end

    %% Verificar posicion alcanzada

    Ttip = getTransform(robot, qPrueba, ...
        'tool_tip', 'base_centrada');

    pReal = Ttip(1:3,4);

    errorPos = norm(pReal - pA);

    % Precision maxima aceptada: 5 mm
    if errorPos > 0.005
        continue;
    end

    numValidas = numValidas + 1;

    %% MEDIR HORIZONTALIDAD DE LA PINZA

    % El tool_tip esta definido sobre -Z local
    % de 4_Link. Esa es la direccion aproximada
    % desde la muneca hacia el centro de agarre.

    T4 = getTransform(robot, qPrueba, ...
        '4_Link', 'base_centrada');

    p4 = T4(1:3,4);

    direccionPinza = pReal - p4;

    % Angulo respecto al plano horizontal XY
    anguloPinza = atan2( ...
        abs(direccionPinza(3)), ...
        norm(direccionPinza(1:2)));

    %% MEDIR HORIZONTALIDAD DEL BRAZO LARGO

    T2 = getTransform(robot, qPrueba, ...
        '2_Link', 'base_centrada');

    T3 = getTransform(robot, qPrueba, ...
        '3_Link', 'base_centrada');

    p2 = T2(1:3,4);
    p3 = T3(1:3,4);

    direccionBrazo = p3 - p2;

    anguloBrazo = atan2( ...
        abs(direccionBrazo(3)), ...
        norm(direccionBrazo(1:2)));

    %% SELECCIONAR MEJOR POSTURA

    % Prioridad:
    % 1. Pinza horizontal.
    % 2. Brazo largo menos inclinado.
    % 3. Evitar giros articulares excesivos.

    costo = ...
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

%% 7. VERIFICAR RESULTADO

fprintf('\nSoluciones validas encontradas: %d\n', ...
    numValidas);

if isempty(mejorQ)

    disp(' ');
    disp('NO SE ENCONTRO UNA SOLUCION VALIDA.');
    disp('El punto A podria estar fuera del alcance');
    disp('o la busqueda no encontro una postura');
    disp('dentro de los limites establecidos.');

    return;

end

%% 8. CALCULAR POSICION FINAL

Tfinal = getTransform(robot, mejorQ, ...
    'tool_tip', 'base_centrada');

pFinal = Tfinal(1:3,4);

qGrados = rad2deg(mejorQ);

%% 9. MOSTRAR RESULTADOS

fprintf('\n========================================\n');
fprintf('CINEMATICA INVERSA - AGARRE EN A\n');
fprintf('========================================\n');

fprintf('Joint 1: %.2f grados\n', qGrados(1));
fprintf('Joint 2: %.2f grados\n', qGrados(2));
fprintf('Joint 3: %.2f grados\n', qGrados(3));
fprintf('Joint 4: %.2f grados\n', qGrados(4));

fprintf('\nPunto objetivo [m]:\n');
disp(pA);

fprintf('Punto alcanzado [m]:\n');
disp(pFinal);

fprintf('Error de posicion: %.3f mm\n', ...
    mejorError*1000);

fprintf('Inclinacion de pinza: %.2f grados\n', ...
    rad2deg(mejorAnguloPinza));

fprintf('Inclinacion de brazo: %.2f grados\n', ...
    rad2deg(mejorAnguloBrazo));

%% 10. VISUALIZAR ROBOT

figure('Color','w', ...
    'Name','Robot2 - agarre de botella en A');

show(robot, mejorQ, ...
    'Visuals','on', ...
    'Frames','off');

hold on;
grid on;
axis equal;

% Punto A objetivo
hA = plot3( ...
    pA(1), pA(2), pA(3), ...
    'r*', ...
    'MarkerSize',16, ...
    'LineWidth',2);

% Centro de agarre alcanzado
hTip = plot3( ...
    pFinal(1), pFinal(2), pFinal(3), ...
    'go', ...
    'MarkerSize',12, ...
    'MarkerFaceColor','g');

% Origen de la base
hBase = plot3( ...
    0,0,0, ...
    'mp', ...
    'MarkerSize',16, ...
    'MarkerFaceColor','m');

% Linea desde muneca al centro de agarre
T4 = getTransform(robot, mejorQ, ...
    '4_Link', 'base_centrada');

p4 = T4(1:3,4);

plot3( ...
    [p4(1), pFinal(1)], ...
    [p4(2), pFinal(2)], ...
    [p4(3), pFinal(3)], ...
    'g--', ...
    'LineWidth',2, ...
    'HandleVisibility','off');

% Tramo del brazo largo
T2 = getTransform(robot, mejorQ, ...
    '2_Link', 'base_centrada');

T3 = getTransform(robot, mejorQ, ...
    '3_Link', 'base_centrada');

p2 = T2(1:3,4);
p3 = T3(1:3,4);

plot3( ...
    [p2(1), p3(1)], ...
    [p2(2), p3(2)], ...
    [p2(3), p3(3)], ...
    'b-', ...
    'LineWidth',3, ...
    'HandleVisibility','off');

legend([hA, hTip, hBase], ...
    {'Punto A', 'Centro de agarre', 'Base'}, ...
    'Location','best');

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

title(sprintf( ...
    'Agarre en A - pinza %.1f grados / brazo %.1f grados', ...
    rad2deg(mejorAnguloPinza), ...
    rad2deg(mejorAnguloBrazo)));

view(45,25);
rotate3d on;

%% 11. ADVERTENCIAS

fprintf('\n========================================\n');
fprintf('VERIFICACIONES\n');
fprintf('========================================\n');

if mejorError <= 0.005
    disp('Posicion alcanzada con error menor a 5 mm.');
else
    warning('Error de posicion demasiado grande.');
end

if rad2deg(mejorAnguloPinza) <= 15
    disp('Direccion de agarre casi horizontal.');
else
    warning('La pinza sigue inclinada.');
end

disp(' ');
disp('ATENCION: aun no se comprobaron colisiones.');
disp('No enviar estos angulos a Gazebo todavia.');
