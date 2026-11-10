
clc;
clear;
close all;

%% =========================================================
% ROBOT2 - TRAYECTORIA A -> B VERSION 2
%
% MOVIMIENTO:
% 1. Agarre en A
% 2. Elevacion vertical aproximada
% 3. Recogimiento del brazo usando el codo
% 4. Giro y extension hacia B
%
% Incluye:
% - Busqueda de postura intermedia
% - Interpolacion articular suave
% - Animacion 3D
% - Graficas articulares
% - Trayectoria cartesiana
% - Inclinacion de pinza
% - Comparacion con trayectoria anterior
%
% IMPORTANTE:
% No se han verificado colisiones.
% ==========================================================

%% 1. IMPORTAR ROBOT

archivo = 'C:\Users\maria\Downloads\robot2.0 (2)\robot2.0\robot2\urdf\robot2_base_centrada.urdf';

if ~isfile(archivo)
    error('No se encontro el URDF: %s', archivo);
end

robot = importrobot(archivo);
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

%% 2. AGREGAR TOOL_TIP

tool = rigidBody('tool_tip');
jointTool = rigidBodyJoint('joint_tool_tip','fixed');

setFixedTransform(jointTool, ...
    trvec2tform([0 0 -0.09267]));

tool.Joint = jointTool;
addBody(robot,tool,'4_Link');

%% 3. PUNTOS DE TRABAJO

pA = [0.28; 0.19; 0.16];
pB = [-0.34; 0; 0.25];

% Posturas encontradas anteriormente
qA = deg2rad([-146.92 88.09 -98.68 -0.89]);
qB = deg2rad([-0.40 81.93 -85.56 0.14]);

% Elevacion inicial de 5 cm
pElevado = pA + [0; 0; 0.05];

%% 4. LIMITES ARTICULARES PRELIMINARES

limites = deg2rad([
    -180  180;
     -90   90;
    -180    0;
     -90   90
]);

%% 5. CONFIGURAR CINEMATICA INVERSA

ik = inverseKinematics('RigidBodyTree',robot);

% Control de posicion XYZ
pesos = [0 0 0 1 1 1];

%% 6. CALCULAR POSTURA ELEVADA

Televado = eye(4);
Televado(1:3,4) = pElevado;

[qElevado,~] = ik( ...
    'tool_tip',Televado,pesos,qA);

qElevado = atan2(sin(qElevado),cos(qElevado));

Treal = getTransform( ...
    robot,qElevado,'tool_tip','base_centrada');

errorElevado = norm(Treal(1:3,4)-pElevado);

dentroElevado = all( ...
    qElevado >= limites(:,1)' - 1e-6 & ...
    qElevado <= limites(:,2)' + 1e-6);

if ~dentroElevado || errorElevado > 0.005
    error('No se encontro una postura elevada valida.');
end

%% 7. BUSCAR POSTURA INTERMEDIA DE RECOGIMIENTO

% Queremos que el robot acerque la botella
% un poco hacia la base y aumente la altura.
%
% Probaremos distintos puntos candidatos
% alrededor de la zona central del robot.

rng(30);

numIntentos = 350;

mejorCosto = inf;
qIntermedio = [];
pIntermedio = [];

% Preferimos una zona elevada y mas cercana
% a la base que A y B.
pPreferido = [0.12; 0.08; 0.28];

fprintf('\nBuscando postura intermedia...\n');

for k = 1:numIntentos

    if k == 1

        pObjetivo = pPreferido;

    else

        % Explorar una region cercana
        pObjetivo = pPreferido + [
            0.10*(2*rand-1);
            0.10*(2*rand-1);
            0.05*(2*rand-1)
        ];

    end

    Tobjetivo = eye(4);
    Tobjetivo(1:3,4) = pObjetivo;

    % Usar la postura elevada como inicio
    % en algunos intentos.
    if mod(k,3) == 0

        qInicial = zeros(1,4);

        for j = 1:4
            qInicial(j) = limites(j,1) + ...
                rand()*(limites(j,2)-limites(j,1));
        end

    else

        qInicial = qElevado;

    end

    [qPrueba,~] = ik( ...
        'tool_tip',Tobjetivo,pesos,qInicial);

    qPrueba = atan2(sin(qPrueba),cos(qPrueba));

    %% Comprobar limites

    dentro = all( ...
        qPrueba >= limites(:,1)' - 1e-6 & ...
        qPrueba <= limites(:,2)' + 1e-6);

    if ~dentro
        continue;
    end

    %% Comprobar posicion

    Ttip = getTransform( ...
        robot,qPrueba,'tool_tip','base_centrada');

    pReal = Ttip(1:3,4);

    errorPos = norm(pReal-pObjetivo);

    if errorPos > 0.005
        continue;
    end

    %% Medir horizontalidad de pinza

    T4 = getTransform( ...
        robot,qPrueba,'4_Link','base_centrada');

    p4 = T4(1:3,4);

    vPinza = pReal-p4;

    anguloPinza = atan2( ...
        abs(vPinza(3)),norm(vPinza(1:2)));

    %% Medir distancia horizontal a base

    radio = norm(pReal(1:2));

    %% Medir cambio de codo

    cambioCodo = abs(qPrueba(2)-qElevado(2)) + ...
                 abs(qPrueba(3)-qElevado(3));

    %% Elegir una postura coherente

    % Preferimos:
    % - Cercania al punto intermedio propuesto
    % - Pinza relativamente horizontal
    % - Radio menor que en A
    % - Movimiento razonable desde elevacion
    %
    % No se fuerza una postura extrema.

    costo = ...
        3*norm(pReal-pPreferido) + ...
        2*anguloPinza + ...
        0.6*radio + ...
        0.15*norm(qPrueba-qElevado);

    % Favorecer movimiento real del codo
    if cambioCodo < deg2rad(8)
        costo = costo + 0.5;
    end

    if costo < mejorCosto

        mejorCosto = costo;
        qIntermedio = qPrueba;
        pIntermedio = pReal;

    end

end

%% 8. VERIFICAR POSTURA INTERMEDIA

if isempty(qIntermedio)

    error(['No se encontro una postura intermedia ' ...
           'valida. Prueba ampliar la busqueda.']);

end

fprintf('\n====================================\n');
fprintf('POSTURA INTERMEDIA ENCONTRADA\n');
fprintf('====================================\n');

fprintf('Posicion [m]:\n');
disp(pIntermedio);

fprintf('Angulos [grados]:\n');
disp(rad2deg(qIntermedio));

fprintf('Cambio de joint 2: %.2f grados\n', ...
    rad2deg(qIntermedio(2)-qElevado(2)));

fprintf('Cambio de joint 3: %.2f grados\n', ...
    rad2deg(qIntermedio(3)-qElevado(3)));

%% 9. DEFINIR TIEMPOS DE MOVIMIENTO

% Fase 1: elevar botella
tElevacion = 3;

% Fase 2: recoger brazo
tRecogimiento = 5;

% Fase 3: extender hacia B
tTraslado = 9;

frecuencia = 20;

%% 10. CREAR TRAYECTORIA SUAVE

% Usamos smoothstep de quinto orden:
% posicion, velocidad y aceleracion
% suaves en cada extremo.

crearTramo = @(q0,q1,n) ...
    q0 + ( ...
    (10*linspace(0,1,n)'.^3 - ...
     15*linspace(0,1,n)'.^4 + ...
      6*linspace(0,1,n)'.^5) ...
    )*(q1-q0);

n1 = round(tElevacion*frecuencia)+1;
n2 = round(tRecogimiento*frecuencia)+1;
n3 = round(tTraslado*frecuencia)+1;

Q1 = crearTramo(qA,qElevado,n1);
Q2 = crearTramo(qElevado,qIntermedio,n2);
Q3 = crearTramo(qIntermedio,qB,n3);

Q = [
    Q1;
    Q2(2:end,:);
    Q3(2:end,:)
];

tiempo = (0:size(Q,1)-1)'/frecuencia;

%% 11. COMPROBAR LIMITES

dentroTrayectoria = all( ...
    Q >= limites(:,1)' - 1e-6 & ...
    Q <= limites(:,2)' + 1e-6, ...
    'all');

if ~dentroTrayectoria
    error('La trayectoria supera los limites articulares.');
end

%% 12. CALCULAR RECORRIDO CARTESIANO

N = size(Q,1);

posiciones = zeros(N,3);
inclinacionPinza = zeros(N,1);

for i = 1:N

    Ttip = getTransform( ...
        robot,Q(i,:), ...
        'tool_tip','base_centrada');

    T4 = getTransform( ...
        robot,Q(i,:), ...
        '4_Link','base_centrada');

    pTip = Ttip(1:3,4);
    p4 = T4(1:3,4);

    posiciones(i,:) = pTip';

    v = pTip-p4;

    inclinacionPinza(i) = rad2deg( ...
        atan2(abs(v(3)),norm(v(1:2))));

end

%% 13. MOSTRAR RESULTADOS

fprintf('\n====================================\n');
fprintf('TRAYECTORIA VERSION 2\n');
fprintf('====================================\n');

fprintf('Duracion: %.1f segundos\n',tiempo(end));

fprintf('Muestras: %d\n',N);

fprintf('Inclinacion maxima pinza: %.2f grados\n', ...
    max(inclinacionPinza));

fprintf('Altura minima del centro de agarre: %.3f m\n', ...
    min(posiciones(:,3)));

fprintf('Altura maxima del centro de agarre: %.3f m\n', ...
    max(posiciones(:,3)));

fprintf('Error final respecto a B: %.2f mm\n', ...
    norm(posiciones(end,:)'-pB)*1000);

%% 14. ANIMACION 3D

fig = figure('Color','w', ...
    'Name','Robot2 - Trayectoria A-B V2');

ax = axes('Parent',fig);

for i = 1:3:N

    if ~isgraphics(fig)
        break;
    end

    show(robot,Q(i,:), ...
        'Parent',ax, ...
        'Visuals','on', ...
        'Frames','off', ...
        'PreservePlot',false);

    hold(ax,'on');
    grid(ax,'on');
    axis(ax,'equal');

    plot3(ax, ...
        posiciones(1:i,1), ...
        posiciones(1:i,2), ...
        posiciones(1:i,3), ...
        'g-', ...
        'LineWidth',2);

    plot3(ax,pA(1),pA(2),pA(3), ...
        'r*','MarkerSize',12,'LineWidth',2);

    plot3(ax,pB(1),pB(2),pB(3), ...
        'bo','MarkerSize',10,'MarkerFaceColor','b');

    plot3(ax, ...
        pIntermedio(1), ...
        pIntermedio(2), ...
        pIntermedio(3), ...
        'mo','MarkerSize',9,'MarkerFaceColor','m');

    xlabel(ax,'X [m]');
    ylabel(ax,'Y [m]');
    zlabel(ax,'Z [m]');

    xlim(ax,[-0.45 0.45]);
    ylim(ax,[-0.40 0.40]);
    zlim(ax,[-0.05 0.60]);

    view(ax,45,25);

    title(ax,sprintf( ...
        'Trayectoria V2 | t = %.2f s', ...
        tiempo(i)));

    drawnow;

end

%% 15. GRAFICA DE ANGULOS

figure('Color','w', ...
    'Name','Angulos articulares V2');

plot(tiempo,rad2deg(Q), ...
    'LineWidth',1.8);

grid on;

xlabel('Tiempo [s]');
ylabel('Angulo [grados]');

title('Movimiento articular con recogimiento');

legend( ...
    'Joint 1','Joint 2','Joint 3','Joint 4', ...
    'Location','best');

xline(tElevacion,'k--','Elevacion');
xline(tElevacion+tRecogimiento, ...
    'k--','Recogimiento');

%% 16. TRAYECTORIA CARTESIANA

figure('Color','w', ...
    'Name','Trayectoria cartesiana V2');

plot3( ...
    posiciones(:,1), ...
    posiciones(:,2), ...
    posiciones(:,3), ...
    'g-', ...
    'LineWidth',2);

hold on;
grid on;
axis equal;

plot3(pA(1),pA(2),pA(3), ...
    'r*','MarkerSize',14,'LineWidth',2);

plot3(pB(1),pB(2),pB(3), ...
    'bo','MarkerSize',10,'MarkerFaceColor','b');

plot3( ...
    pIntermedio(1), ...
    pIntermedio(2), ...
    pIntermedio(3), ...
    'mo','MarkerSize',10,'MarkerFaceColor','m');

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

title('Recorrido del centro de agarre - V2');

legend( ...
    'Trayectoria', ...
    'Punto A', ...
    'Punto B', ...
    'Recogimiento', ...
    'Location','best');

view(45,25);

%% 17. INCLINACION DE PINZA

figure('Color','w', ...
    'Name','Inclinacion de pinza V2');

plot(tiempo,inclinacionPinza, ...
    'LineWidth',2);

grid on;

xlabel('Tiempo [s]');
ylabel('Inclinacion [grados]');

title('Inclinacion durante el traslado - V2');

xline(tElevacion,'k--','Elevacion');
xline(tElevacion+tRecogimiento, ...
    'k--','Recogimiento');

%% 18. GUARDAR RESULTADOS

save('trayectoria_A_B_v2.mat', ...
    'Q', ...
    'tiempo', ...
    'posiciones', ...
    'inclinacionPinza', ...
    'qA', ...
    'qElevado', ...
    'qIntermedio', ...
    'qB', ...
    'pA', ...
    'pElevado', ...
    'pIntermedio', ...
    'pB');

fprintf('\nTrayectoria guardada en trayectoria_A_B_v2.mat\n');

fprintf('\nIMPORTANTE:\n');
fprintf('No se verificaron colisiones.\n');
fprintf('No se verificaron velocidades maximas reales.\n');
fprintf('No enviar a Gazebo todavia.\n');
