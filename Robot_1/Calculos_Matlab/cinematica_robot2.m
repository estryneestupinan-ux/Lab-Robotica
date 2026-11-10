
clc;
clear;
close all;

%% =========================================================
% CINEMATICA ROBOT2 - BASE CENTRADA Y TOOL_TIP
% ==========================================================

%% 1. IMPORTAR EL ROBOT

archivo = 'C:\Users\jandr\Downloads\robot2.0\robot2\urdf\robot2_base_centrada.urdf';

if ~isfile(archivo)
    error('No se encontro el archivo URDF: %s', archivo);
end

robot = importrobot(archivo);

robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

disp('Robot importado correctamente.');

%% 2. AGREGAR EL CENTRO DE AGARRE (TOOL_TIP)

% Crear el nuevo cuerpo
tool = rigidBody('tool_tip');

% Crear una articulacion fija
jointTool = rigidBodyJoint('joint_tool_tip', 'fixed');

% Distancia desde 4_Link al centro de agarre:
% 92.67 mm sobre el eje Z local negativo
distancia = 0.09267;

Ttool_local = trvec2tform([0 0 -distancia]);

setFixedTransform(jointTool, Ttool_local);

tool.Joint = jointTool;

% Unir el punto de agarre al ultimo eslabon
addBody(robot, tool, '4_Link');

disp('tool_tip agregado correctamente.');

%% 3. MOSTRAR ESTRUCTURA DEL ROBOT

disp(' ');
disp('ESTRUCTURA DEL ROBOT:');
showdetails(robot);

%% 4. CONFIGURACION INICIAL

q0 = homeConfiguration(robot);

disp(' ');
disp('Configuracion inicial:');
disp(q0);

%% 5. CINEMATICA DIRECTA DE 4_Link

T4 = getTransform(robot, q0, ...
    '4_Link', 'base_centrada');

P4 = T4(1:3,4);

disp(' ');
disp('Posicion del origen de 4_Link [m]:');
disp(P4);

%% 6. CINEMATICA DIRECTA DEL CENTRO DE AGARRE

Ttip = getTransform(robot, q0, ...
    'tool_tip', 'base_centrada');

Ptip = Ttip(1:3,4);

disp(' ');
disp('Posicion del centro de agarre [m]:');
disp(Ptip);

%% 7. VISUALIZACION DEL ROBOT COMPLETO

figure('Color','w', ...
    'Name','Robot completo con tool_tip');

show(robot, q0, ...
    'Visuals','on', ...
    'Frames','off');

hold on;
grid on;
axis equal;

% Origen de la base
plot3(0,0,0, ...
    'mp', ...
    'MarkerSize',18, ...
    'MarkerFaceColor','m', ...
    'LineWidth',2);

text(0,0,0.03, ...
    'BASE CENTRADA', ...
    'Color','m', ...
    'FontWeight','bold');

% Origen del ultimo eslabon
plot3(P4(1),P4(2),P4(3), ...
    'ro', ...
    'MarkerSize',10, ...
    'MarkerFaceColor','r');

% Centro de agarre
plot3(Ptip(1),Ptip(2),Ptip(3), ...
    'go', ...
    'MarkerSize',12, ...
    'MarkerFaceColor','g');

text(Ptip(1),Ptip(2),Ptip(3)+0.02, ...
    'TOOL TIP', ...
    'Color','g', ...
    'FontWeight','bold');

% Linea entre 4_Link y tool_tip
plot3([P4(1),Ptip(1)], ...
      [P4(2),Ptip(2)], ...
      [P4(3),Ptip(3)], ...
      'g--', ...
      'LineWidth',2);

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

title('Robot2 con centro de agarre');

view(45,25);
rotate3d on;

%% 8. VISTA AMPLIADA DE LA PINZA

figure('Color','w', ...
    'Name','Detalle del centro de agarre');

show(robot, q0, ...
    'Visuals','on', ...
    'Frames','off');

hold on;
grid on;
axis equal;

% Punto rojo: origen de 4_Link
plot3(P4(1),P4(2),P4(3), ...
    'ro', ...
    'MarkerSize',12, ...
    'MarkerFaceColor','r');

% Punto verde: centro de agarre
plot3(Ptip(1),Ptip(2),Ptip(3), ...
    'go', ...
    'MarkerSize',14, ...
    'MarkerFaceColor','g');

% Linea de referencia
plot3([P4(1),Ptip(1)], ...
      [P4(2),Ptip(2)], ...
      [P4(3),Ptip(3)], ...
      'g--', ...
      'LineWidth',2);

text(P4(1),P4(2),P4(3)+0.015, ...
    'ORIGEN 4_Link', ...
    'Color','r', ...
    'FontWeight','bold');

text(Ptip(1),Ptip(2),Ptip(3)+0.015, ...
    'CENTRO DE AGARRE', ...
    'Color','g', ...
    'FontWeight','bold');

% Limites de visualizacion
margen = 0.16;

xlim([P4(1)-margen, Ptip(1)+margen]);
ylim([P4(2)-margen, P4(2)+margen]);
zlim([P4(3)-margen, P4(3)+margen]);

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

title('Detalle de la pinza y tool_tip');

view(45,25);
rotate3d on;

%% 9. RESUMEN FINAL

disp(' ');
disp('============================================');
disp('        RESULTADOS FINALES');
disp('============================================');

fprintf('4_Link X: %.4f m\n', P4(1));
fprintf('4_Link Y: %.4f m\n', P4(2));
fprintf('4_Link Z: %.4f m\n', P4(3));

disp(' ');

fprintf('Tool_tip X: %.4f m\n', Ptip(1));
fprintf('Tool_tip Y: %.4f m\n', Ptip(2));
fprintf('Tool_tip Z: %.4f m\n', Ptip(3));

disp(' ');
fprintf('Distancia 4_Link a tool_tip: %.2f mm\n', ...
    norm(Ptip-P4)*1000);

disp('============================================');
disp('CINEMATICA DIRECTA COMPLETADA');
disp('============================================');
