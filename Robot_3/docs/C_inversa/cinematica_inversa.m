clear; clc; close all;

x_obj = -5;    % X vertical [cm]
y_obj = -29;    % Y horizontal [cm]


L1 = 15;         %eslabon 1 [cm]
L2 = 15;         %eslabon 2 [cm]
LIM2 = 2;        % limite segundo eslabon [rad]

mejor = [];  mejor_holgura = -inf;

for codo = [+1 -1]
    % distancia al objetivo
    r2 = x_obj^2 + y_obj^2;
    r  = sqrt(r2);

    % fuera de alcance
    if r > (L1+L2) || r < abs(L1-L2)
        continue;
    end

    % ---- angulo del codo (entre eslabon 1 y 2) ----
    c2 = (r2 - L1^2 - L2^2)/(2*L1*L2);
    if abs(c2) > 1, continue; end
    s2 = codo*sqrt(1 - c2^2);
    ang2 = atan2(s2, c2);   % angulo interno relativo

    % ---- angulo del primer eslabon ----
    ang1 = atan2(y_obj, x_obj) - atan2(L2*sin(ang2), L1 + L2*cos(ang2));

    % ---- convierto a mis convenciones ----
    theta1 = -ang1;     % + hacia la izquierda (sin limite)
    theta2 = ang2;      % + hacia la derecha  (limitado a +-2)

    % ---- filtro: SOLO el segundo eslabon tiene limite ----
    if abs(theta2) <= LIM2
        holgura = LIM2 - abs(theta2);
        if holgura > mejor_holgura
            mejor_holgura = holgura;
            mejor = struct('codo',codo,'theta1',theta1,'theta2',theta2, ...
                           'ang1',ang1,'ang2',ang2);
        end
    end
end

if isempty(mejor)
    error(['No se puede alcanzar el punto (%.2f, %.2f) ' ...
           'con el segundo eslabon dentro de [-2, 2].'], x_obj, y_obj);
end

% ==== Resultados ====
fprintf('\n=== SOLUCION ENCONTRADA ===\n');
fprintf('codo = %+d\n', mejor.codo);
fprintf('theta1 = %+8.4f rad  (%+7.2f deg)  [+ = izquierda]\n', ...
        mejor.theta1, rad2deg(mejor.theta1));
fprintf('theta2 = %+8.4f rad  (%+7.2f deg)  [+ = derecha]\n', ...
        mejor.theta2, rad2deg(mejor.theta2));
fprintf('margen al limite (eslabon 2) = %.3f rad\n', mejor_holgura);

% ==== Posiciones de cada articulacion ====
a1 = mejor.ang1;
a2 = mejor.ang1 + mejor.ang2;
P0 = [0;0];
P1 = P0 + L1*[cos(a1); sin(a1)];
P2 = P1 + L2*[cos(a2); sin(a2)];

fprintf('\n=== POSICIONES (X vert, Y horiz) [cm] ===\n');
fprintf('Base : (%7.2f, %7.2f)\n', P0(1), P0(2));
fprintf('Art1 : (%7.2f, %7.2f)\n', P1(1), P1(2));
fprintf('Efec.: (%7.2f, %7.2f)  <- objetivo (%.2f, %.2f)\n', ...
        P2(1), P2(2), x_obj, y_obj);

% ==== Grafica ====
figure; hold on; grid on; axis equal;
plot([P0(2) P1(2) P2(2)], [P0(1) P1(1) P2(1)], ...
     '-o','LineWidth',3,'MarkerFaceColor','r','Color',[0.85 0.1 0.1]);
plot(y_obj, x_obj, 'kx','MarkerSize',14,'LineWidth',2);
plot(0,0,'ks','MarkerSize',12,'MarkerFaceColor','k');
text(P1(2),P1(1),'  Art1');
text(P2(2),P2(1),'  Efector');
xlabel('Y [cm] (horizontal)'); ylabel('X [cm] (vertical)');
title(sprintf('IK | objetivo (%.1f, %.1f)  codo=%+d', ...
      x_obj, y_obj, mejor.codo));
lim = L1+L2+5; xlim([-lim lim]); ylim([-lim lim]);

