clear; clc; close all;

%% ================= ROBOT =================
L1 = 13;        % eslabon 1 [cm]
L2 = 15;        % eslabon 2 [cm]
G  = 11;        % gripper (colineal con eslabon 2) [cm]
L2e = L2 + G;   

OFF1 = deg2rad(90);       
OFF2 = deg2rad(-120);     

TH2_MIN = -2;  
TH2_MAX =  2;  
TH1_HOME2 = deg2rad(0);
TH2_HOME2 = deg2rad(-90) - OFF2;

%% ================= PUNTOS =================

tabla  = { 'PLL (origen)',  [0;  26; 0];
           'PS (destino)',  [0; -26; 0] };
puntos = cell(size(tabla));
for k = 1:size(tabla,1)
    p = tabla{k,2};
    puntos(k,:) = { tabla{k,1}, [p(2); p(1)] };  
end

%% CINEMATICA INVERSA DE CADA PUNTO
res = struct('nombre',{},'th1',{},'th2',{},'codo',{});
for k = 1:size(puntos,1)
    nombre = puntos{k,1};  P = puntos{k,2};
    sols = ik(P(1), P(2), L1, L2e, OFF1, OFF2, TH2_MIN, TH2_MAX);

    fprintf('\n=== %s  (X=%.1f, Y=%.1f) ===\n', nombre, P(1), P(2));
    for s = sols
        fprintf(['  codo %+d : theta1 = %+8.2f deg   theta2 = %+8.2f deg (%+.4f rad)' ...
                 '   [rel. a prolongacion = %+7.2f deg]  %s\n'], ...
            s.codo, rad2deg(s.th1), rad2deg(s.th2), s.th2, rad2deg(s.q), ...
            ternario(s.ok,'OK','fuera de limite'));
    end

    validas = sols([sols.ok]);
    if isempty(validas)
        error('%s: ninguna solucion deja theta2 dentro de [%.2f, %.2f] rad.', nombre, TH2_MIN, TH2_MAX);
    end
    [~, i] = max([validas.holgura]);
    m = validas(i);
    [~, ~, Pe] = fk(m.th1, m.th2, L1, L2, G, OFF1, OFF2);
    fprintf('  -> ELEGIDA codo %+d : theta1 = %+.2f deg, theta2 = %+.2f deg | error FK = %.1e cm\n', ...
        m.codo, rad2deg(m.th1), rad2deg(m.th2), norm(Pe - P));

    res(end+1) = struct('nombre',nombre,'th1',m.th1,'th2',m.th2,'codo',m.codo); %#ok<SAGROW>
end

[H1, H2, H3] = fk(TH1_HOME2, TH2_HOME2, L1, L2, G, OFF1, OFF2);
fprintf('\n=== HOME 2 ===\n');
fprintf('  theta1 = %+.2f deg, theta2 = %+.2f deg\n', rad2deg(TH1_HOME2), rad2deg(TH2_HOME2));
fprintf('  codo   : (X=%7.2f, Y=%7.2f)\n', H1);
fprintf('  muneca : (X=%7.2f, Y=%7.2f)\n', H2);
fprintf('  punta  : (X=%7.2f, Y=%7.2f)\n', H3);

confs = { 'Home',        0,          0,          [0.2 0.2 0.2],  '--';
          'Home 2',      TH1_HOME2,  TH2_HOME2,  [0.55 0.2 0.7], '--';
          res(1).nombre, res(1).th1, res(1).th2, [0.85 0.1 0.1], '-';
          res(2).nombre, res(2).th1, res(2).th2, [0.1 0.4 0.85], '-' };
verde = [0.1 0.65 0.2];

figure('Color','w'); hold on; grid on; axis equal;
nc = size(confs,1);
h = gobjects(nc,1);  etiquetas = cell(nc,1);
for k = 1:nc
    [Q1, Q2, Q3] = fk(confs{k,2}, confs{k,3}, L1, L2, G, OFF1, OFF2);
    h(k) = plot([0 Q1(2) Q2(2)], [0 Q1(1) Q2(1)], confs{k,5}, 'Marker','o', ...
                'LineWidth',3, 'Color',confs{k,4}, 'MarkerFaceColor',confs{k,4});
    plot([Q2(2) Q3(2)], [Q2(1) Q3(1)], confs{k,5}, 'LineWidth',3, 'Color',verde);  % gripper
    etiquetas{k} = sprintf('%s: \\theta_1=%.1f°, \\theta_2=%.1f°', confs{k,1}, ...
                           rad2deg(confs{k,2}), rad2deg(confs{k,3}));
end
for k = 1:size(puntos,1)
    P = puntos{k,2};
    plot(P(2), P(1), 'kx', 'MarkerSize',16, 'LineWidth',2.5);
    text(P(2)+2, P(1), strtok(puntos{k,1}));
end
plot([0 (L1+L2e)*sin(OFF1)], [0 (L1+L2e)*cos(OFF1)], ':', 'Color',[0.6 0.6 0.6]);   % cero theta1
plot(0, 0, 'ks', 'MarkerSize',12, 'MarkerFaceColor','k');
legend(h, etiquetas, 'Location','southoutside');
xlabel('Y [cm] (horizontal)'); ylabel('X [cm] (vertical)');
title(sprintf('L1=%g, L2=%g, G=%g cm  (gripper en verde)', L1, L2, G));
lim = L1 + L2e + 5; xlim([-lim lim]); ylim([-lim lim]);

function sols = ik(x, y, L1, L2e, OFF1, OFF2, TH2_MIN, TH2_MAX)
    r2 = x^2 + y^2;  r = sqrt(r2);
    if r > (L1+L2e) || r < abs(L1-L2e)
        error('El punto (%.2f, %.2f) esta fuera del alcance.', x, y);
    end
    c = max(min((r2 - L1^2 - L2e^2)/(2*L1*L2e), 1), -1);
    sols = struct('codo',{},'th1',{},'th2',{},'q',{},'ok',{},'holgura',{});
    for codo = [+1 -1]
        q   = codo*acos(c);                      
        a1  = atan2(y, x) - atan2(L2e*sin(q), L1 + L2e*cos(q));
        th1 = wrapPi(a1 - OFF1);                
        th2 = wrapPi(q - OFF2);                 
        ok  = th2 >= TH2_MIN && th2 <= TH2_MAX;
        sols(end+1) = struct('codo',codo,'th1',th1,'th2',th2,'q',q,'ok',ok, ...
                             'holgura',min(th2 - TH2_MIN, TH2_MAX - th2)); 
    end
end

function [P1, P2, P3] = fk(th1, th2, L1, L2, G, OFF1, OFF2)
    a1 = OFF1 + th1;
    a2 = a1 + OFF2 + th2;
    u2 = [cos(a2); sin(a2)];
    P1 = L1*[cos(a1); sin(a1)];   
    P2 = P1 + L2*u2;            
    P3 = P2 + G*u2;               
end

function a = wrapPi(a)
    a = atan2(sin(a), cos(a));
end

function s = ternario(c, a, b)
    if c, s = a; else, s = b; end
end